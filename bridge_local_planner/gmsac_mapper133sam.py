#(bridgelocalmap) ubuntu@DESKTOP-BEGQOKO:/mnt/c/Users/user/Documents/bridge_local_mapper$ python -m bridge_local_planner.gmsac_mapper133sam --cloud /mnt/c/Users/user/Documents/bridge_local_mapper/data/231031_HYU_Yang/zed_17-21-38_476.ply
#error on local mapper
#!/usr/bin/env python3
"""
Enhanced Floor Segmentation and Mapping System
Improved version with better error handling and optimizations
"""

from pathlib import Path
import sys
import numpy as np
import cv2 as cv
import open3d as o3d
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from datetime import datetime
import yaml
from scipy.ndimage import binary_fill_holes, gaussian_filter
from scipy.interpolate import griddata
import json
import logging
from typing import Dict, Tuple, Optional, List, Any
import warnings
warnings.filterwarnings('ignore')

# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format='[%(levelname)s] %(message)s'
)
logger = logging.getLogger(__name__)

# Dynamic imports with better error handling
def setup_paths():
    """Setup system paths for imports"""
    current_dir = Path(__file__).resolve().parent
    parent_dir = current_dir.parent
    grounding_dino_path = parent_dir / "GroundingDINO"
    
    paths_to_add = [str(parent_dir)]
    if grounding_dino_path.exists():
        paths_to_add.append(str(grounding_dino_path))
    
    for path in paths_to_add:
        if path not in sys.path:
            sys.path.append(path)

setup_paths()

# Import base mapper
try:
    from gtrack_mapper import GTrackMapper
except ImportError:
    try:
        from bridge_local_planner.gtrack_mapper import GTrackMapper
    except ImportError:
        logger.error("Could not import GTrackMapper. Please check installation.")
        sys.exit(1)

# Import detectors with error handling
GROUNDING_DINO = None
try:
    from ai_detectors import build as build_detector
    _cfg_file = Path(__file__).parent / "config" / "detector.yaml"
    if _cfg_file.exists():
        GROUNDING_DINO = build_detector(**yaml.safe_load(open(_cfg_file)))
        logger.info("GroundingDINO loaded successfully")
    else:
        logger.warning(f"GroundingDINO config not found at {_cfg_file}")
except Exception as e:
    logger.warning(f"Could not load GroundingDINO: {e}")

# Import SAM with error handling
SAM_AVAILABLE = False
try:
    from segment_anything import sam_model_registry, SamAutomaticMaskGenerator, SamPredictor
    import torch
    SAM_AVAILABLE = True
    logger.info("SAM available")
except ImportError:
    logger.warning("SAM not available. Install with: pip install segment-anything")


class OutputManager:
    """Manages output directory structure and file operations"""
    
    def __init__(self, base_path: Path, pointcloud_name: str):
        self.timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.folder_name = f"{pointcloud_name}_{self.timestamp}"
        self.root = Path(base_path) / "output" / self.folder_name
        
        self.dirs = {
            'root': self.root,
            'segmentation': self.root / '1_segmentation',
            'ground_detection': self.root / '2_ground_detection',
            'maps': self.root / '3_maps',
            'summary': self.root / '4_summary',
            'debug': self.root / '5_debug',
            'multi_res': self.root / '6_multi_resolution'
        }
        
        self._create_directories()
    
    def _create_directories(self):
        """Create all output directories"""
        for dir_path in self.dirs.values():
            dir_path.mkdir(parents=True, exist_ok=True)
        logger.info(f"Created output directory: {self.root}")
    
    def __getitem__(self, key: str) -> Path:
        return self.dirs[key]


class SAMProcessor:
    """Handles SAM model loading and floor segmentation"""
    
    def __init__(self, checkpoint_path: Optional[Path] = None):
        self.sam_model = None
        self.device = None
        self.available = SAM_AVAILABLE
        
        if self.available:
            self._load_model(checkpoint_path)
    
    def _load_model(self, checkpoint_path: Optional[Path] = None):
        """Load SAM model with automatic checkpoint detection"""
        if checkpoint_path is None:
            checkpoint_path = self._find_checkpoint()
        
        if checkpoint_path is None or not Path(checkpoint_path).exists():
            logger.error("SAM checkpoint not found. Download from:")
            logger.error("  wget https://dl.fbaipublicfiles.com/segment_anything/sam_vit_b_4b8939.pth")
            self.available = False
            return
        
        logger.info(f"Loading SAM from: {checkpoint_path}")
        self.device = "cuda" if torch.cuda.is_available() else "cpu"
        
        # Determine model type
        checkpoint_str = str(checkpoint_path)
        if "vit_h" in checkpoint_str:
            model_type = "vit_h"
        elif "vit_l" in checkpoint_str:
            model_type = "vit_l"
        else:
            model_type = "vit_b"
        
        try:
            self.sam_model = sam_model_registry[model_type](checkpoint=checkpoint_str)
            self.sam_model.to(device=self.device)
            logger.info(f"SAM model loaded on {self.device}")
        except Exception as e:
            logger.error(f"Failed to load SAM model: {e}")
            self.available = False
    
    def _find_checkpoint(self) -> Optional[Path]:
        """Find SAM checkpoint in common locations"""
        search_paths = [
            Path(__file__).parent.parent / "weights" / "sam_vit_b_4b8939.pth",
            Path(__file__).parent.parent / "weights" / "sam_vit_h_4b8939.pth",
            Path(__file__).parent.parent / "weights" / "sam_vit_l_0b3195.pth",
            Path.home() / ".cache" / "sam" / "sam_vit_b_4b8939.pth",
        ]
        
        for path in search_paths:
            if path.exists():
                return path
        return None
    
    def segment_floor(self, image: np.ndarray, output_dir: Optional[Path] = None) -> Optional[np.ndarray]:
        """Advanced floor segmentation using multiple SAM strategies"""
        if not self.available or self.sam_model is None:
            return None
        
        logger.info("Running advanced SAM floor segmentation...")
        
        h, w = image.shape[:2]
        best_floor_mask = None
        best_score = 0
        
        # Try different strategies
        strategies = [
            self._automatic_mask_generation,
            self._box_prompt_strategy,
            self._grid_point_strategy
        ]
        
        for strategy in strategies:
            try:
                mask, score = strategy(image, h, w)
                if mask is not None and score > best_score:
                    best_score = score
                    best_floor_mask = mask
                    if score > 0.7:  # Good enough, stop trying
                        break
            except Exception as e:
                logger.warning(f"Strategy failed: {e}")
                if self.device == "cuda":
                    torch.cuda.empty_cache()
        
        # Post-process the mask
        if best_floor_mask is not None:
            best_floor_mask = self._post_process_mask(best_floor_mask, h, w)
            
            if output_dir:
                self._save_visualization(image, best_floor_mask, best_score, output_dir)
        
        return best_floor_mask
    
    def _automatic_mask_generation(self, image: np.ndarray, h: int, w: int) -> Tuple[Optional[np.ndarray], float]:
        """Strategy 1: Automatic mask generation"""
        mask_generator = SamAutomaticMaskGenerator(
            self.sam_model,
            points_per_side=16,
            pred_iou_thresh=0.86,
            stability_score_thresh=0.92,
            min_mask_region_area=int(h * w * 0.01),
            crop_n_layers=0,
            crop_n_points_downscale_factor=1,
        )
        
        # Handle potential CUDA OOM
        try:
            masks = mask_generator.generate(image)
        except RuntimeError as e:
            if "out of memory" in str(e) and self.device == "cuda":
                logger.warning("CUDA OOM, falling back to CPU...")
                torch.cuda.empty_cache()
                self.sam_model.cpu()
                masks = mask_generator.generate(image)
                self.sam_model.to(self.device)
            else:
                raise e
        
        logger.info(f"SAM generated {len(masks)} masks")
        
        # Score masks for floor likelihood
        best_mask = None
        best_score = 0
        
        for mask_data in masks:
            mask = mask_data['segmentation']
            score = self._score_floor_mask(mask, mask_data, h, w)
            
            if score > best_score:
                best_score = score
                best_mask = mask
        
        return best_mask, best_score
    
    def _box_prompt_strategy(self, image: np.ndarray, h: int, w: int) -> Tuple[Optional[np.ndarray], float]:
        """Strategy 2: Box prompt for floor region"""
        predictor = SamPredictor(self.sam_model)
        predictor.set_image(image)
        
        # Define floor region box (bottom 2/3, full width)
        floor_box = np.array([20, int(h * 0.3), w - 20, h - 20])
        
        masks, scores, _ = predictor.predict(
            box=floor_box,
            multimask_output=True
        )
        
        if len(masks) == 0:
            return None, 0
        
        # Select best mask
        best_idx = 0
        best_score = 0
        
        for i, mask in enumerate(masks):
            # Evaluate floor properties
            bottom_coverage = np.mean(mask[-int(h*0.2):, :])
            h_continuity = np.mean(np.sum(mask, axis=0) > h * 0.3)
            floor_score = scores[i] * 0.5 + bottom_coverage * 0.3 + h_continuity * 0.2
            
            if floor_score > best_score:
                best_score = floor_score
                best_idx = i
        
        return masks[best_idx], best_score
    
    def _grid_point_strategy(self, image: np.ndarray, h: int, w: int) -> Tuple[Optional[np.ndarray], float]:
        """Strategy 3: Grid point prompts"""
        predictor = SamPredictor(self.sam_model)
        predictor.set_image(image)
        
        # Create point grid
        point_coords = []
        point_labels = []
        
        # Positive points (floor) - denser at bottom
        for y_ratio in np.linspace(0.5, 0.95, 6):
            for x_ratio in np.linspace(0.1, 0.9, 8):
                point_coords.append([int(x_ratio * w), int(y_ratio * h)])
                point_labels.append(1)
        
        # Negative points (not floor) - upper region
        for y_ratio in np.linspace(0.05, 0.3, 3):
            for x_ratio in np.linspace(0.1, 0.9, 5):
                point_coords.append([int(x_ratio * w), int(y_ratio * h)])
                point_labels.append(0)
        
        masks, scores, _ = predictor.predict(
            point_coords=np.array(point_coords),
            point_labels=np.array(point_labels),
            multimask_output=True,
        )
        
        if len(masks) == 0:
            return None, 0
        
        best_idx = np.argmax(scores)
        return masks[best_idx], scores[best_idx]
    
    def _score_floor_mask(self, mask: np.ndarray, mask_data: dict, h: int, w: int) -> float:
        """Score a mask for floor likelihood"""
        area = mask_data['area']
        y_coords, x_coords = np.where(mask)
        
        if len(y_coords) == 0:
            return 0
        
        center_y = np.mean(y_coords)
        coverage = area / (h * w)
        
        # Scoring factors
        lower_bias = (center_y / h) ** 2
        area_score = min(coverage * 2, 1.0)
        width_ratio = (np.max(x_coords) - np.min(x_coords)) / w if len(x_coords) > 0 else 0
        continuity_score = mask_data.get('stability_score', 0.5)
        bottom_edge_score = np.sum(mask[-int(h*0.1):, :]) / (w * int(h*0.1)) if h > 10 else 0
        
        # Combined score
        score = (lower_bias * 0.3 + 
                area_score * 0.25 + 
                width_ratio * 0.2 + 
                continuity_score * 0.15 + 
                bottom_edge_score * 0.1)
        
        return score
    
    def _post_process_mask(self, mask: np.ndarray, h: int, w: int) -> np.ndarray:
        """Post-process the mask with morphological operations"""
        mask = mask.astype(np.uint8)
        
        # Morphological operations
        kernel_size = max(5, int(min(h, w) * 0.01))
        kernel = np.ones((kernel_size, kernel_size), np.uint8)
        
        # Close gaps
        mask = cv.morphologyEx(mask, cv.MORPH_CLOSE, kernel)
        
        # Remove small holes
        mask = binary_fill_holes(mask).astype(np.uint8)
        
        # Keep only largest connected component
        contours, _ = cv.findContours(mask, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
        if len(contours) > 1:
            largest_contour = max(contours, key=cv.contourArea)
            mask = np.zeros_like(mask)
            cv.drawContours(mask, [largest_contour], -1, 1, -1)
        
        return mask
    
    def _save_visualization(self, image: np.ndarray, mask: np.ndarray, score: float, output_dir: Path):
        """Save comprehensive visualization"""
        fig, axes = plt.subplots(2, 3, figsize=(18, 12))
        
        # Original image
        axes[0, 0].imshow(cv.cvtColor(image, cv.COLOR_BGR2RGB))
        axes[0, 0].set_title('Original Image')
        axes[0, 0].axis('off')
        
        # Floor mask
        axes[0, 1].imshow(mask, cmap='gray')
        axes[0, 1].set_title(f'Floor Mask (Score: {score:.2f})')
        axes[0, 1].axis('off')
        
        # Overlay
        overlay = image.copy()
        overlay[mask.astype(bool)] = [0, 255, 0]
        result = cv.addWeighted(image, 0.6, overlay, 0.4, 0)
        
        # Add contour
        contours, _ = cv.findContours(mask, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
        cv.drawContours(result, contours, -1, (0, 255, 0), 3)
        
        axes[0, 2].imshow(cv.cvtColor(result, cv.COLOR_BGR2RGB))
        axes[0, 2].set_title('Floor Overlay')
        axes[0, 2].axis('off')
        
        # Statistics
        h, w = image.shape[:2]
        coverage = np.sum(mask) / (h * w) * 100
        
        axes[1, 0].axis('off')
        stats_text = f"Coverage: {coverage:.1f}%\nPixels: {np.sum(mask):,}"
        axes[1, 0].text(0.5, 0.5, stats_text, transform=axes[1, 0].transAxes,
                       fontsize=14, ha='center', va='center',
                       bbox=dict(boxstyle="round,pad=0.5", facecolor="lightgray"))
        
        # Height profile
        y_profile = np.mean(mask, axis=1)
        axes[1, 1].plot(y_profile, range(h))
        axes[1, 1].invert_yaxis()
        axes[1, 1].set_xlabel('Floor Coverage')
        axes[1, 1].set_ylabel('Image Height')
        axes[1, 1].set_title('Vertical Profile')
        axes[1, 1].grid(True, alpha=0.3)
        
        # Width profile
        x_profile = np.mean(mask, axis=0)
        axes[1, 2].plot(x_profile)
        axes[1, 2].set_xlabel('Image Width')
        axes[1, 2].set_ylabel('Floor Coverage')
        axes[1, 2].set_title('Horizontal Profile')
        axes[1, 2].grid(True, alpha=0.3)
        
        plt.tight_layout()
        plt.savefig(output_dir / 'sam_analysis.png', dpi=150, bbox_inches='tight')
        plt.close()
        
        # Save individual outputs
        cv.imwrite(str(output_dir / 'floor_mask.png'), mask * 255)
        cv.imwrite(str(output_dir / 'floor_overlay.png'), result)
        
        logger.info(f"SAM floor coverage: {coverage:.1f}%")


class ObjectDetector:
    """Handles object detection using GroundingDINO"""
    
    def __init__(self):
        self.detector = GROUNDING_DINO
        self.available = GROUNDING_DINO is not None
    
    def detect_objects(self, image: np.ndarray, output_dir: Optional[Path] = None) -> Optional[np.ndarray]:
        """Detect objects with filtering"""
        if not self.available:
            logger.warning("GroundingDINO not available")
            return None
        
        logger.info("Running GroundingDINO object detection...")
        
        try:
            det_out = self.detector.detect(image)
        except Exception as e:
            logger.error(f"GroundingDINO detection failed: {e}")
            return None
        
        if det_out is None or "boxes" not in det_out:
            return None
        
        boxes = det_out["boxes"]
        logger.info(f"GroundingDINO found {len(boxes)} objects")
        
        # Convert to pixel coordinates and filter
        H, W = image.shape[:2]
        pixel_boxes = boxes * np.array([W, H, W, H])
        fixed_boxes = self._filter_boxes(pixel_boxes, H, W)
        
        # Save visualization
        if output_dir and len(fixed_boxes) > 0:
            self._save_visualization(image, fixed_boxes, output_dir)
        
        return fixed_boxes
    
    def _filter_boxes(self, boxes: np.ndarray, H: int, W: int) -> np.ndarray:
        """Filter and fix bounding boxes"""
        fixed_boxes = []
        
        for box in boxes:
            x1, y1, x2, y2 = box
            
            # Ensure proper order
            x1, x2 = min(x1, x2), max(x1, x2)
            y1, y2 = min(y1, y2), max(y1, y2)
            
            # Clamp to image boundaries
            x1 = np.clip(x1, 0, W-1)
            x2 = np.clip(x2, 0, W-1)
            y1 = np.clip(y1, 0, H-1)
            y2 = np.clip(y2, 0, H-1)
            
            # Filter criteria
            width = x2 - x1
            height = y2 - y1
            area = width * height
            
            # Skip too small or too large boxes
            if width < 20 or height < 20:
                continue
            if area > 0.5 * H * W:  # Skip if larger than half image
                continue
            
            fixed_boxes.append([x1, y1, x2, y2])
        
        return np.array(fixed_boxes) if len(fixed_boxes) > 0 else np.array([])
    
    def _save_visualization(self, image: np.ndarray, boxes: np.ndarray, output_dir: Path):
        """Save object detection visualization"""
        vis_img = image.copy()
        colors = [(0, 255, 0), (255, 0, 0), (0, 0, 255), (255, 255, 0), (255, 0, 255)]
        
        for i, (x1, y1, x2, y2) in enumerate(boxes.astype(int)):
            color = colors[i % len(colors)]
            cv.rectangle(vis_img, (x1, y1), (x2, y2), color, 2)
            cv.putText(vis_img, f"Obj {i}", (x1, max(y1-5, 20)), 
                      cv.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)
        
        cv.imwrite(str(output_dir / 'detected_objects.png'), vis_img)


class EnhancedFloorMapper(GTrackMapper):
    """Enhanced mapper with improved features and error handling"""
    
    def __init__(self, map_x_length: float = 10., map_y_length: float = 10., map_cellsize: float = 0.1):
        super().__init__(map_x_length, map_y_length, map_cellsize)
        
        # Enhanced parameters
        self.params.update({
            'ransac_num_iters': 2000,
            'ransac_threshold': 0.15,
            'plane_z_threshold': 0.95,
            'plane_max_height': 3.0,
            'use_sam': True,
            'use_grounding_dino': True,
            'use_height_filter': True,
            'floor_height_threshold': 0.5,
            'filter_combination': 'sam_primary',
            'multi_resolution': True,
            'camera_fov_degrees': 70,
            'pts_sampling_step': 1
        })
        
        # Multi-resolution support
        self.resolutions = [0.05, 0.1, 0.2]
        self.multi_res_maps = {}
        
        # Semantic labels
        self.semantic_labels = {
            'unknown': 0,
            'floor': 1,
            'obstacle': 2,
            'wall': 3,
            'dynamic': 4
        }
        
        # State
        self.floor_mask = None
        self.object_boxes = None
        self.output_manager = None
        self.debug_info = {}
        
        # Processors
        self.sam_processor = None
        self.object_detector = None
    
    def initialize_processors(self, sam_checkpoint: Optional[Path] = None):
        """Initialize SAM and object detection processors"""
        if self.params['use_sam']:
            self.sam_processor = SAMProcessor(sam_checkpoint)
            if not self.sam_processor.available:
                self.params['use_sam'] = False
                logger.warning("SAM disabled due to initialization failure")
        
        if self.params['use_grounding_dino']:
            self.object_detector = ObjectDetector()
            if not self.object_detector.available:
                self.params['use_grounding_dino'] = False
                logger.warning("GroundingDINO disabled due to initialization failure")
    
    def process_image(self, image: np.ndarray):
        """Process image for floor and object detection"""
        if image is None:
            return
        
        self.current_rgb_frame = image
        
        # SAM floor segmentation
        if self.params['use_sam'] and self.sam_processor:
            self.floor_mask = self.sam_processor.segment_floor(
                image, 
                self.output_manager['segmentation'] if self.output_manager else None
            )
            
            # Adjust parameters based on mask quality
            if self.floor_mask is not None:
                self._adjust_parameters_from_mask()
        
        # Object detection
        if self.params['use_grounding_dino'] and self.object_detector:
            self.object_boxes = self.object_detector.detect_objects(
                image,
                self.output_manager['segmentation'] if self.output_manager else None
            )
    
    def _adjust_parameters_from_mask(self):
        """Dynamically adjust parameters based on mask quality"""
        if self.floor_mask is None:
            return
        
        coverage = np.sum(self.floor_mask) / self.floor_mask.size
        
        if coverage < 0.2:
            logger.info(f"Low floor coverage ({coverage:.1%}), adjusting parameters")
            self.params['floor_height_threshold'] = 1.0
            self.params['ransac_threshold'] = 0.2
        elif coverage > 0.6:
            logger.info(f"High floor coverage ({coverage:.1%}), using strict parameters")
            self.params['floor_height_threshold'] = 0.3
            self.params['ransac_threshold'] = 0.1
    
    def apply_pointcloud(self, pts: np.ndarray) -> np.ndarray:
        """Enhanced point cloud processing"""
        logger.info(f"Processing {len(pts)} points...")
        
        # Sampling
        step = self.params.get('pts_sampling_step', 1)
        if step > 1:
            pts = pts[::step]
            logger.info(f"Sampled to {len(pts)} points (step={step})")
        
        # Transform to robot frame
        pts_robot = self._transform_to_robot_frame(pts)
        
        # Apply filters
        floor_points = self._apply_filters(pts_robot)
        
        # RANSAC ground detection
        if len(floor_points) > 100:
            floor_points = self._ransac_ground_detection(floor_points)
        
        # Store debug info
        self._store_debug_info(pts, pts_robot, floor_points)
        
        # Save visualizations
        if self.output_manager:
            self._save_ground_detection_viz()
        
        # Multi-resolution mapping
        if self.params['multi_resolution'] and len(floor_points) > 0:
            self._create_multi_resolution_maps(floor_points)
        
        # Call parent mapper
        return super().apply_pointcloud(floor_points)
    
    def _transform_to_robot_frame(self, pts: np.ndarray) -> np.ndarray:
        """Transform points to robot frame with validation"""
        try:
            if hasattr(self, 'sensor2robot_T'):
                pts_robot = pts @ self.sensor2robot_T[:3,:3].T + self.sensor2robot_T[:3,-1]
            else:
                logger.warning("No sensor2robot_T transformation found, using identity")
                pts_robot = pts
        except Exception as e:
            logger.error(f"Transformation failed: {e}, using identity")
            pts_robot = pts
        
        return pts_robot
    
    def _apply_filters(self, pts_robot: np.ndarray) -> np.ndarray:
        """Apply all filtering strategies"""
        # Initialize masks
        height_mask = np.ones(len(pts_robot), dtype=bool)
        sam_mask = np.ones(len(pts_robot), dtype=bool)
        object_mask = np.ones(len(pts_robot), dtype=bool)
        
        # Height-based filtering
        if self.params['use_height_filter']:
            height_mask = self._apply_height_filter(pts_robot)
        
        # SAM-based filtering
        if self.params['use_sam'] and self.floor_mask is not None:
            sam_mask = self._apply_sam_filter(pts_robot)
        
        # Object removal
        if self.params['use_grounding_dino'] and self.object_boxes is not None:
            object_mask = self._apply_object_filter(pts_robot)
        
        # Combine masks
        final_mask = self._combine_masks(height_mask, sam_mask, object_mask)
        
        # Store for debugging
        self.debug_info.update({
            'height_mask': height_mask,
            'sam_mask': sam_mask,
            'object_mask': object_mask,
            'final_mask': final_mask
        })
        
        floor_points = pts_robot[final_mask]
        logger.info(f"Final filtering: {len(floor_points)} floor points ({len(floor_points)/len(pts_robot)*100:.1f}%)")
        
        return floor_points
    
    def _apply_height_filter(self, pts_robot: np.ndarray) -> np.ndarray:
        """Apply height-based filtering"""
        valid_z = ~np.isnan(pts_robot[:, 2]) & ~np.isinf(pts_robot[:, 2])
        
        if np.sum(valid_z) == 0:
            logger.warning("No valid Z values found, disabling height filter")
            return np.ones(len(pts_robot), dtype=bool)
        
        z_min = np.min(pts_robot[valid_z, 2])
        threshold = self.params['floor_height_threshold']
        height_mask = pts_robot[:, 2] < (z_min + threshold)
        
        logger.info(f"Height filter: {np.sum(height_mask)}/{len(pts_robot)} points below {z_min + threshold:.2f}m")
        return height_mask
    
    def _apply_sam_filter(self, pts_robot: np.ndarray) -> np.ndarray:
        """Apply SAM-based filtering"""
        if not hasattr(self, 'current_rgb_frame'):
            return np.ones(len(pts_robot), dtype=bool)
        
        K = self._estimate_camera_intrinsics(self.current_rgb_frame.shape)
        u, v, valid = self._project_points_safe(pts_robot, K, self.current_rgb_frame.shape)
        
        sam_mask = np.zeros(len(pts_robot), dtype=bool)
        h, w = self.floor_mask.shape
        
        for i in np.where(valid)[0]:
            if 0 <= v[i] < h and 0 <= u[i] < w:
                if self.floor_mask[v[i], u[i]]:
                    sam_mask[i] = True
        
        logger.info(f"SAM filter: {np.sum(sam_mask)}/{len(pts_robot)} points on floor mask")
        return sam_mask
    
    def _apply_object_filter(self, pts_robot: np.ndarray) -> np.ndarray:
        """Apply object removal filter"""
        if not hasattr(self, 'current_rgb_frame') or len(self.object_boxes) == 0:
            return np.ones(len(pts_robot), dtype=bool)
        
        K = self._estimate_camera_intrinsics(self.current_rgb_frame.shape)
        u, v, valid = self._project_points_safe(pts_robot, K, self.current_rgb_frame.shape)
        
        object_mask = np.ones(len(pts_robot), dtype=bool)
        
        for box in self.object_boxes:
            x1, y1, x2, y2 = box.astype(int)
            in_box = valid & (u >= x1) & (u <= x2) & (v >= y1) & (v <= y2)
            object_mask[in_box] = False
        
        logger.info(f"Object filter: {np.sum(object_mask)}/{len(pts_robot)} points outside objects")
        return object_mask
    
    def _combine_masks(self, height_mask: np.ndarray, sam_mask: np.ndarray, 
                      object_mask: np.ndarray) -> np.ndarray:
        """Smart combination of filter masks"""
        filter_mode = self.params['filter_combination']
        
        if filter_mode == 'sam_primary':
            if np.sum(sam_mask) > 100:
                final_mask = sam_mask & object_mask
                if np.sum(final_mask) < 100:
                    final_mask = sam_mask
            else:
                final_mask = height_mask & object_mask
        
        elif filter_mode == 'adaptive':
            masks_available = {
                'height': np.sum(height_mask) > 0,
                'sam': np.sum(sam_mask) > 0,
                'object': np.sum(object_mask) < len(object_mask)
            }
            
            if masks_available['sam']:
                final_mask = sam_mask
                if masks_available['object']:
                    final_mask &= object_mask
            elif masks_available['height']:
                final_mask = height_mask
                if masks_available['object']:
                    final_mask &= object_mask
            else:
                final_mask = np.ones(len(height_mask), dtype=bool)
        
        else:  # 'all'
            final_mask = height_mask & sam_mask & object_mask
        
        return final_mask
    
    def _ransac_ground_detection(self, floor_points: np.ndarray) -> np.ndarray:
        """RANSAC with multiple attempts"""
        best_plane = None
        best_inliers = None
        best_score = 0
        
        # Try different thresholds
        for threshold_mult in [1.0, 1.5, 2.0]:
            threshold = self.params['ransac_threshold'] * threshold_mult
            plane, inliers = self._detect_ground_plane(floor_points, threshold)
            
            if plane is not None:
                score = np.sum(inliers) / len(floor_points)
                if score > best_score:
                    best_score = score
                    best_plane = plane
                    best_inliers = inliers
                    
                    if score > 0.8:  # Good enough
                        break
        
        if best_plane is not None:
            logger.info(f"RANSAC: {np.sum(best_inliers)} inliers ({best_score*100:.1f}%)")
            return floor_points[best_inliers]
        else:
            logger.warning("RANSAC failed to find ground plane")
            return floor_points
    
    def _detect_ground_plane(self, pts: np.ndarray, threshold: float) -> Tuple[Optional[np.ndarray], np.ndarray]:
        """RANSAC ground plane detection"""
        if len(pts) < 3:
            return None, np.array([])
        
        best_plane, best_mask = None, None
        max_inliers = 0
        
        num_iters = min(self.params['ransac_num_iters'], len(pts) // 10)
        
        for _ in range(num_iters):
            # Sample 3 points
            idx = np.random.choice(len(pts), 3, replace=False)
            p1, p2, p3 = pts[idx]
            
            # Compute plane normal
            v1 = p2 - p1
            v2 = p3 - p1
            normal = np.cross(v1, v2)
            
            if np.linalg.norm(normal) < 1e-6:
                continue
            
            normal /= np.linalg.norm(normal)
            
            # Ensure upward normal
            if normal[2] < 0:
                normal = -normal
            
            # Check angle constraint
            if normal[2] < self.params['plane_z_threshold']:
                continue
            
            # Plane equation
            d = -np.dot(normal, p1)
            plane = np.append(normal, d)
            
            # Count inliers
            distances = np.abs(pts @ normal + d)
            mask = distances < threshold
            n_inliers = np.sum(mask)
            
            if n_inliers > max_inliers:
                max_inliers = n_inliers
                best_plane = plane
                best_mask = mask
        
        return best_plane, best_mask
    
    def _estimate_camera_intrinsics(self, image_shape: Tuple[int, int], fov_deg: float = None) -> np.ndarray:
        """Estimate camera intrinsics"""
        if fov_deg is None:
            fov_deg = self.params['camera_fov_degrees']
        
        H, W = image_shape[:2]
        fov_rad = np.deg2rad(fov_deg)
        fx = W / (2 * np.tan(fov_rad / 2))
        fy = fx
        cx = W / 2
        cy = H / 2
        
        K = np.array([[fx, 0, cx], [0, fy, cy], [0, 0, 1]])
        
        # Validate
        if fx <= 0 or fy <= 0:
            logger.warning("Invalid camera intrinsics, using default")
            K = np.array([[W, 0, W/2], [0, W, H/2], [0, 0, 1]])
        
        return K
    
    def _project_points_safe(self, pts: np.ndarray, K: np.ndarray, 
                           image_shape: Tuple[int, int]) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
        """Safe point projection with validation"""
        H, W = image_shape[:2]
        
        # Transform to camera frame
        if hasattr(self, 'sensor2robot_T'):
            try:
                robot2sensor = np.linalg.inv(self.sensor2robot_T)
                pts_cam = pts @ robot2sensor[:3,:3].T + robot2sensor[:3,-1]
            except:
                logger.warning("Invalid transformation, using identity")
                pts_cam = pts
        else:
            pts_cam = pts
        
        # Filter valid points
        valid_mask = np.ones(len(pts), dtype=bool)
        valid_mask &= ~np.any(np.isnan(pts_cam) | np.isinf(pts_cam), axis=1)
        valid_mask &= (pts_cam[:, 2] > 0.1) & (pts_cam[:, 2] < 100)
        
        if np.sum(valid_mask) == 0:
            return np.full(len(pts), -1), np.full(len(pts), -1), np.zeros(len(pts), dtype=bool)
        
        # Project valid points
        valid_pts = pts_cam[valid_mask]
        uvw = K @ valid_pts.T
        
        # Safe division
        z_safe = np.maximum(uvw[2], 0.001)
        u = uvw[0] / z_safe
        v = uvw[1] / z_safe
        
        # Bounds check
        in_bounds = (u >= 0) & (u < W) & (v >= 0) & (v < H)
        
        # Output arrays
        u_out = np.full(len(pts), -1, dtype=int)
        v_out = np.full(len(pts), -1, dtype=int)
        valid_out = np.zeros(len(pts), dtype=bool)
        
        # Fill valid projections
        valid_indices = np.where(valid_mask)[0]
        good_indices = valid_indices[in_bounds]
        
        if len(good_indices) > 0:
            u_out[good_indices] = np.clip(u[in_bounds], 0, W-1).astype(int)
            v_out[good_indices] = np.clip(v[in_bounds], 0, H-1).astype(int)
            valid_out[good_indices] = True
        
        return u_out, v_out, valid_out
    
    def _store_debug_info(self, pts_original: np.ndarray, pts_robot: np.ndarray, floor_points: np.ndarray):
        """Store debug information"""
        self.debug_info.update({
            'total_points': len(pts_original),
            'floor_points': floor_points,
            'pts_robot': pts_robot,
            'filter_mode': self.params['filter_combination']
        })
    
    def _create_multi_resolution_maps(self, floor_points: np.ndarray):
        """Create maps at multiple resolutions"""
        if not self.output_manager:
            return
        
        logger.info("Creating multi-resolution maps...")
        
        for res in self.resolutions:
            # Create map at this resolution
            nx = int(self.map_x_length / res)
            ny = int(self.map_y_length / res)
            
            # Initialize maps
            height_map = np.full((ny, nx), np.nan)
            count_map = np.zeros((ny, nx))
            
            # Compute grid indices
            x_idx = ((floor_points[:, 0] + self.map_x_length/2) / res).astype(int)
            y_idx = ((floor_points[:, 1] + self.map_y_length/2) / res).astype(int)
            
            # Filter valid indices
            valid = (x_idx >= 0) & (x_idx < nx) & (y_idx >= 0) & (y_idx < ny)
            
            # Accumulate heights
            for i in np.where(valid)[0]:
                xi, yi = x_idx[i], y_idx[i]
                if np.isnan(height_map[yi, xi]):
                    height_map[yi, xi] = floor_points[i, 2]
                else:
                    n = count_map[yi, xi]
                    height_map[yi, xi] = (height_map[yi, xi] * n + floor_points[i, 2]) / (n + 1)
                count_map[yi, xi] += 1
            
            # Interpolate missing values
            if np.sum(~np.isnan(height_map)) > 10:
                height_map = self._interpolate_map(height_map)
            
            # Store
            self.multi_res_maps[f'height_{res}'] = height_map
            self.multi_res_maps[f'count_{res}'] = count_map
            
            # Save visualization
            self._save_resolution_map(height_map, res)
    
    def _interpolate_map(self, map_data: np.ndarray) -> np.ndarray:
        """Interpolate missing values in map"""
        h, w = map_data.shape
        
        # Get valid points
        valid_mask = ~np.isnan(map_data)
        if np.sum(valid_mask) < 3:
            return map_data
        
        # Create coordinate grids
        y, x = np.mgrid[0:h, 0:w]
        
        # Get valid coordinates and values
        points = np.column_stack((x[valid_mask], y[valid_mask]))
        values = map_data[valid_mask]
        
        # Interpolate
        try:
            interpolated = griddata(points, values, (x, y), method='linear')
            
            # Fill remaining NaN with nearest
            if np.any(np.isnan(interpolated)):
                interpolated_nearest = griddata(points, values, (x, y), method='nearest')
                interpolated[np.isnan(interpolated)] = interpolated_nearest[np.isnan(interpolated)]
            
            return interpolated
        except Exception as e:
            logger.warning(f"Interpolation failed: {e}")
            return map_data
    
    def _save_resolution_map(self, height_map: np.ndarray, resolution: float):
        """Save a single resolution map"""
        fig, ax = plt.subplots(figsize=(8, 8))
        extent = [-self.map_x_length/2, self.map_x_length/2,
                  -self.map_y_length/2, self.map_y_length/2]
        
        im = ax.imshow(height_map, extent=extent, origin='lower', cmap='terrain')
        ax.set_title(f'Height Map (Resolution: {resolution}m)')
        ax.set_xlabel('X (m)')
        ax.set_ylabel('Y (m)')
        plt.colorbar(im, ax=ax, label='Height (m)')
        
        plt.savefig(self.output_manager['multi_res'] / f'height_map_{resolution}m.png', 
                   dpi=150, bbox_inches='tight')
        plt.close()
    
    def _save_ground_detection_viz(self):
        """Save ground detection visualization"""
        output_dir = self.output_manager['ground_detection']
        
        # Create comprehensive figure
        fig = plt.figure(figsize=(20, 16))
        
        # 3D visualization
        ax1 = fig.add_subplot(2, 3, 1, projection='3d')
        self._plot_3d_points(ax1)
        
        # Filter analysis
        ax2 = fig.add_subplot(2, 3, 2)
        self._plot_filter_analysis(ax2)
        
        # Height distribution
        ax3 = fig.add_subplot(2, 3, 3)
        self._plot_height_distribution(ax3)
        
        # XY distribution
        ax4 = fig.add_subplot(2, 3, 4)
        self._plot_xy_distribution(ax4)
        
        # Filter combination
        ax5 = fig.add_subplot(2, 3, 5)
        self._plot_filter_combination(ax5)
        
        # Statistics
        ax6 = fig.add_subplot(2, 3, 6)
        self._plot_statistics(ax6)
        
        plt.tight_layout()
        plt.savefig(output_dir / 'comprehensive_analysis.png', dpi=200, bbox_inches='tight')
        plt.close()
        
        # Save data for further analysis
        self._save_debug_data(output_dir)
    
    def _plot_3d_points(self, ax):
        """Plot 3D point cloud"""
        pts_robot = self.debug_info['pts_robot']
        floor_points = self.debug_info['floor_points']
        final_mask = self.debug_info['final_mask']
        
        # Sample for visualization
        max_pts = 10000
        if len(pts_robot) > max_pts:
            sample_idx = np.random.choice(len(pts_robot), max_pts, replace=False)
            pts_viz = pts_robot[sample_idx]
            mask_viz = final_mask[sample_idx]
        else:
            pts_viz = pts_robot
            mask_viz = final_mask
        
        # Plot non-floor points
        non_floor = pts_viz[~mask_viz]
        if len(non_floor) > 0:
            ax.scatter(non_floor[:, 0], non_floor[:, 1], non_floor[:, 2],
                      c='red', marker='.', s=1, alpha=0.3, label='Non-floor')
        
        # Plot floor points
        if len(floor_points) > 0:
            floor_sample = floor_points[::max(1, len(floor_points)//5000)]
            ax.scatter(floor_sample[:, 0], floor_sample[:, 1], floor_sample[:, 2],
                      c='green', marker='.', s=2, alpha=0.8, label='Floor')
        
        ax.set_xlabel('X (m)')
        ax.set_ylabel('Y (m)')
        ax.set_zlabel('Z (m)')
        ax.set_title('3D Floor Detection')
        ax.legend()
        
        # Set reasonable limits
        if len(floor_points) > 0:
            ax.set_xlim([floor_points[:, 0].min(), floor_points[:, 0].max()])
            ax.set_ylim([floor_points[:, 1].min(), floor_points[:, 1].max()])
            ax.set_zlim([floor_points[:, 2].min(), floor_points[:, 2].max() + 1])
    
    def _plot_filter_analysis(self, ax):
        """Plot filter effectiveness"""
        masks = {
            'Height': self.debug_info.get('height_mask', []),
            'SAM': self.debug_info.get('sam_mask', []),
            'Object': self.debug_info.get('object_mask', []),
            'Final': self.debug_info.get('final_mask', [])
        }
        
        names = list(masks.keys())
        counts = [np.sum(mask) if len(mask) > 0 else 0 for mask in masks.values()]
        total = self.debug_info.get('total_points', 1)
        percentages = [count / total * 100 for count in counts]
        
        bars = ax.bar(names, percentages)
        ax.set_ylabel('Percentage of Points (%)')
        ax.set_title('Filter Analysis')
        
        # Add value labels
        for bar, pct, count in zip(bars, percentages, counts):
            height = bar.get_height()
            ax.text(bar.get_x() + bar.get_width()/2., height + 0.5,
                   f'{pct:.1f}%\n({count:,})',
                   ha='center', va='bottom')
        
        ax.set_ylim(0, max(percentages) * 1.2 if percentages else 100)
    
    def _plot_height_distribution(self, ax):
        """Plot height distribution"""
        pts_robot = self.debug_info['pts_robot']
        floor_points = self.debug_info['floor_points']
        
        # Remove invalid values
        valid_z = ~np.isnan(pts_robot[:, 2]) & ~np.isinf(pts_robot[:, 2])
        all_heights = pts_robot[valid_z, 2]
        
        if len(all_heights) > 0 and len(floor_points) > 0:
            ax.hist(all_heights, bins=50, alpha=0.5, label='All points', density=True)
            ax.hist(floor_points[:, 2], bins=30, alpha=0.7, label='Floor points', density=True)
            
            # Add threshold line
            threshold = self.params['floor_height_threshold']
            ax.axvline(np.min(all_heights) + threshold, 
                      color='red', linestyle='--', label='Height threshold')
            
            ax.set_xlabel('Height (m)')
            ax.set_ylabel('Density')
            ax.set_title('Height Distribution')
            ax.legend()
    
    def _plot_xy_distribution(self, ax):
        """Plot XY distribution"""
        floor_points = self.debug_info['floor_points']
        
        if len(floor_points) > 0:
            scatter = ax.scatter(floor_points[:, 0], floor_points[:, 1], 
                               c=floor_points[:, 2], cmap='viridis', 
                               s=1, alpha=0.5)
            
            ax.set_xlabel('X (m)')
            ax.set_ylabel('Y (m)')
            ax.set_title('Floor Points XY Distribution')
            ax.axis('equal')
            
            # Add colorbar
            plt.colorbar(scatter, ax=ax, label='Height (m)')
    
    def _plot_filter_combination(self, ax):
        """Visualize filter combinations"""
        h_mask = self.debug_info.get('height_mask', [])
        s_mask = self.debug_info.get('sam_mask', [])
        o_mask = self.debug_info.get('object_mask', [])
        
        if len(h_mask) > 0 and len(s_mask) > 0 and len(o_mask) > 0:
            # Count combinations
            h_only = np.sum(h_mask & ~s_mask & o_mask)
            s_only = np.sum(~h_mask & s_mask & o_mask)
            hs_both = np.sum(h_mask & s_mask & o_mask)
            none = np.sum(~h_mask & ~s_mask)
            
            labels = ['Height\nonly', 'SAM\nonly', 'Both\n(used)', 'Neither']
            sizes = [h_only, s_only, hs_both, none]
            colors = ['lightblue', 'lightgreen', 'darkgreen', 'lightgray']
            
            # Only plot if we have data
            if sum(sizes) > 0:
                wedges, texts, autotexts = ax.pie(sizes, labels=labels, colors=colors, 
                                                  autopct='%1.1f%%', startangle=90)
                ax.set_title(f'Filter Combinations\n(Mode: {self.debug_info.get("filter_mode", "N/A")})')
            else:
                ax.text(0.5, 0.5, 'No filter data available', 
                       ha='center', va='center', transform=ax.transAxes)
        else:
            ax.text(0.5, 0.5, 'Filters not applied', 
                   ha='center', va='center', transform=ax.transAxes)
    
    def _plot_statistics(self, ax):
        """Display statistics"""
        ax.axis('off')
        
        stats = [
            f"Total Points: {self.debug_info.get('total_points', 0):,}",
            f"Floor Points: {len(self.debug_info.get('floor_points', [])):,}",
            f"Coverage: {len(self.debug_info.get('floor_points', []))/max(1, self.debug_info.get('total_points', 1))*100:.1f}%",
            "",
            "Filter Performance:",
            f"  Height: {np.sum(self.debug_info.get('height_mask', [])):,}",
            f"  SAM: {np.sum(self.debug_info.get('sam_mask', [])):,}",
            f"  Object: {np.sum(self.debug_info.get('object_mask', [])):,}",
            "",
            f"Filter Mode: {self.debug_info.get('filter_mode', 'N/A')}",
            f"Height Threshold: {self.params['floor_height_threshold']}m",
            f"RANSAC Threshold: {self.params['ransac_threshold']}m"
        ]
        
        stats_text = '\n'.join(stats)
        ax.text(0.1, 0.9, stats_text, transform=ax.transAxes,
               fontsize=12, verticalalignment='top', fontfamily='monospace',
               bbox=dict(boxstyle="round,pad=0.5", facecolor="lightgray"))
    
    def _save_debug_data(self, output_dir: Path):
        """Save debug data for further analysis"""
        data_to_save = {}
        
        # Only save arrays that exist
        for key in ['floor_points', 'height_mask', 'sam_mask', 'object_mask', 'final_mask']:
            if key in self.debug_info and len(self.debug_info[key]) > 0:
                data_to_save[key] = self.debug_info[key]
        
        if data_to_save:
            np.savez(output_dir / 'debug_data.npz', **data_to_save)
    
    def save_all_maps(self):
        """Save all generated maps"""
        if not self.output_manager or not hasattr(self, 'map_data'):
            return
        
        output_dir = self.output_manager['maps']
        
        # Enhance maps
        enhanced_maps = self._enhance_maps()
        
        # Create semantic map
        semantic_map = self._create_semantic_map(enhanced_maps)
        if semantic_map is not None:
            enhanced_maps['semantic'] = semantic_map
        
        # Save individual maps
        self._save_individual_maps(enhanced_maps, output_dir)
        
        # Create combined view
        self._create_combined_map_view(enhanced_maps, output_dir)
    
    def _enhance_maps(self) -> Dict[str, np.ndarray]:
        """Apply enhancements to maps"""
        enhanced_maps = {}
        
        for name, data in self.map_data.items():
            if name in ['elevation', 'height']:
                # Apply Gaussian smoothing
                enhanced = gaussian_filter(data, sigma=1.0)
                
                # Interpolate missing values
                mask = ~np.isnan(enhanced)
                if np.sum(mask) > 10:
                    enhanced = self._interpolate_map(enhanced)
                
                enhanced_maps[name] = enhanced
            else:
                enhanced_maps[name] = data
        
        return enhanced_maps
    
    def _create_semantic_map(self, enhanced_maps: Dict[str, np.ndarray]) -> Optional[np.ndarray]:
        """Create semantic segmentation map"""
        if 'elevation' not in enhanced_maps or 'obstacles' not in enhanced_maps:
            return None
        
        elevation = enhanced_maps['elevation']
        obstacles = enhanced_maps['obstacles']
        
        semantic_map = np.zeros_like(elevation, dtype=np.uint8)
        
        # Floor pixels
        floor_mask = (obstacles < 0.3) & ~np.isnan(elevation)
        semantic_map[floor_mask] = self.semantic_labels['floor']
        
        # Obstacle pixels
        obstacle_mask = obstacles > 0.7
        semantic_map[obstacle_mask] = self.semantic_labels['obstacle']
        
        return semantic_map
    
    def _save_individual_maps(self, maps: Dict[str, np.ndarray], output_dir: Path):
        """Save individual map visualizations"""
        extent = [-self.map_x_length/2, self.map_x_length/2,
                  -self.map_y_length/2, self.map_y_length/2]
        
        for name, data in maps.items():
            fig, ax = plt.subplots(figsize=(10, 10))
            
            # Choose appropriate colormap
            if name == 'elevation':
                im = ax.imshow(data, extent=extent, origin='lower', cmap='terrain')
                cbar_label = 'Height (m)'
            elif name == 'obstacles':
                im = ax.imshow(data, extent=extent, origin='lower', cmap='RdYlGn_r')
                cbar_label = 'Obstacle Probability'
            elif name == 'semantic':
                # Custom colormap for semantic
                from matplotlib.colors import ListedColormap
                colors = ['black', 'green', 'red', 'blue', 'yellow']
                cmap = ListedColormap(colors[:len(self.semantic_labels)])
                im = ax.imshow(data, extent=extent, origin='lower', cmap=cmap, 
                             vmin=0, vmax=len(self.semantic_labels)-1)
                cbar_label = 'Semantic Label'
                
                # Add legend
                handles = []
                for label, idx in self.semantic_labels.items():
                    handles.append(plt.Rectangle((0, 0), 1, 1, fc=colors[idx], label=label))
                ax.legend(handles=handles, loc='upper right')
            else:
                im = ax.imshow(data, extent=extent, origin='lower', cmap='viridis')
                cbar_label = 'Value'
            
            ax.set_title(f'{name.capitalize()} Map', fontsize=14)
            ax.set_xlabel('X (m)')
            ax.set_ylabel('Y (m)')
            plt.colorbar(im, ax=ax, label=cbar_label)
            
            # Add grid
            ax.grid(True, alpha=0.3)
            
            # Add statistics
            if not np.all(np.isnan(data)):
                stats_text = f"Min: {np.nanmin(data):.2f}, Max: {np.nanmax(data):.2f}, Mean: {np.nanmean(data):.2f}"
                ax.text(0.02, 0.02, stats_text, transform=ax.transAxes,
                       bbox=dict(boxstyle="round,pad=0.3", facecolor="white", alpha=0.8))
            
            plt.savefig(output_dir / f'{name}_map_enhanced.png', dpi=200, bbox_inches='tight')
            plt.close()
            
            # Save raw data
            np.save(output_dir / f'{name}_map.npy', data)
    
    def _create_combined_map_view(self, maps: Dict[str, np.ndarray], output_dir: Path):
        """Create combined view of all maps"""
        n_maps = len(maps)
        cols = min(3, n_maps)
        rows = (n_maps + cols - 1) // cols
        
        fig, axes = plt.subplots(rows, cols, figsize=(6*cols, 5*rows))
        if rows == 1 and cols == 1:
            axes = np.array([[axes]])
        elif rows == 1 or cols == 1:
            axes = axes.reshape(rows, cols)
        
        extent = [-self.map_x_length/2, self.map_x_length/2,
                  -self.map_y_length/2, self.map_y_length/2]
        
        for idx, (name, data) in enumerate(maps.items()):
            row = idx // cols
            col = idx % cols
            ax = axes[row, col]
            
            if name == 'elevation':
                im = ax.imshow(data, extent=extent, origin='lower', cmap='terrain')
            elif name == 'obstacles':
                im = ax.imshow(data, extent=extent, origin='lower', cmap='RdYlGn_r')
            elif name == 'semantic':
                im = ax.imshow(data, extent=extent, origin='lower', cmap='tab10')
            else:
                im = ax.imshow(data, extent=extent, origin='lower', cmap='viridis')
            
            ax.set_title(name.capitalize())
            ax.set_xlabel('X (m)')
            ax.set_ylabel('Y (m)')
            plt.colorbar(im, ax=ax)
        
        # Hide empty subplots
        for idx in range(n_maps, rows * cols):
            row = idx // cols
            col = idx % cols
            axes[row, col].axis('off')
        
        plt.tight_layout()
        plt.savefig(output_dir / 'all_maps_overview.png', dpi=200, bbox_inches='tight')
        plt.close()


def create_summary_report(mapper: EnhancedFloorMapper, output_manager: OutputManager, 
                         rgb_image: np.ndarray, floor_mask: np.ndarray, cloud_name: str):
    """Create comprehensive summary report"""
    fig = plt.figure(figsize=(24, 18))
    
    # Create grid
    gs = plt.GridSpec(4, 4, figure=fig, hspace=0.3, wspace=0.3)
    
    # 1. Input RGB
    ax1 = fig.add_subplot(gs[0, 0])
    if rgb_image is not None:
        ax1.imshow(cv.cvtColor(rgb_image, cv.COLOR_BGR2RGB))
    ax1.set_title('Input RGB', fontsize=12)
    ax1.axis('off')
    
    # 2. SAM Floor Mask
    ax2 = fig.add_subplot(gs[0, 1])
    if floor_mask is not None:
        ax2.imshow(floor_mask, cmap='gray')
        coverage = np.sum(floor_mask) / floor_mask.size * 100
        ax2.set_title(f'SAM Floor ({coverage:.1f}%)', fontsize=12)
    ax2.axis('off')
    
    # 3. Object Detection
    ax3 = fig.add_subplot(gs[0, 2])
    obj_path = output_manager['segmentation'] / 'detected_objects.png'
    if obj_path.exists():
        obj_img = plt.imread(obj_path)
        ax3.imshow(obj_img)
    ax3.set_title('Object Detection', fontsize=12)
    ax3.axis('off')
    
    # 4. 3D Floor Points
    ax4 = fig.add_subplot(gs[0, 3], projection='3d')
    if hasattr(mapper, 'debug_info') and 'floor_points' in mapper.debug_info:
        floor_pts = mapper.debug_info['floor_points']
        if len(floor_pts) > 0:
            sample = floor_pts[::max(1, len(floor_pts)//5000)]
            ax4.scatter(sample[:, 0], sample[:, 1], sample[:, 2],
                       c=sample[:, 2], cmap='viridis', s=1)
    ax4.set_title('3D Floor Points', fontsize=12)
    ax4.set_xlabel('X')
    ax4.set_ylabel('Y')
    ax4.set_zlabel('Z')
    
    # 5-8. Maps
    if hasattr(mapper, 'map_data'):
        extent = [-mapper.map_x_length/2, mapper.map_x_length/2,
                  -mapper.map_y_length/2, mapper.map_y_length/2]
        
        map_positions = {
            'elevation': (1, 0),
            'obstacles': (1, 1),
            'histogram': (1, 2),
            'semantic': (1, 3)
        }
        
        for name, (row, col) in map_positions.items():
            if name in mapper.map_data:
                ax = fig.add_subplot(gs[row, col])
                data = mapper.map_data[name]
                
                if name == 'elevation':
                    im = ax.imshow(data, extent=extent, origin='lower', cmap='terrain')
                elif name == 'obstacles':
                    im = ax.imshow(data, extent=extent, origin='lower', cmap='RdYlGn_r')
                elif name == 'semantic' and 'semantic' in mapper.map_data:
                    im = ax.imshow(data, extent=extent, origin='lower', cmap='tab10')
                else:
                    im = ax.imshow(data, extent=extent, origin='lower', cmap='viridis')
                
                ax.set_title(f'{name.capitalize()} Map', fontsize=12)
                ax.set_xlabel('X (m)')
                ax.set_ylabel('Y (m)')
                plt.colorbar(im, ax=ax)
    
    # 9. Filter Analysis
    ax9 = fig.add_subplot(gs[2, 0:2])
    if hasattr(mapper, 'debug_info'):
        masks = {
            'Height': mapper.debug_info.get('height_mask', []),
            'SAM': mapper.debug_info.get('sam_mask', []),
            'Object': mapper.debug_info.get('object_mask', []),
            'Final': mapper.debug_info.get('final_mask', [])
        }
        
        if all(len(m) > 0 for m in masks.values()):
            names = list(masks.keys())
            counts = [np.sum(mask) for mask in masks.values()]
            
            bars = ax9.bar(names, counts)
            ax9.set_ylabel('Number of Points')
            ax9.set_title('Filter Analysis', fontsize=12)
            
            for bar, count in zip(bars, counts):
                height = bar.get_height()
                ax9.text(bar.get_x() + bar.get_width()/2., height + 100,
                        f'{count:,}', ha='center', va='bottom')
    
    # 10. Multi-resolution preview
    ax10 = fig.add_subplot(gs[2, 2:4])
    multi_res_dir = output_manager['multi_res']
    if multi_res_dir.exists():
        res_files = list(multi_res_dir.glob('height_map_*.png'))
        if res_files:
            # Show finest resolution
            finest = sorted(res_files)[0]
            res_img = plt.imread(finest)
            ax10.imshow(res_img)
            ax10.set_title('Multi-Resolution Maps', fontsize=12)
    ax10.axis('off')
    
    # 11. Statistics
    ax11 = fig.add_subplot(gs[3, :])
    ax11.axis('off')
    
    if hasattr(mapper, 'debug_info'):
        total_pts = mapper.debug_info.get('total_points', 0)
        floor_pts = len(mapper.debug_info.get('floor_points', []))
        coverage = floor_pts / max(1, total_pts) * 100
        
        stats_text = f"""
Floor Mapping Summary - {cloud_name}

Processing Statistics:
• Total Input Points: {total_pts:,}
• Floor Points Detected: {floor_pts:,}
• Floor Coverage: {coverage:.1f}%

Filter Performance:
• Height Filter: {np.sum(mapper.debug_info.get('height_mask', [])):,} points passed
• SAM Filter: {np.sum(mapper.debug_info.get('sam_mask', [])):,} points on floor
• Object Filter: {np.sum(mapper.debug_info.get('object_mask', [])):,} points outside objects

Configuration:
• Filter Mode: {mapper.debug_info.get('filter_mode', 'N/A')}
• Height Threshold: {mapper.params.get('floor_height_threshold', 0)}m
• RANSAC Threshold: {mapper.params.get('ransac_threshold', 0)}m
• Multi-Resolution: {mapper.params.get('multi_resolution', False)}
• Resolutions: {mapper.resolutions}

Output Files:
• Segmentation results in: {output_manager['segmentation'].name}/
• Ground detection in: {output_manager['ground_detection'].name}/
• Maps in: {output_manager['maps'].name}/
• Multi-resolution in: {output_manager['multi_res'].name}/
        """
    else:
        stats_text = "No statistics available"
    
    ax11.text(0.5, 0.5, stats_text, transform=ax11.transAxes,
             fontsize=11, ha='center', va='center', fontfamily='monospace',
             bbox=dict(boxstyle="round,pad=0.5", facecolor="lightgray", alpha=0.8))
    
    plt.suptitle(f'Enhanced Floor Segmentation and Mapping - {cloud_name}', fontsize=16, y=0.98)
    plt.tight_layout()
    plt.savefig(output_manager['summary'] / 'complete_summary.png', dpi=250, bbox_inches='tight')
    plt.close()
    
    # Save configuration
    config = {
        'timestamp': datetime.now().isoformat(),
        'cloud_name': cloud_name,
        'parameters': mapper.params,
        'statistics': {
            'total_points': total_pts,
            'floor_points': floor_pts,
            'coverage_percent': coverage
        }
    }
    
    with open(output_manager['summary'] / 'config.json', 'w') as f:
        json.dump(config, f, indent=2)


def process_pointcloud(cloud_path: Path, args):
    """Main processing function"""
    logger.info(f"Processing point cloud: {cloud_path}")
    
    # Load point cloud
    try:
        pcd = o3d.io.read_point_cloud(str(cloud_path))
        if not pcd.has_points():
            logger.error("Point cloud has no points")
            return
        
        pts = np.asarray(pcd.points)
        logger.info(f"Loaded {len(pts)} points")
    except Exception as e:
        logger.error(f"Failed to load point cloud: {e}")
        return
    
    # Create output manager
    cloud_name = cloud_path.stem
    output_manager = OutputManager(Path.cwd(), cloud_name)
    
    # Initialize mapper
    mapper = EnhancedFloorMapper()
    mapper.output_manager = output_manager
    
    # Update parameters from args
    mapper.params['use_sam'] = not args.no_sam
    mapper.params['use_grounding_dino'] = not args.no_dino
    mapper.params['use_height_filter'] = not args.no_height
    mapper.params['camera_fov_degrees'] = args.fov
    mapper.params['filter_combination'] = args.filter_mode
    mapper.params['multi_resolution'] = not args.no_multi_res
    
    # Initialize processors
    mapper.initialize_processors(
        Path(args.sam_checkpoint) if args.sam_checkpoint else None
    )
    
    # Load RGB image if available
    rgb_image = None
    rgb_path = cloud_path.parent / f"{cloud_name}.png"
    if not rgb_path.exists():
        rgb_path = cloud_path.parent / f"{cloud_name}.jpg"
    
    if rgb_path.exists():
        logger.info(f"Loading RGB image: {rgb_path}")
        rgb_image = cv.imread(str(rgb_path))
        if rgb_image is not None:
            mapper.process_image(rgb_image)
    else:
        logger.warning("No RGB image found, some features will be disabled")
        mapper.params['use_sam'] = False
        mapper.params['use_grounding_dino'] = False
    
    # Process point cloud
    try:
        mapper.apply_pointcloud(pts)
        logger.info("Point cloud processing completed")
    except Exception as e:
        logger.error(f"Point cloud processing failed: {e}")
        import traceback
        traceback.print_exc()
        return
    
    # Save maps
    try:
        mapper.save_all_maps()
        logger.info("Maps saved successfully")
    except Exception as e:
        logger.error(f"Failed to save maps: {e}")
    
    # Create summary
    try:
        create_summary_report(
            mapper, output_manager, rgb_image, 
            mapper.floor_mask, cloud_name
        )
        logger.info("Summary report created")
    except Exception as e:
        logger.error(f"Failed to create summary: {e}")
    
    logger.info(f"Processing complete. Results saved to: {output_manager['root']}")


def main():
    import argparse
    
    parser = argparse.ArgumentParser(description="Enhanced Floor Segmentation and Mapping")
    parser.add_argument("--cloud", required=True, help="Path to .ply point cloud")
    parser.add_argument("--no-sam", action="store_true", help="Disable SAM")
    parser.add_argument("--no-dino", action="store_true", help="Disable GroundingDINO")
    parser.add_argument("--no-height", action="store_true", help="Disable height filtering")
    parser.add_argument("--fov", type=float, default=70, help="Camera FOV in degrees")
    parser.add_argument("--sam-checkpoint", help="Path to SAM checkpoint")
    parser.add_argument("--filter-mode", default="sam_primary", 
                       choices=["sam_primary", "adaptive", "all"],
                       help="Filter combination mode")
    parser.add_argument("--no-multi-res", action="store_true", help="Disable multi-resolution")
    args = parser.parse_args()
    
    # Process point cloud
    cloud_path = Path(args.cloud).expanduser()
    if not cloud_path.exists():
        logger.error(f"Point cloud not found: {cloud_path}")
        return
    
    process_pointcloud(cloud_path, args)


if __name__ == "__main__":
    main()
