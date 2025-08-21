import numpy as np
import time
import sys
import os
import cv2
import torch

# Add external model paths
sys.path.append(os.path.join(os.path.dirname(__file__), '../external_models/MiDaS'))
sys.path.append(os.path.join(os.path.dirname(__file__), '../external_models/ESANet'))
sys.path.append(os.path.join(os.path.dirname(__file__), '../external_models/SegFormer'))
sys.path.append(os.path.join(os.path.dirname(__file__), '../external_models/semantic-segmentation-pytorch'))
sys.path.append(os.path.join(os.path.dirname(__file__), '../external_models/detectron2'))
sys.path.append(os.path.join(os.path.dirname(__file__), '../external_models/segment-anything'))
sys.path.append(os.path.join(os.path.dirname(__file__), '../external_models/segment-anything-2'))


class ModelGroundDetector:
    """Simple wrapper to call different segmentation models and extract ground plane."""
    
    def __init__(self, model_name='midas'):
        self.model_name = model_name
        self.model = None
        self.device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
        
    def load_model(self):
        """Load the specified model."""
        if self.model_name == 'sam' or self.model_name == 'sam2':
            # Load SAM or SAM2
            try:
                if self.model_name == 'sam2':
                    from segment_anything_2 import sam_model_registry, SamAutomaticMaskGenerator
                    # SAM2 uses different checkpoint names
                    sam_checkpoint = "sam2_vit_h.pth"  # You need to download this
                    model_type = "vit_h"
                else:
                    from segment_anything import sam_model_registry, SamAutomaticMaskGenerator
                    sam_checkpoint = "sam_vit_h_4b8939.pth"  # You need to download this
                    model_type = "vit_h"
                
                # Check if checkpoint exists, otherwise use default
                if not os.path.exists(sam_checkpoint):
                    print(f"SAM checkpoint not found. Please download from the official repo.")
                    print("Using MiDaS as fallback...")
                    self.model_name = 'midas'
                    self.load_model()
                    return
                    
                sam = sam_model_registry[model_type](checkpoint=sam_checkpoint)
                sam.to(device=self.device)
                self.model = SamAutomaticMaskGenerator(sam)
                print(f"Loaded {self.model_name.upper()} model")
            except Exception as e:
                print(f"Error loading {self.model_name}: {e}")
                print("Using MiDaS as fallback...")
                self.model_name = 'midas'
                self.load_model()
                
        elif self.model_name == 'midas':
            # Load MiDaS
            self.model = torch.hub.load("intel-isl/MiDaS", "MiDaS")
            self.model.to(self.device)
            self.model.eval()
            
        elif self.model_name == 'segformer':
            # Load SegFormer using HuggingFace
            from transformers import SegformerForSemanticSegmentation, SegformerImageProcessor
            self.processor = SegformerImageProcessor.from_pretrained("nvidia/segformer-b0-finetuned-ade-512-512")
            self.model = SegformerForSemanticSegmentation.from_pretrained("nvidia/segformer-b0-finetuned-ade-512-512")
            self.model.to(self.device)
            self.model.eval()
            
        elif self.model_name == 'detectron2':
            # Load Detectron2 panoptic segmentation
            from detectron2 import model_zoo
            from detectron2.engine import DefaultPredictor
            from detectron2.config import get_cfg
            
            cfg = get_cfg()
            cfg.merge_from_file(model_zoo.get_config_file("COCO-PanopticSegmentation/panoptic_fpn_R_50_3x.yaml"))
            cfg.MODEL.WEIGHTS = model_zoo.get_checkpoint_url("COCO-PanopticSegmentation/panoptic_fpn_R_50_3x.yaml")
            cfg.MODEL.DEVICE = 'cuda' if torch.cuda.is_available() else 'cpu'
            self.model = DefaultPredictor(cfg)
            
        elif self.model_name == 'esanet':
            # ESANet would require more setup
            print(f"ESANet requires additional setup. Using MiDaS as fallback.")
            self.model_name = 'midas'
            self.load_model()
            
    def detect_ground_from_image(self, rgb_image, depth_image=None):
        """Detect ground/floor pixels from RGB (and optionally depth) image.
        
        Returns:
            ground_mask: Boolean mask of ground pixels matching image shape
        """
        if self.model is None:
            self.load_model()
            
        h, w = rgb_image.shape[:2]
        
        if self.model_name in ['sam', 'sam2']:
            # SAM generates masks for everything, we need to find floor
            masks = self.model.generate(rgb_image)
            
            # Heuristic: floor is typically the largest mask in the bottom half
            h, w = rgb_image.shape[:2]
            best_floor_mask = None
            best_score = 0
            
            for mask_data in masks:
                mask = mask_data['segmentation']
                
                # Calculate how much of the mask is in bottom half
                bottom_pixels = np.sum(mask[h//2:, :])
                total_pixels = np.sum(mask)
                
                if total_pixels > 0:
                    bottom_ratio = bottom_pixels / total_pixels
                    # Score based on size and bottom position
                    score = total_pixels * bottom_ratio
                    
                    if score > best_score:
                        best_score = score
                        best_floor_mask = mask
                        
            ground_mask = best_floor_mask if best_floor_mask is not None else np.zeros((h, w), dtype=bool)
            
        elif self.model_name == 'midas':
            # MiDaS predicts depth, we use it to find floor
            midas_transforms = torch.hub.load("intel-isl/MiDaS", "transforms")
            transform = midas_transforms.default_transform
            
            input_batch = transform(rgb_image).to(self.device)
            
            with torch.no_grad():
                prediction = self.model(input_batch)
                prediction = torch.nn.functional.interpolate(
                    prediction.unsqueeze(1),
                    size=(h, w),
                    mode="bicubic",
                    align_corners=False,
                ).squeeze()
            
            depth_map = prediction.cpu().numpy()
            
            # Simple heuristic: floor is typically at bottom with consistent depth
            grad_y = np.gradient(depth_map, axis=0)
            y_coords = np.arange(h).reshape(-1, 1)
            y_weight = y_coords / h  # Higher weight for lower parts
            floor_score = (1 - np.abs(grad_y)) * y_weight
            ground_mask = floor_score > 0.5
            
        elif self.model_name == 'segformer':
            # SegFormer semantic segmentation
            inputs = self.processor(images=rgb_image, return_tensors="pt")
            inputs = {k: v.to(self.device) for k, v in inputs.items()}
            
            with torch.no_grad():
                outputs = self.model(**inputs)
                
            logits = outputs.logits
            upsampled = torch.nn.functional.interpolate(
                logits,
                size=(h, w),
                mode="bilinear",
                align_corners=False
            )
            
            pred_seg = upsampled.argmax(dim=1)[0].cpu().numpy()
            # ADE20K classes: 4=floor, 14=ground, 30=road
            ground_mask = np.isin(pred_seg, [4, 14, 30])
            
        elif self.model_name == 'detectron2':
            # Detectron2 panoptic segmentation
            outputs = self.model(rgb_image)
            
            # Get semantic segmentation
            if "sem_seg" in outputs:
                sem_seg = outputs["sem_seg"].argmax(dim=0).cpu().numpy()
                # COCO classes: floor/ground related classes
                ground_mask = np.isin(sem_seg, [14, 15, 16])  # floor, pavement, ground
            else:
                # Fallback to simple bottom region
                ground_mask = np.zeros((h, w), dtype=bool)
                ground_mask[int(h*0.7):, :] = True
                
        else:
            # Default: simple bottom region detection
            ground_mask = np.zeros((h, w), dtype=bool)
            ground_mask[int(h*0.7):, :] = True
            
        return ground_mask.astype(bool)
    
    def convert_mask_to_3d_mask(self, image_mask, point_cloud_size):
        """Convert 2D image mask to 3D point cloud mask.
        
        Args:
            image_mask: 2D boolean mask from image
            point_cloud_size: Number of points in point cloud
            
        Returns:
            3D boolean mask for point cloud
        """
        # Flatten and sample/interpolate to match point cloud size
        mask_flat = image_mask.flatten()
        
        if len(mask_flat) == point_cloud_size:
            return mask_flat
        else:
            # Sample to match point cloud size
            indices = np.linspace(0, len(mask_flat)-1, point_cloud_size).astype(int)
            return mask_flat[indices]
    
    def detect_ground_with_timing(self, rgb_image, points_3d, depth_image=None):
        """Detect ground plane with timing.
        
        Args:
            rgb_image: RGB image
            points_3d: 3D point cloud
            depth_image: Optional depth image
            
        Returns:
            ground_plane: Fitted plane parameters [a,b,c,d]
            ground_mask: Boolean mask for ground points
            time_taken: Time in seconds
        """
        start_time = time.time()
        
        # Get 2D ground mask from image
        image_ground_mask = self.detect_ground_from_image(rgb_image, depth_image)
        
        # Convert to 3D mask
        ground_mask = self.convert_mask_to_3d_mask(image_ground_mask, len(points_3d))
        
        # Fit plane to ground points
        ground_points = points_3d[ground_mask]
        
        if len(ground_points) < 3:
            return None, ground_mask, time.time() - start_time
        
        # Simple plane fitting using SVD
        ground_plane = self.fit_plane_svd(ground_points)
        
        time_taken = time.time() - start_time
        
        return ground_plane, ground_mask, time_taken
    
    def fit_plane_svd(self, points):
        """Fit plane using SVD.
        
        Args:
            points: Nx3 array of 3D points
            
        Returns:
            plane: [a,b,c,d] where ax+by+cz+d=0
        """
        centroid = np.mean(points, axis=0)
        points_centered = points - centroid
        
        # SVD
        _, _, vh = np.linalg.svd(points_centered)
        normal = vh[2, :]
        
        # Ensure upward pointing normal
        if normal[2] < 0:
            normal = -normal
            
        d = -np.dot(normal, centroid)
        
        return np.append(normal, d)