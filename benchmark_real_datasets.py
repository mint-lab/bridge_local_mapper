#!/usr/bin/env python3
"""
Benchmark using real public datasets:
- KITTI (autonomous driving)
- NYU Depth V2 (indoor RGB-D)
- SUN RGB-D (indoor scenes)
- Or custom RGB-D data
"""

import os
os.environ['KMP_DUPLICATE_LIB_OK'] = 'TRUE'
import numpy as np
import cv2
import time
import sys
import json
import urllib.request
import zipfile
from pathlib import Path
from datetime import datetime
sys.path.append(os.path.dirname(os.path.abspath(__file__)))

from bridge_local_planner.gtrack_mapper_with_models import GTrackMapper


class RealDatasetBenchmark:
    def __init__(self, dataset_dir="datasets", output_dir="benchmark_real"):
        self.dataset_dir = Path(dataset_dir)
        self.dataset_dir.mkdir(exist_ok=True)
        self.output_dir = Path(output_dir)
        self.timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.session_dir = self.output_dir / f"session_{self.timestamp}"
        self.session_dir.mkdir(parents=True, exist_ok=True)
        
    def download_sample_data(self):
        """Download sample data from public datasets."""
        print("Downloading sample datasets...")
        
        # Create sample data info
        datasets = {
            'kitti': {
                'url': 'https://s3.eu-central-1.amazonaws.com/avg-kitti/data_odometry_color.zip',
                'description': 'KITTI odometry dataset (sample)',
                'type': 'lidar_camera'
            },
            'nyu_depth': {
                'url': 'http://horatio.cs.nyu.edu/mit/silberman/nyu_depth_v2/nyu_depth_v2_labeled.mat',
                'description': 'NYU Depth V2 indoor scenes',
                'type': 'rgbd'
            }
        }
        
        print("\nNote: For full datasets, please download from:")
        print("- KITTI: http://www.cvlibs.net/datasets/kitti/")
        print("- NYU Depth V2: https://cs.nyu.edu/~silberman/datasets/nyu_depth_v2.html")
        print("- SUN RGB-D: http://rgbd.cs.princeton.edu/")
        
        return datasets
    
    def load_kitti_sample(self, sequence="00", frame=0):
        """Load KITTI dataset sample.
        
        KITTI provides:
        - Velodyne point clouds
        - Camera images (left/right, color/grayscale)
        - GPS/IMU data
        - Ground truth poses
        """
        kitti_dir = self.dataset_dir / "kitti"
        
        # Check if KITTI data exists
        if not kitti_dir.exists():
            print(f"KITTI data not found at {kitti_dir}")
            print("Please download from: http://www.cvlibs.net/datasets/kitti/eval_odometry.php")
            return None, None, None
        
        # Load velodyne point cloud
        velo_file = kitti_dir / f"sequences/{sequence}/velodyne/{frame:06d}.bin"
        if velo_file.exists():
            points = np.fromfile(str(velo_file), dtype=np.float32).reshape(-1, 4)
            points = points[:, :3]  # Remove intensity
        else:
            print(f"Velodyne file not found: {velo_file}")
            return None, None, None
        
        # Load camera image
        img_file = kitti_dir / f"sequences/{sequence}/image_2/{frame:06d}.png"
        if img_file.exists():
            rgb_image = cv2.imread(str(img_file))
            rgb_image = cv2.cvtColor(rgb_image, cv2.COLOR_BGR2RGB)
        else:
            print(f"Image file not found: {img_file}")
            rgb_image = None
        
        return points, rgb_image, None
    
    def load_nyu_depth_sample(self, idx=0):
        """Load NYU Depth V2 sample.
        
        NYU Depth V2 provides:
        - RGB images (640x480)
        - Depth maps
        - Semantic labels
        """
        # For demonstration, create sample data structure
        # In reality, you would load from .mat files or preprocessed data
        
        print("NYU Depth V2 dataset loading...")
        print("Note: Requires scipy to load .mat files")
        
        # Placeholder for NYU data
        height, width = 480, 640
        
        # Generate sample RGB-D data (replace with actual loading)
        rgb_image = np.zeros((height, width, 3), dtype=np.uint8)
        depth_image = np.ones((height, width), dtype=np.float32) * 3.0
        
        # Convert depth to point cloud
        points = self.depth_to_pointcloud(depth_image, fx=518.8579, fy=518.8579, cx=325.5824, cy=253.7362)
        
        return points, rgb_image, depth_image
    
    def load_rgbd_from_files(self, rgb_path, depth_path, camera_params=None):
        """Load RGB-D data from image files and convert to point cloud.
        
        Args:
            rgb_path: Path to RGB image
            depth_path: Path to depth image
            camera_params: Dict with 'fx', 'fy', 'cx', 'cy' (camera intrinsics)
        """
        # Load RGB image
        rgb_image = cv2.imread(str(rgb_path))
        if rgb_image is None:
            print(f"Failed to load RGB image: {rgb_path}")
            return None, None, None
        rgb_image = cv2.cvtColor(rgb_image, cv2.COLOR_BGR2RGB)
        
        # Load depth image
        depth_image = cv2.imread(str(depth_path), cv2.IMREAD_ANYDEPTH)
        if depth_image is None:
            print(f"Failed to load depth image: {depth_path}")
            return None, None, None
        
        # Convert depth to meters if needed (many datasets store in mm)
        if depth_image.dtype == np.uint16:
            depth_image = depth_image.astype(np.float32) / 1000.0  # mm to meters
        
        # Default camera parameters (can be overridden)
        if camera_params is None:
            # These are typical for Kinect/RealSense
            camera_params = {
                'fx': 525.0,  # Focal length x
                'fy': 525.0,  # Focal length y
                'cx': rgb_image.shape[1] / 2,  # Principal point x
                'cy': rgb_image.shape[0] / 2,  # Principal point y
            }
        
        # Convert depth to point cloud
        points = self.depth_to_pointcloud(
            depth_image,
            camera_params['fx'],
            camera_params['fy'],
            camera_params['cx'],
            camera_params['cy']
        )
        
        return points, rgb_image, depth_image
    
    def depth_to_pointcloud(self, depth_image, fx, fy, cx, cy, max_depth=10.0):
        """Convert depth image to 3D point cloud using camera intrinsics.
        
        Args:
            depth_image: HxW depth map in meters
            fx, fy: Focal lengths
            cx, cy: Principal point
            max_depth: Maximum depth threshold
        """
        height, width = depth_image.shape
        
        # Create mesh grid
        xx, yy = np.meshgrid(np.arange(width), np.arange(height))
        
        # Valid depth mask
        valid_mask = (depth_image > 0) & (depth_image < max_depth)
        
        # Back-project to 3D
        z = depth_image[valid_mask]
        x = (xx[valid_mask] - cx) * z / fx
        y = (yy[valid_mask] - cy) * z / fy
        
        # Stack into point cloud
        points = np.stack([x, y, z], axis=1)
        
        return points
    
    def run_benchmark_on_dataset(self, dataset_type="custom"):
        """Run benchmark on specified dataset."""
        
        print(f"\n{'='*60}")
        print(f"Running benchmark on {dataset_type} dataset")
        print('='*60)
        
        # Initialize mapper
        mapper = GTrackMapper()
        mapper.params['debug_info'] = True
        
        results = {}
        
        if dataset_type == "kitti":
            # Test on KITTI samples
            sequences = ["00"]  # Add more sequences as needed
            frames = [0, 100, 200]  # Sample frames
            
            for seq in sequences:
                for frame in frames:
                    points, rgb_image, _ = self.load_kitti_sample(seq, frame)
                    if points is not None:
                        results[f"kitti_{seq}_{frame}"] = self.benchmark_frame(
                            mapper, points, rgb_image, None,
                            f"KITTI Seq {seq} Frame {frame}"
                        )
                        
        elif dataset_type == "nyu":
            # Test on NYU Depth samples
            for idx in range(3):  # Test 3 samples
                points, rgb_image, depth_image = self.load_nyu_depth_sample(idx)
                if points is not None:
                    results[f"nyu_{idx}"] = self.benchmark_frame(
                        mapper, points, rgb_image, depth_image,
                        f"NYU Depth Sample {idx}"
                    )
                    
        elif dataset_type == "custom":
            # Load custom RGB-D data
            custom_dir = self.dataset_dir / "custom"
            if custom_dir.exists():
                rgb_files = sorted(custom_dir.glob("*_rgb.png"))
                depth_files = sorted(custom_dir.glob("*_depth.png"))
                
                for rgb_file, depth_file in zip(rgb_files[:5], depth_files[:5]):
                    points, rgb_image, depth_image = self.load_rgbd_from_files(
                        rgb_file, depth_file
                    )
                    if points is not None:
                        results[rgb_file.stem] = self.benchmark_frame(
                            mapper, points, rgb_image, depth_image,
                            f"Custom: {rgb_file.stem}"
                        )
            else:
                print(f"Custom data directory not found: {custom_dir}")
                print("Please place RGB-D images in this directory:")
                print("  - *_rgb.png for RGB images")
                print("  - *_depth.png for depth images")
        
        # Save results
        self.save_results(results, dataset_type)
        return results
    
    def benchmark_frame(self, mapper, points, rgb_image, depth_image, description):
        """Benchmark a single frame."""
        
        print(f"\nProcessing: {description}")
        print(f"  Points: {len(points)}")
        
        models = ['original_ransac', 'midas', 'segformer', 'detectron2']
        frame_results = {}
        
        if rgb_image is not None:
            mapper.set_rgb_image(rgb_image)
        if depth_image is not None:
            mapper.set_depth_image(depth_image)
        
        for model_name in models:
            print(f"  Testing {model_name}...", end=' ')
            
            # Configure model
            if model_name == 'original_ransac':
                mapper.set_detection_method(use_model=False)
            else:
                mapper.set_detection_method(use_model=True, model_name=model_name)
            
            # Run detection
            start_time = time.time()
            success = mapper.apply_pointcloud(points.copy())
            inference_time = (time.time() - start_time) * 1000  # ms
            
            if success and 'ground_mask' in mapper.debug_info:
                ground_mask = mapper.debug_info['ground_mask']
                n_ground = np.sum(ground_mask)
                
                frame_results[model_name] = {
                    'time_ms': inference_time,
                    'ground_points': int(n_ground),
                    'total_points': len(points),
                    'ground_ratio': float(n_ground / len(points))
                }
                print(f"{inference_time:.2f} ms, Ground: {n_ground}/{len(points)}")
            else:
                frame_results[model_name] = {'error': 'Detection failed'}
                print("Failed")
        
        return frame_results
    
    def save_results(self, results, dataset_type):
        """Save benchmark results."""
        
        # Save JSON results
        results_file = self.session_dir / f"{dataset_type}_results.json"
        with open(results_file, 'w') as f:
            json.dump(results, f, indent=2)
        
        # Generate summary report
        report_file = self.session_dir / f"{dataset_type}_report.md"
        with open(report_file, 'w') as f:
            f.write(f"# Benchmark Results: {dataset_type.upper()} Dataset\n\n")
            f.write(f"Generated: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}\n\n")
            
            f.write("## Performance Summary\n\n")
            f.write("| Frame | Model | Time (ms) | Ground Points | Ratio |\n")
            f.write("|-------|-------|-----------|---------------|-------|\n")
            
            for frame_id, frame_results in results.items():
                for model_name, metrics in frame_results.items():
                    if 'error' not in metrics:
                        f.write(f"| {frame_id} | {model_name} | "
                               f"{metrics['time_ms']:.2f} | "
                               f"{metrics['ground_points']}/{metrics['total_points']} | "
                               f"{metrics['ground_ratio']:.2%} |\n")
        
        print(f"\nResults saved to: {self.session_dir}")


def main():
    print("="*60)
    print("REAL DATASET BENCHMARK")
    print("="*60)
    
    benchmark = RealDatasetBenchmark()
    
    print("\nDataset Options:")
    print("1. KITTI (Autonomous Driving)")
    print("2. NYU Depth V2 (Indoor RGB-D)")
    print("3. Custom RGB-D Data")
    print("4. Download Instructions")
    
    # Show download instructions
    benchmark.download_sample_data()
    
    # Run on available datasets
    print("\nChecking for available datasets...")
    
    # Try custom data first (easiest to set up)
    custom_dir = benchmark.dataset_dir / "custom"
    custom_dir.mkdir(exist_ok=True)
    
    print(f"\nTo use custom RGB-D data:")
    print(f"1. Place RGB images in: {custom_dir}")
    print(f"   - Name format: *_rgb.png")
    print(f"2. Place depth images in: {custom_dir}")
    print(f"   - Name format: *_depth.png")
    print(f"3. Ensure matching names (e.g., frame001_rgb.png, frame001_depth.png)")
    
    # Try to run on available data
    if (custom_dir / "sample_rgb.png").exists():
        benchmark.run_benchmark_on_dataset("custom")
    else:
        print("\nNo custom data found. Creating sample data for demonstration...")
        # Create a sample for testing
        sample_rgb = np.zeros((480, 640, 3), dtype=np.uint8)
        sample_rgb[:240, :] = [135, 206, 235]  # Sky
        sample_rgb[240:, :] = [100, 100, 100]  # Ground
        cv2.imwrite(str(custom_dir / "sample_rgb.png"), cv2.cvtColor(sample_rgb, cv2.COLOR_RGB2BGR))
        
        sample_depth = np.ones((480, 640), dtype=np.uint16) * 3000  # 3 meters in mm
        sample_depth[240:, :] = 1000  # 1 meter for ground
        cv2.imwrite(str(custom_dir / "sample_depth.png"), sample_depth)
        
        print("Sample data created. Running benchmark...")
        benchmark.run_benchmark_on_dataset("custom")


if __name__ == "__main__":
    main()