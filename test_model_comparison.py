#!/usr/bin/env python3
"""
Test script to compare ground detection methods:
1. Original RANSAC-based detection
2. MiDaS depth estimation
3. SegFormer semantic segmentation  
4. Detectron2 panoptic segmentation

Usage:
    python test_model_comparison.py
"""

import os
os.environ['KMP_DUPLICATE_LIB_OK'] = 'TRUE'  # Fix OpenMP issue
import numpy as np
import cv2
import time
import sys
sys.path.append(os.path.dirname(os.path.abspath(__file__)))

from bridge_local_planner.gtrack_mapper_with_models import GTrackMapper


def generate_synthetic_data():
    """Generate synthetic point cloud and RGB image for testing."""
    # Generate a simple floor plane with some objects
    n_points = 10000
    
    # Floor points (80% of points)
    n_floor = int(n_points * 0.8)
    floor_x = np.random.uniform(-5, 5, n_floor)
    floor_y = np.random.uniform(-5, 5, n_floor)
    floor_z = np.random.normal(0, 0.05, n_floor)  # Small noise around z=0
    floor_pts = np.stack([floor_x, floor_y, floor_z], axis=1)
    
    # Object points (20% of points)
    n_objects = n_points - n_floor
    obj_x = np.random.uniform(-2, 2, n_objects)
    obj_y = np.random.uniform(-2, 2, n_objects)
    obj_z = np.random.uniform(0.5, 2.0, n_objects)  # Objects above ground
    obj_pts = np.stack([obj_x, obj_y, obj_z], axis=1)
    
    # Combine points
    points = np.vstack([floor_pts, obj_pts])
    
    # Create synthetic RGB image (640x480)
    height, width = 480, 640
    rgb_image = np.zeros((height, width, 3), dtype=np.uint8)
    
    # Sky/background (top half)
    rgb_image[:height//2, :] = [135, 206, 235]  # Sky blue
    
    # Floor (bottom half)
    rgb_image[height//2:, :] = [128, 128, 128]  # Gray floor
    
    # Add some objects
    for _ in range(5):
        x = np.random.randint(50, width-50)
        y = np.random.randint(height//3, 2*height//3)
        w = np.random.randint(20, 60)
        h = np.random.randint(30, 80)
        color = np.random.randint(0, 255, 3).tolist()
        cv2.rectangle(rgb_image, (x, y), (x+w, y+h), color, -1)
    
    # Create depth image (optional)
    depth_image = np.ones((height, width), dtype=np.float32) * 5.0
    depth_image[height//2:, :] = 1.0  # Floor is closer
    
    return points, rgb_image, depth_image


def test_model_comparison():
    """Test and compare different ground detection methods."""
    
    print("Initializing GTrackMapper...")
    mapper = GTrackMapper()
    
    # Generate or load test data
    print("Generating synthetic test data...")
    points, rgb_image, depth_image = generate_synthetic_data()
    
    # Test models
    models_to_test = ['midas', 'segformer', 'detectron2', 'sam', 'sam2']
    
    print("\n" + "="*60)
    print("TESTING GROUND DETECTION MODELS")
    print("="*60)
    
    for model_name in models_to_test:
        print(f"\nTesting {model_name}...")
        
        # Set RGB and depth images
        mapper.set_rgb_image(rgb_image)
        mapper.set_depth_image(depth_image)
        
        # Enable model detection
        mapper.set_detection_method(use_model=True, model_name=model_name)
        
        # Apply point cloud (this will run both original and model detection)
        success = mapper.apply_pointcloud(points)
        
        if success:
            print(f"[OK] {model_name} completed successfully")
        else:
            print(f"[FAIL] {model_name} failed")
    
    # Print final timing summary
    mapper.print_timing_summary()
    
    # Test with original method only
    print("\nTesting original RANSAC only...")
    mapper.set_detection_method(use_model=False)
    
    for i in range(5):
        success = mapper.apply_pointcloud(points)
        if not success:
            print(f"  Iteration {i+1}: Failed")
    
    # Final summary
    print("\n" + "="*60)
    print("FINAL TIMING SUMMARY")
    mapper.print_timing_summary()


def test_with_real_data(rgb_path=None, depth_path=None, pcd_path=None):
    """Test with real sensor data if available.
    
    Args:
        rgb_path: Path to RGB image
        depth_path: Path to depth image  
        pcd_path: Path to point cloud file
    """
    mapper = GTrackMapper()
    
    # Load real data
    if rgb_path and os.path.exists(rgb_path):
        rgb_image = cv2.imread(rgb_path)
        mapper.set_rgb_image(rgb_image)
        print(f"Loaded RGB image: {rgb_path}")
    
    if depth_path and os.path.exists(depth_path):
        depth_image = cv2.imread(depth_path, cv2.IMREAD_ANYDEPTH)
        mapper.set_depth_image(depth_image)
        print(f"Loaded depth image: {depth_path}")
    
    if pcd_path and os.path.exists(pcd_path):
        # Load point cloud (assuming numpy array or similar format)
        points = np.load(pcd_path) if pcd_path.endswith('.npy') else None
        
        if points is not None:
            # Test all models
            for model_name in ['midas', 'segformer', 'detectron2']:
                print(f"\nTesting {model_name} on real data...")
                mapper.set_detection_method(use_model=True, model_name=model_name)
                mapper.apply_pointcloud(points)
            
            mapper.print_timing_summary()


if __name__ == "__main__":
    # Run synthetic test
    test_model_comparison()
    
    # Uncomment to test with real data
    # test_with_real_data(
    #     rgb_path="path/to/rgb.png",
    #     depth_path="path/to/depth.png", 
    #     pcd_path="path/to/points.npy"
    # )