#!/usr/bin/env python3
"""
Complete test script to compare all ground detection models.

Available models:
1. Original RANSAC - Geometric plane fitting
2. MiDaS - Depth estimation based ground detection  
3. SegFormer - Transformer-based semantic segmentation
4. Detectron2 - Facebook's panoptic segmentation
5. SAM - Segment Anything Model v1
6. SAM2 - Segment Anything Model v2 (latest)

Usage:
    python run_model_comparison.py
"""

import numpy as np
import cv2
import sys
import os

# Add path for imports
sys.path.append(os.path.dirname(os.path.abspath(__file__)))

from bridge_local_planner.gtrack_mapper_with_models import GTrackMapper


def generate_test_data():
    """Generate synthetic test data (point cloud + RGB + depth)."""
    
    # Generate point cloud
    n_points = 10000
    
    # Floor points (80%)
    n_floor = int(n_points * 0.8)
    floor_x = np.random.uniform(-5, 5, n_floor)
    floor_y = np.random.uniform(-5, 5, n_floor)
    floor_z = np.random.normal(0, 0.05, n_floor)
    floor_pts = np.stack([floor_x, floor_y, floor_z], axis=1)
    
    # Object points (20%)
    n_objects = n_points - n_floor
    obj_x = np.random.uniform(-2, 2, n_objects)
    obj_y = np.random.uniform(-2, 2, n_objects)
    obj_z = np.random.uniform(0.5, 2.0, n_objects)
    obj_pts = np.stack([obj_x, obj_y, obj_z], axis=1)
    
    points = np.vstack([floor_pts, obj_pts])
    
    # Generate RGB image
    height, width = 480, 640
    rgb_image = np.zeros((height, width, 3), dtype=np.uint8)
    
    # Sky (top half)
    rgb_image[:height//2, :] = [135, 206, 235]
    
    # Floor (bottom half)
    rgb_image[height//2:, :] = [128, 128, 128]
    
    # Add objects
    for _ in range(5):
        x = np.random.randint(50, width-50)
        y = np.random.randint(height//3, 2*height//3)
        w = np.random.randint(20, 60)
        h = np.random.randint(30, 80)
        color = np.random.randint(0, 255, 3).tolist()
        cv2.rectangle(rgb_image, (x, y), (x+w, y+h), color, -1)
    
    # Generate depth image
    depth_image = np.ones((height, width), dtype=np.float32) * 5.0
    depth_image[height//2:, :] = 1.0  # Floor is closer
    
    return points, rgb_image, depth_image


def main():
    """Main function to run model comparison."""
    
    print("="*70)
    print("GROUND DETECTION MODEL COMPARISON")
    print("="*70)
    
    # Available models with descriptions
    models = {
        'original': 'Original RANSAC (geometric plane fitting)',
        'midas': 'MiDaS (depth estimation → ground detection)',
        'segformer': 'SegFormer (transformer-based segmentation)',
        'detectron2': 'Detectron2 (panoptic segmentation)',
        'sam': 'SAM v1 (Segment Anything Model)',
        'sam2': 'SAM v2 (latest Segment Anything Model)'
    }
    
    print("\nAvailable models:")
    for i, (key, desc) in enumerate(models.items(), 1):
        print(f"  {i}. {key:12s} - {desc}")
    
    # Initialize mapper
    print("\n" + "-"*70)
    print("Initializing GTrackMapper...")
    mapper = GTrackMapper()
    
    # Generate test data
    print("Generating synthetic test data...")
    points, rgb_image, depth_image = generate_test_data()
    
    # Set images for model-based detection
    mapper.set_rgb_image(rgb_image)
    mapper.set_depth_image(depth_image)
    
    # Test each model
    print("\n" + "-"*70)
    print("Testing models (this may take a while on first run)...")
    print("-"*70)
    
    # Test original RANSAC only
    print(f"\n1. Testing ORIGINAL RANSAC only:")
    mapper.set_detection_method(use_model=False)
    success = mapper.apply_pointcloud(points)
    if success:
        print("   ✓ Original RANSAC completed")
    else:
        print("   ✗ Original RANSAC failed")
    
    # Test each semantic segmentation model
    test_models = ['midas', 'segformer', 'detectron2']  # SAM models require weights
    
    for i, model_name in enumerate(test_models, 2):
        print(f"\n{i}. Testing {model_name.upper()}:")
        print(f"   Description: {models[model_name]}")
        
        try:
            # Enable model detection
            mapper.set_detection_method(use_model=True, model_name=model_name)
            
            # Apply point cloud (runs both original and model for comparison)
            success = mapper.apply_pointcloud(points)
            
            if success:
                print(f"   ✓ {model_name} completed")
            else:
                print(f"   ✗ {model_name} failed")
                
        except Exception as e:
            print(f"   ✗ Error with {model_name}: {e}")
    
    # Print timing summary
    print("\n" + "="*70)
    mapper.print_timing_summary()
    
    # Instructions for SAM models
    print("\n" + "-"*70)
    print("NOTE: SAM and SAM2 require model weights to be downloaded:")
    print("  - SAM:  wget https://dl.fbaipublicfiles.com/segment_anything/sam_vit_h_4b8939.pth")
    print("  - SAM2: Download from the official SAM2 repository")
    print("-"*70)


def example_usage():
    """Show example usage in code."""
    
    print("\n" + "="*70)
    print("EXAMPLE CODE USAGE")
    print("="*70)
    
    print("""
from bridge_local_planner.gtrack_mapper_with_models import GTrackMapper
import cv2
import numpy as np

# Initialize mapper
mapper = GTrackMapper()

# Load your data
points = np.load('your_pointcloud.npy')  # Your 3D points
rgb_image = cv2.imread('your_image.jpg')  # Your RGB image

# Set images for model detection
mapper.set_rgb_image(rgb_image)
mapper.set_depth_image(depth_image)  # Optional for RGB-D models

# Method 1: Use original RANSAC
mapper.set_detection_method(use_model=False)
mapper.apply_pointcloud(points)

# Method 2: Use MiDaS model
mapper.set_detection_method(use_model=True, model_name='midas')
mapper.apply_pointcloud(points)

# Method 3: Use SegFormer model
mapper.set_detection_method(use_model=True, model_name='segformer')
mapper.apply_pointcloud(points)

# Method 4: Use Detectron2 model
mapper.set_detection_method(use_model=True, model_name='detectron2')
mapper.apply_pointcloud(points)

# Method 5: Use SAM (requires weights)
mapper.set_detection_method(use_model=True, model_name='sam')
mapper.apply_pointcloud(points)

# Method 6: Use SAM2 (requires weights)
mapper.set_detection_method(use_model=True, model_name='sam2')
mapper.apply_pointcloud(points)

# View timing comparison
mapper.print_timing_summary()

# Access the map data
obstacles = mapper.map_data['obstacles']
elevation = mapper.map_data['elevation']
histogram = mapper.map_data['histogram']

# Visualize
mapper.imshow_map_pyplot(obstacles, 'Obstacle Map')
""")


if __name__ == "__main__":
    # Run main comparison
    main()
    
    # Show example usage
    example_usage()
    
    print("\n✅ Model comparison complete!")