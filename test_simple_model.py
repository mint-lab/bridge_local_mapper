#!/usr/bin/env python3
"""
Simple test to debug model detection.
"""

import os
os.environ['KMP_DUPLICATE_LIB_OK'] = 'TRUE'
import numpy as np
import cv2
import sys
sys.path.append(os.path.dirname(os.path.abspath(__file__)))

from bridge_local_planner.gtrack_mapper_with_models import GTrackMapper

def test_simple():
    print("Creating mapper...")
    mapper = GTrackMapper()
    
    # Generate simple test data
    print("Generating test data...")
    n_points = 5000
    
    # Ground points at z=0
    ground_x = np.random.uniform(-3, 3, n_points)
    ground_y = np.random.uniform(-3, 3, n_points) 
    ground_z = np.zeros(n_points)
    points = np.stack([ground_x, ground_y, ground_z], axis=1)
    
    # Simple RGB image - gray floor
    height, width = 480, 640
    rgb_image = np.ones((height, width, 3), dtype=np.uint8) * 128
    
    print(f"Points shape: {points.shape}")
    print(f"RGB image shape: {rgb_image.shape}")
    
    # Set images
    mapper.set_rgb_image(rgb_image)
    
    # Test original RANSAC
    print("\n1. Testing original RANSAC...")
    mapper.set_detection_method(use_model=False)
    success = mapper.apply_pointcloud(points)
    print(f"   Result: {'SUCCESS' if success else 'FAILED'}")
    
    if mapper.debug_info.get('ground_plane') is not None:
        print(f"   Ground plane: {mapper.debug_info['ground_plane']}")
        print(f"   Ground points: {np.sum(mapper.debug_info.get('ground_mask', []))} / {len(points)}")
    
    # Test with MiDaS
    print("\n2. Testing MiDaS model...")
    mapper.set_detection_method(use_model=True, model_name='midas')
    mapper.set_params({'debug_info': True})
    success = mapper.apply_pointcloud(points)
    print(f"   Result: {'SUCCESS' if success else 'FAILED'}")
    
    if mapper.debug_info.get('ground_plane') is not None:
        print(f"   Ground plane: {mapper.debug_info['ground_plane']}")
        print(f"   Ground points: {np.sum(mapper.debug_info.get('ground_mask', []))} / {len(points)}")

if __name__ == "__main__":
    test_simple()