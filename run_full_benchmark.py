#!/usr/bin/env python3
"""
Full benchmark comparing RANSAC vs semantic segmentation models.
"""

import os
os.environ['KMP_DUPLICATE_LIB_OK'] = 'TRUE'
import numpy as np
import cv2
import time
import sys
sys.path.append(os.path.dirname(os.path.abspath(__file__)))

from bridge_local_planner.gtrack_mapper_with_models import GTrackMapper

def generate_realistic_scene():
    """Generate realistic point cloud and RGB image."""
    n_points = 20000
    
    # Ground plane with slight variations
    n_ground = int(n_points * 0.7)
    ground_x = np.random.uniform(-5, 5, n_ground)
    ground_y = np.random.uniform(-5, 5, n_ground)
    # Add small noise to simulate real ground
    ground_z = np.random.normal(0, 0.02, n_ground)
    ground_pts = np.stack([ground_x, ground_y, ground_z], axis=1)
    
    # Add some obstacles
    n_obstacles = n_points - n_ground
    # Create clusters of obstacles
    obstacle_pts = []
    for _ in range(5):  # 5 obstacle clusters
        cluster_size = n_obstacles // 5
        center_x = np.random.uniform(-3, 3)
        center_y = np.random.uniform(-3, 3)
        obs_x = np.random.normal(center_x, 0.3, cluster_size)
        obs_y = np.random.normal(center_y, 0.3, cluster_size)
        obs_z = np.random.uniform(0.2, 1.5, cluster_size)
        obstacle_pts.append(np.stack([obs_x, obs_y, obs_z], axis=1))
    
    obstacle_pts = np.vstack(obstacle_pts)
    points = np.vstack([ground_pts, obstacle_pts])
    
    # Create corresponding RGB image
    height, width = 480, 640
    rgb_image = np.zeros((height, width, 3), dtype=np.uint8)
    
    # Sky (top third)
    rgb_image[:height//3, :] = [135, 206, 235]
    
    # Ground (bottom two thirds) 
    rgb_image[height//3:, :] = [100, 100, 100]
    
    # Add some obstacle representations
    for i in range(10):
        x = np.random.randint(50, width-50)
        y = np.random.randint(height//3, 2*height//3)
        radius = np.random.randint(10, 30)
        color = (np.random.randint(50, 200), np.random.randint(50, 200), np.random.randint(50, 200))
        cv2.circle(rgb_image, (x, y), radius, color, -1)
    
    return points, rgb_image

def run_benchmark():
    """Run full benchmark."""
    print("="*70)
    print("SEMANTIC SEGMENTATION vs RANSAC BENCHMARK")
    print("="*70)
    
    # Initialize mapper
    mapper = GTrackMapper()
    mapper.set_params({'debug_info': True})
    
    # Generate test data
    print("\nGenerating realistic test scene...")
    points, rgb_image = generate_realistic_scene()
    print(f"  Point cloud: {points.shape[0]} points")
    print(f"  RGB image: {rgb_image.shape}")
    
    # Models to test
    models = ['midas', 'segformer', 'detectron2', 'sam', 'sam2']
    
    results = {}
    
    # Test original RANSAC first (baseline)
    print("\n" + "-"*70)
    print("BASELINE: Original RANSAC")
    print("-"*70)
    
    mapper.set_detection_method(use_model=False)
    ransac_times = []
    ransac_success = 0
    
    for i in range(5):
        start = time.time()
        success = mapper.apply_pointcloud(points)
        elapsed = time.time() - start
        ransac_times.append(elapsed)
        if success:
            ransac_success += 1
            if i == 0 and mapper.debug_info.get('ground_mask') is not None:
                n_ground = np.sum(mapper.debug_info['ground_mask'])
                print(f"  Ground points detected: {n_ground}/{len(points)} ({100*n_ground/len(points):.1f}%)")
    
    ransac_mean = np.mean(ransac_times) * 1000
    ransac_std = np.std(ransac_times) * 1000
    print(f"  Success rate: {ransac_success}/5")
    print(f"  Time: {ransac_mean:.2f} ± {ransac_std:.2f} ms")
    results['ransac'] = {'mean': ransac_mean, 'std': ransac_std, 'success': ransac_success}
    
    # Test each model
    for model_name in models:
        print("\n" + "-"*70)
        print(f"MODEL: {model_name.upper()}")
        print("-"*70)
        
        mapper.set_rgb_image(rgb_image)
        mapper.set_detection_method(use_model=True, model_name=model_name)
        
        model_times = []
        model_success = 0
        
        try:
            for i in range(3):  # 3 runs per model
                print(f"  Run {i+1}/3...")
                start = time.time()
                success = mapper.apply_pointcloud(points)
                elapsed = time.time() - start
                
                if success:
                    model_success += 1
                    model_times.append(elapsed)
                    
                    if i == 0 and mapper.debug_info.get('ground_mask') is not None:
                        n_ground = np.sum(mapper.debug_info['ground_mask'])
                        print(f"    Ground points: {n_ground}/{len(points)} ({100*n_ground/len(points):.1f}%)")
                else:
                    print(f"    Failed")
            
            if model_times:
                model_mean = np.mean(model_times) * 1000
                model_std = np.std(model_times) * 1000
                speedup = ransac_mean / model_mean if model_mean > 0 else 0
                
                print(f"  Success rate: {model_success}/3")
                print(f"  Time: {model_mean:.2f} ± {model_std:.2f} ms")
                print(f"  Speedup vs RANSAC: {speedup:.2f}x")
                
                results[model_name] = {
                    'mean': model_mean, 
                    'std': model_std, 
                    'success': model_success,
                    'speedup': speedup
                }
            else:
                print(f"  All runs failed")
                results[model_name] = {'mean': 0, 'std': 0, 'success': 0, 'speedup': 0}
                
        except Exception as e:
            print(f"  Error: {e}")
            results[model_name] = {'mean': 0, 'std': 0, 'success': 0, 'speedup': 0}
    
    # Print summary
    print("\n" + "="*70)
    print("BENCHMARK SUMMARY")
    print("="*70)
    print(f"{'Method':<15} {'Time (ms)':<20} {'Success':<10} {'Speedup':<10}")
    print("-"*70)
    
    print(f"{'RANSAC':<15} {results['ransac']['mean']:.2f} ± {results['ransac']['std']:.2f}"
          f"{'':<5} {results['ransac']['success']}/5"
          f"{'':<5} {'1.00x':<10}")
    
    for model in models:
        if model in results:
            r = results[model]
            if r['mean'] > 0:
                print(f"{model.upper():<15} {r['mean']:.2f} ± {r['std']:.2f}"
                      f"{'':<5} {r['success']}/3"
                      f"{'':<5} {r['speedup']:.2f}x")
            else:
                print(f"{model.upper():<15} {'FAILED':<20} {r['success']}/3"
                      f"{'':<5} {'-':<10}")
    
    print("="*70)
    
    # Print winner
    best_speedup = 0
    best_model = None
    for model in models:
        if model in results and results[model]['speedup'] > best_speedup:
            best_speedup = results[model]['speedup']
            best_model = model
    
    if best_model and best_speedup > 1:
        print(f"\nFASTEST MODEL: {best_model.upper()} ({best_speedup:.2f}x faster than RANSAC)")
    else:
        print(f"\nRESULT: RANSAC is faster than all tested models")

if __name__ == "__main__":
    run_benchmark()