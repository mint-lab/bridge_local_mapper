#!/usr/bin/env python3
"""
Comprehensive benchmark for semantic segmentation models vs RANSAC ground detection.
Includes warm-up runs, multiple iterations, and detailed performance metrics.
"""

import os
os.environ['KMP_DUPLICATE_LIB_OK'] = 'TRUE'
import numpy as np
import cv2
import time
import sys
import json
from datetime import datetime
sys.path.append(os.path.dirname(os.path.abspath(__file__)))

from bridge_local_planner.gtrack_mapper_with_models import GTrackMapper


class GroundDetectionBenchmark:
    def __init__(self, n_warmup=2, n_iterations=10):
        self.n_warmup = n_warmup
        self.n_iterations = n_iterations
        self.results = {}
        
    def generate_test_scenarios(self):
        """Generate different test scenarios with varying complexity."""
        scenarios = []
        
        # Scenario 1: Simple flat ground
        scenarios.append({
            'name': 'simple_flat',
            'n_points': 5000,
            'ground_ratio': 0.9,
            'noise_level': 0.01,
            'n_objects': 2
        })
        
        # Scenario 2: Complex scene with many objects
        scenarios.append({
            'name': 'complex_scene',
            'n_points': 10000,
            'ground_ratio': 0.6,
            'noise_level': 0.05,
            'n_objects': 10
        })
        
        # Scenario 3: Large point cloud
        scenarios.append({
            'name': 'large_cloud',
            'n_points': 20000,
            'ground_ratio': 0.7,
            'noise_level': 0.03,
            'n_objects': 5
        })
        
        return scenarios
    
    def generate_synthetic_scene(self, n_points=10000, ground_ratio=0.8, 
                                 noise_level=0.05, n_objects=5):
        """Generate synthetic point cloud and RGB/depth images."""
        
        # Ground points
        n_ground = int(n_points * ground_ratio)
        ground_x = np.random.uniform(-5, 5, n_ground)
        ground_y = np.random.uniform(-5, 5, n_ground)
        ground_z = np.random.normal(0, noise_level, n_ground)
        ground_pts = np.stack([ground_x, ground_y, ground_z], axis=1)
        
        # Object points
        n_obj_pts = n_points - n_ground
        obj_pts_list = []
        pts_per_obj = n_obj_pts // n_objects
        
        for i in range(n_objects):
            obj_x = np.random.uniform(-3, 3, pts_per_obj) + np.random.uniform(-2, 2)
            obj_y = np.random.uniform(-3, 3, pts_per_obj) + np.random.uniform(-2, 2)
            obj_z = np.random.uniform(0.3, 2.0, pts_per_obj)
            obj_pts_list.append(np.stack([obj_x, obj_y, obj_z], axis=1))
        
        obj_pts = np.vstack(obj_pts_list)
        
        # Combine all points
        all_points = np.vstack([ground_pts, obj_pts])
        
        # Generate RGB image
        height, width = 480, 640
        rgb_image = np.zeros((height, width, 3), dtype=np.uint8)
        
        # Sky gradient
        for i in range(height//2):
            color_val = int(135 + (70 * i / (height//2)))
            rgb_image[i, :] = [color_val, 180 + i//4, 235]
        
        # Ground texture
        ground_color = np.random.randint(100, 150, (height//2, width, 3))
        rgb_image[height//2:, :] = ground_color
        
        # Add objects
        for i in range(n_objects):
            x = np.random.randint(50, width-50)
            y = np.random.randint(height//3, 2*height//3)
            w = np.random.randint(30, 80)
            h = np.random.randint(40, 100)
            color = np.random.randint(50, 200, 3).tolist()
            cv2.rectangle(rgb_image, (x, y), (min(x+w, width-1), min(y+h, height-1)), color, -1)
        
        # Generate depth image
        depth_image = np.ones((height, width), dtype=np.float32) * 5.0
        depth_image[height//2:, :] = 1.0 + np.random.normal(0, 0.1, (height//2, width))
        
        # Add object depths
        for i in range(n_objects):
            x = np.random.randint(50, width-50)
            y = np.random.randint(height//3, 2*height//3)
            w = np.random.randint(30, 80)
            h = np.random.randint(40, 100)
            depth_val = np.random.uniform(1.5, 4.0)
            depth_image[y:min(y+h, height), x:min(x+w, width)] = depth_val
        
        return all_points, rgb_image, depth_image, n_ground
    
    def calculate_accuracy(self, detected_mask, true_n_ground, total_points):
        """Calculate detection accuracy metrics."""
        n_detected_ground = np.sum(detected_mask)
        
        # Approximate accuracy (since we know ground ratio)
        expected_ground = true_n_ground
        accuracy = 1.0 - abs(n_detected_ground - expected_ground) / total_points
        
        metrics = {
            'detected_ground_points': int(n_detected_ground),
            'expected_ground_points': int(expected_ground),
            'total_points': int(total_points),
            'ground_ratio': float(n_detected_ground / total_points),
            'accuracy_score': float(accuracy)
        }
        
        return metrics
    
    def run_benchmark(self):
        """Run the full benchmark suite."""
        print("="*70)
        print("SEMANTIC SEGMENTATION vs RANSAC GROUND DETECTION BENCHMARK")
        print("="*70)
        print(f"Configuration: {self.n_warmup} warm-up runs, {self.n_iterations} test iterations")
        print()
        
        # Models to test
        models = ['original_ransac', 'midas', 'segformer', 'detectron2']
        scenarios = self.generate_test_scenarios()
        
        # Initialize mapper
        mapper = GTrackMapper()
        mapper.params['debug_info'] = True  # Enable debug info for accuracy
        
        for scenario in scenarios:
            print(f"\nScenario: {scenario['name']}")
            print(f"  Points: {scenario['n_points']}, Ground ratio: {scenario['ground_ratio']}")
            print("-"*50)
            
            scenario_results = {}
            
            # Generate test data for this scenario
            points, rgb_image, depth_image, n_ground = self.generate_synthetic_scene(
                n_points=scenario['n_points'],
                ground_ratio=scenario['ground_ratio'],
                noise_level=scenario['noise_level'],
                n_objects=scenario['n_objects']
            )
            
            mapper.set_rgb_image(rgb_image)
            mapper.set_depth_image(depth_image)
            
            for model_name in models:
                print(f"\n  Testing {model_name}...")
                
                # Configure model
                if model_name == 'original_ransac':
                    mapper.set_detection_method(use_model=False)
                else:
                    mapper.set_detection_method(use_model=True, model_name=model_name)
                
                timings = []
                accuracies = []
                
                # Warm-up runs
                print(f"    Warm-up runs...", end='')
                for _ in range(self.n_warmup):
                    mapper.apply_pointcloud(points.copy())
                print(" done")
                
                # Reset timing results
                mapper.timing_results = {
                    'original_ransac': [],
                    'model_detection': []
                }
                
                # Benchmark runs
                print(f"    Benchmark runs: ", end='')
                for i in range(self.n_iterations):
                    start = time.time()
                    success = mapper.apply_pointcloud(points.copy())
                    total_time = time.time() - start
                    
                    if success:
                        timings.append(total_time)
                        
                        # Calculate accuracy
                        if mapper.debug_info and 'ground_mask' in mapper.debug_info:
                            ground_mask = mapper.debug_info['ground_mask']
                            accuracy = self.calculate_accuracy(ground_mask, n_ground, len(points))
                            accuracies.append(accuracy['accuracy_score'])
                        
                        print(".", end='', flush=True)
                    else:
                        print("x", end='', flush=True)
                
                print()
                
                # Calculate statistics
                if timings:
                    avg_time = np.mean(timings) * 1000  # ms
                    std_time = np.std(timings) * 1000
                    min_time = np.min(timings) * 1000
                    max_time = np.max(timings) * 1000
                    
                    avg_accuracy = np.mean(accuracies) if accuracies else 0
                    
                    scenario_results[model_name] = {
                        'avg_time_ms': avg_time,
                        'std_time_ms': std_time,
                        'min_time_ms': min_time,
                        'max_time_ms': max_time,
                        'avg_accuracy': avg_accuracy,
                        'n_successful': len(timings)
                    }
                    
                    print(f"    Results: {avg_time:.2f}±{std_time:.2f} ms, "
                          f"Accuracy: {avg_accuracy:.2%}")
                else:
                    scenario_results[model_name] = {
                        'error': 'No successful runs'
                    }
                    print(f"    Results: Failed")
            
            self.results[scenario['name']] = scenario_results
        
        return self.results
    
    def print_summary(self):
        """Print a summary table of results."""
        print("\n" + "="*70)
        print("BENCHMARK SUMMARY")
        print("="*70)
        
        if not self.results:
            print("No results to display")
            return
        
        # Print table header
        print(f"{'Scenario':<15} {'Model':<15} {'Avg Time (ms)':<15} {'Std Dev':<10} {'Accuracy':<10}")
        print("-"*70)
        
        for scenario_name, scenario_results in self.results.items():
            for model_name, metrics in scenario_results.items():
                if 'error' in metrics:
                    print(f"{scenario_name:<15} {model_name:<15} {'FAILED':<15}")
                else:
                    avg_time = metrics['avg_time_ms']
                    std_time = metrics['std_time_ms']
                    accuracy = metrics['avg_accuracy']
                    print(f"{scenario_name:<15} {model_name:<15} {avg_time:>8.2f}       "
                          f"{std_time:>8.2f}   {accuracy:>8.2%}")
        
        # Calculate speedups
        print("\n" + "="*70)
        print("SPEEDUP ANALYSIS (vs Original RANSAC)")
        print("="*70)
        
        for scenario_name, scenario_results in self.results.items():
            print(f"\n{scenario_name}:")
            
            if 'original_ransac' in scenario_results and 'avg_time_ms' in scenario_results['original_ransac']:
                ransac_time = scenario_results['original_ransac']['avg_time_ms']
                
                for model_name, metrics in scenario_results.items():
                    if model_name != 'original_ransac' and 'avg_time_ms' in metrics:
                        model_time = metrics['avg_time_ms']
                        speedup = ransac_time / model_time if model_time > 0 else 0
                        
                        if speedup > 1:
                            print(f"  {model_name:<15}: {speedup:.2f}x faster")
                        else:
                            print(f"  {model_name:<15}: {1/speedup:.2f}x slower")
    
    def save_results(self, filename=None):
        """Save results to JSON file."""
        if filename is None:
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            filename = f"benchmark_results_{timestamp}.json"
        
        with open(filename, 'w') as f:
            json.dump(self.results, f, indent=2)
        
        print(f"\nResults saved to {filename}")


def main():
    # Enable debug info for accuracy calculation
    benchmark = GroundDetectionBenchmark(n_warmup=2, n_iterations=5)
    
    # Run benchmark
    results = benchmark.run_benchmark()
    
    # Print summary
    benchmark.print_summary()
    
    # Save results
    benchmark.save_results()


if __name__ == "__main__":
    main()