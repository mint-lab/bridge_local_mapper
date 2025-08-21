#!/usr/bin/env python3
"""
Optimized version of quick benchmark demo with memory management.
Prevents freezing by cleaning up models between runs.
"""

import os
os.environ['KMP_DUPLICATE_LIB_OK'] = 'TRUE'
import torch
import gc
import argparse
import psutil

# Import the full benchmark
from benchmark_kitti_rgbd_1000 import LargeScaleBenchmark

def clear_memory():
    """Clear GPU cache and run garbage collection."""
    gc.collect()
    if torch.cuda.is_available():
        torch.cuda.empty_cache()
        torch.cuda.synchronize()

def print_memory_usage():
    """Print current memory usage."""
    ram = psutil.virtual_memory()
    print(f"  RAM: {ram.percent:.1f}% ({ram.used/1e9:.1f}/{ram.total/1e9:.1f} GB)")
    if torch.cuda.is_available():
        print(f"  GPU Memory: {torch.cuda.memory_allocated()/1e9:.2f} GB allocated")

def main():
    parser = argparse.ArgumentParser(description='Optimized benchmark demo')
    parser.add_argument('--cpu-only', action='store_true', help='Force CPU-only mode')
    parser.add_argument('--kitti-samples', type=int, default=10, help='Number of KITTI samples (default: 10)')
    parser.add_argument('--rgbd-samples', type=int, default=10, help='Number of RGB-D samples (default: 10)') 
    args = parser.parse_args()
    
    # Force CPU if requested
    if args.cpu_only:
        os.environ['CUDA_VISIBLE_DEVICES'] = '-1'
        torch.cuda.is_available = lambda: False
        print("Running in CPU-only mode")
    
    print("="*70)
    print(f"OPTIMIZED QUICK BENCHMARK DEMO: {args.kitti_samples} KITTI + {args.rgbd_samples} RGB-D IMAGES")
    print("="*70)
    print("\nThis optimized version includes:")
    print("  - Memory cleanup between models")
    print("  - GPU cache clearing (if GPU available)")
    print("  - Model deletion after each frame")
    print("="*70)
    
    benchmark = LargeScaleBenchmark(output_dir="benchmark_demo_optimized")
    
    # Override the benchmark_single_frame method with memory management
    original_benchmark = benchmark.benchmark_single_frame
    
    def benchmark_single_frame_with_cleanup(mapper, points, rgb=None, depth=None):
        """Wrapper that adds memory cleanup."""
        if points is None or len(points) < 100:
            return None
        
        frame_results = {}
        models = ['original_ransac', 'midas', 'segformer', 'detectron2']  # All 4 models
        
        # Set images
        if rgb is not None:
            mapper.set_rgb_image(rgb)
        if depth is not None:
            mapper.set_depth_image(depth)
        
        for model in models:
            print(f"    Testing {model}...", end=" ")
            
            # Clear memory before each model
            clear_memory()
            
            if model == 'original_ransac':
                mapper.set_detection_method(use_model=False)
            else:
                # Clean up previous model if it exists
                if hasattr(mapper, 'model_detector') and mapper.model_detector is not None:
                    if hasattr(mapper.model_detector, 'model') and mapper.model_detector.model is not None:
                        del mapper.model_detector.model
                        mapper.model_detector.model = None
                    del mapper.model_detector
                    mapper.model_detector = None
                    clear_memory()
                
                mapper.set_detection_method(use_model=True, model_name=model)
            
            # Time the detection
            import time
            import numpy as np
            start = time.time()
            
            try:
                success = mapper.apply_pointcloud(points.copy())
                elapsed = (time.time() - start) * 1000  # ms
                
                if success and 'ground_mask' in mapper.debug_info:
                    ground_mask = mapper.debug_info['ground_mask']
                    n_ground = np.sum(ground_mask)
                    
                    frame_results[model] = {
                        'time_ms': elapsed,
                        'ground_points': int(n_ground),
                        'total_points': len(points),
                        'ground_ratio': float(n_ground / len(points))
                    }
                    print(f"OK {elapsed:.1f}ms")
                else:
                    frame_results[model] = {'error': 'Failed'}
                    print(f"FAILED")
                    
            except Exception as e:
                print(f"ERROR: {str(e)[:50]}")
                frame_results[model] = {'error': str(e)[:100]}
            
            # Clean up model after use (except RANSAC)
            if model != 'original_ransac':
                if hasattr(mapper, 'model_detector') and mapper.model_detector is not None:
                    if hasattr(mapper.model_detector, 'model'):
                        del mapper.model_detector.model
                    mapper.model_detector = None
                clear_memory()
        
        return frame_results
    
    # Replace the method
    benchmark.benchmark_single_frame = benchmark_single_frame_with_cleanup
    
    print("\nInitial memory status:")
    print_memory_usage()
    
    # Run with specified samples
    print(f"\nRunning KITTI benchmark ({args.kitti_samples} samples)...")
    benchmark.run_kitti_benchmark(n_samples=args.kitti_samples)
    
    # Clear memory between datasets
    clear_memory()
    print("\nMemory after KITTI:")
    print_memory_usage()
    
    print(f"\nRunning RGB-D benchmark ({args.rgbd_samples} samples)...")
    benchmark.run_rgbd_benchmark(n_samples=args.rgbd_samples)
    
    # Clear memory after RGB-D
    clear_memory()
    print("\nMemory after RGB-D:")
    print_memory_usage()
    
    # Generate plots
    print("\nGenerating visualization plots...")
    benchmark.generate_plots()
    
    # Save all results
    print("\nSaving results...")
    benchmark.save_results()
    
    print("\n" + "="*70)
    print("OPTIMIZED DEMO COMPLETE")
    print("="*70)
    print(f"Results directory: {benchmark.session_dir}")
    
    print("\nFinal memory status:")
    print_memory_usage()
    
    print("\nTo run full 1000+1000 benchmark:")
    print("  python benchmark_kitti_rgbd_1000.py")

if __name__ == "__main__":
    main()