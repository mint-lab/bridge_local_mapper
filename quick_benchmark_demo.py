#!/usr/bin/env python3
"""
Quick demo of the large-scale benchmark with reduced samples for testing.
Tests 10 KITTI + 10 RGB-D images to demonstrate the system.
"""

import os
os.environ['KMP_DUPLICATE_LIB_OK'] = 'TRUE'

# Import the full benchmark
from benchmark_kitti_rgbd_1000 import LargeScaleBenchmark

def main():
    print("="*70)
    print("QUICK BENCHMARK DEMO: 10 KITTI + 10 RGB-D IMAGES")
    print("="*70)
    print("\nThis is a demo with reduced samples.")
    print("For full 1000+1000 benchmark, run: python benchmark_kitti_rgbd_1000.py")
    print("="*70)
    
    benchmark = LargeScaleBenchmark(output_dir="benchmark_demo")
    
    # Run with reduced samples for quick demo
    print("\nRunning KITTI benchmark (10 samples)...")
    benchmark.run_kitti_benchmark(n_samples=10)
    
    print("\nRunning RGB-D benchmark (10 samples)...")
    benchmark.run_rgbd_benchmark(n_samples=10)
    
    # Generate plots
    print("\nGenerating visualization plots...")
    benchmark.generate_plots()
    
    # Save all results
    print("\nSaving results...")
    benchmark.save_results()
    
    print("\n" + "="*70)
    print("DEMO COMPLETE")
    print("="*70)
    print(f"Results directory: {benchmark.session_dir}")
    print("\nTo run full 1000+1000 benchmark:")
    print("  python benchmark_kitti_rgbd_1000.py")
    print("\nNote: Full benchmark will take approximately:")
    print("  - 30-60 minutes with CPU")
    print("  - 10-20 minutes with GPU")

if __name__ == "__main__":
    main()