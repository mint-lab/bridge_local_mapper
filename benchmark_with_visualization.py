#!/usr/bin/env python3
"""
Enhanced benchmark with visualization for semantic segmentation models.
Saves images showing what each model detects as ground vs objects.
"""

import os
os.environ['KMP_DUPLICATE_LIB_OK'] = 'TRUE'
import numpy as np
import cv2
import time
import sys
import json
import matplotlib.pyplot as plt
from datetime import datetime
from pathlib import Path
sys.path.append(os.path.dirname(os.path.abspath(__file__)))

from bridge_local_planner.gtrack_mapper_with_models import GTrackMapper


class VisualBenchmark:
    def __init__(self, output_dir="benchmark_results"):
        self.output_dir = Path(output_dir)
        self.timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.session_dir = self.output_dir / f"session_{self.timestamp}"
        self.session_dir.mkdir(parents=True, exist_ok=True)
        
        print(f"Results will be saved to: {self.session_dir}")
        
    def generate_realistic_scene(self, scenario_type="simple"):
        """Generate more realistic synthetic scenes with proper visualization."""
        
        if scenario_type == "simple_flat":
            # Parking lot or open field scenario
            n_points = 5000
            
            # Flat ground with very slight variations (like real pavement)
            ground_x = np.random.uniform(-5, 5, 4500)
            ground_y = np.random.uniform(-5, 5, 4500)
            # Slight undulation in ground (real ground isn't perfectly flat)
            ground_z = 0.02 * np.sin(ground_x/2) * np.cos(ground_y/2) + np.random.normal(0, 0.01, 4500)
            ground_pts = np.stack([ground_x, ground_y, ground_z], axis=1)
            
            # Two cars/obstacles
            obj_pts_list = []
            # Car 1
            car1_x = np.random.uniform(1, 2, 250) 
            car1_y = np.random.uniform(-1, 1, 250)
            car1_z = np.random.uniform(0.3, 1.5, 250)  # Car height
            obj_pts_list.append(np.stack([car1_x, car1_y, car1_z], axis=1))
            
            # Car 2
            car2_x = np.random.uniform(-2, -1, 250)
            car2_y = np.random.uniform(-1, 1, 250)
            car2_z = np.random.uniform(0.3, 1.5, 250)
            obj_pts_list.append(np.stack([car2_x, car2_y, car2_z], axis=1))
            
            obj_pts = np.vstack(obj_pts_list)
            scenario_desc = "Simple: Parking lot with 2 vehicles"
            
        elif scenario_type == "complex_indoor":
            # Indoor room with furniture
            n_points = 10000
            
            # Floor with some unevenness (tiles, carpet edges)
            ground_x = np.random.uniform(-4, 4, 6000)
            ground_y = np.random.uniform(-4, 4, 6000)
            ground_z = np.random.normal(0, 0.03, 6000)  # More variation indoors
            # Add threshold effect (like door threshold)
            threshold_mask = (ground_x > -0.5) & (ground_x < 0.5)
            ground_z[threshold_mask] += 0.02
            ground_pts = np.stack([ground_x, ground_y, ground_z], axis=1)
            
            # Multiple furniture pieces (10 objects)
            obj_pts_list = []
            furniture_types = [
                ("chair", 0.4, 0.8),      # name, min_height, max_height
                ("table", 0.7, 0.8),
                ("sofa", 0.3, 0.6),
                ("shelf", 0.1, 2.0),
                ("box", 0.2, 0.5),
            ]
            
            for i in range(10):
                ftype = furniture_types[i % len(furniture_types)]
                n_pts = 400
                # Random position for each furniture
                center_x = np.random.uniform(-3, 3)
                center_y = np.random.uniform(-3, 3)
                spread = 0.5
                
                obj_x = np.random.uniform(center_x-spread, center_x+spread, n_pts)
                obj_y = np.random.uniform(center_y-spread, center_y+spread, n_pts)
                obj_z = np.random.uniform(ftype[1], ftype[2], n_pts)
                obj_pts_list.append(np.stack([obj_x, obj_y, obj_z], axis=1))
            
            obj_pts = np.vstack(obj_pts_list)
            scenario_desc = "Complex: Indoor room with 10 furniture pieces"
            
        elif scenario_type == "outdoor_terrain":
            # Outdoor scene with uneven terrain
            n_points = 20000
            
            # Sloped/uneven ground (hillside, rough terrain)
            ground_x = np.random.uniform(-6, 6, 14000)
            ground_y = np.random.uniform(-6, 6, 14000)
            # Create terrain with slopes and bumps
            ground_z = 0.1 * ground_x + 0.05 * ground_y  # Overall slope
            ground_z += 0.2 * np.sin(ground_x) * np.cos(ground_y)  # Hills
            ground_z += np.random.normal(0, 0.05, 14000)  # Roughness
            ground_pts = np.stack([ground_x, ground_y, ground_z], axis=1)
            
            # Trees, rocks, bushes (5 large objects)
            obj_pts_list = []
            object_types = [
                ("tree", 0.5, 4.0, 1200),     # name, min_h, max_h, n_points
                ("rock", 0.3, 1.5, 1200),
                ("bush", 0.2, 1.0, 1200),
                ("bench", 0.4, 0.5, 1200),
                ("pole", 2.0, 3.0, 1200),
            ]
            
            for otype in object_types:
                center_x = np.random.uniform(-4, 4)
                center_y = np.random.uniform(-4, 4)
                spread = 0.8
                
                obj_x = np.random.uniform(center_x-spread, center_x+spread, otype[3])
                obj_y = np.random.uniform(center_y-spread, center_y+spread, otype[3])
                obj_z = np.random.uniform(otype[1], otype[2], otype[3])
                obj_pts_list.append(np.stack([obj_x, obj_y, obj_z], axis=1))
            
            obj_pts = np.vstack(obj_pts_list)
            scenario_desc = "Outdoor: Uneven terrain with trees and objects"
            
        else:
            raise ValueError(f"Unknown scenario type: {scenario_type}")
        
        # Combine all points
        all_points = np.vstack([ground_pts, obj_pts])
        n_ground = len(ground_pts)
        
        # Generate corresponding RGB image
        rgb_image = self.generate_rgb_image(scenario_type, ground_pts, obj_pts)
        
        # Generate depth image
        depth_image = self.generate_depth_image(all_points)
        
        return all_points, rgb_image, depth_image, n_ground, scenario_desc
    
    def generate_rgb_image(self, scenario_type, ground_pts, obj_pts):
        """Generate RGB image that corresponds to the point cloud."""
        height, width = 480, 640
        rgb_image = np.zeros((height, width, 3), dtype=np.uint8)
        
        if scenario_type == "simple_flat":
            # Sky
            rgb_image[:height//2, :] = [135, 206, 235]
            # Asphalt ground
            rgb_image[height//2:, :] = [80, 80, 80]
            # Add parking lines
            for i in range(0, width, 100):
                cv2.line(rgb_image, (i, height//2), (i, height), (255, 255, 255), 2)
            # Add cars (rectangular shapes)
            cv2.rectangle(rgb_image, (400, 250), (500, 350), (150, 50, 50), -1)
            cv2.rectangle(rgb_image, (150, 280), (250, 380), (50, 50, 150), -1)
            
        elif scenario_type == "complex_indoor":
            # Ceiling/walls
            rgb_image[:height//3, :] = [240, 240, 240]
            # Floor (wood/tile pattern)
            for i in range(height//3, height):
                for j in range(width):
                    if (i//20 + j//20) % 2 == 0:
                        rgb_image[i, j] = [139, 90, 43]  # Brown
                    else:
                        rgb_image[i, j] = [160, 110, 60]  # Light brown
            # Add furniture shapes
            for _ in range(10):
                x = np.random.randint(50, width-100)
                y = np.random.randint(height//3, height-50)
                w = np.random.randint(40, 80)
                h = np.random.randint(40, 80)
                color = [
                    np.random.randint(100, 200),
                    np.random.randint(50, 150),
                    np.random.randint(50, 150)
                ]
                cv2.rectangle(rgb_image, (x, y), (x+w, y+h), color, -1)
                
        elif scenario_type == "outdoor_terrain":
            # Sky gradient
            for i in range(height//2):
                blue_val = max(0, 235 - i)  # Ensure non-negative
                rgb_image[i, :] = [135, 180, blue_val]
            # Grass/dirt ground with variation
            for i in range(height//2, height):
                for j in range(width):
                    green_var = np.random.randint(-20, 20)
                    rgb_image[i, j] = [80, 120+green_var, 60]
            # Add trees (circles/triangles)
            for _ in range(5):
                x = np.random.randint(50, width-50)
                y = np.random.randint(height//3, 2*height//3)
                # Tree trunk
                cv2.rectangle(rgb_image, (x-5, y), (x+5, y+40), (101, 67, 33), -1)
                # Tree top
                cv2.circle(rgb_image, (x, y-20), 30, (34, 139, 34), -1)
                
        return rgb_image
    
    def generate_depth_image(self, points):
        """Generate depth image from point cloud."""
        height, width = 480, 640
        depth_image = np.ones((height, width), dtype=np.float32) * 10.0
        
        # Project points to image plane (simple projection)
        # This is a simplified version - real projection would use camera intrinsics
        fx, fy = 500, 500  # Focal length
        cx, cy = width/2, height/2  # Principal point
        
        for pt in points:
            if pt[2] > 0:  # Only project points in front
                u = int(fx * pt[0] / pt[2] + cx)
                v = int(fy * pt[1] / pt[2] + cy)
                if 0 <= u < width and 0 <= v < height:
                    depth_image[v, u] = min(depth_image[v, u], pt[2])
        
        return depth_image
    
    def visualize_detection(self, points, ground_mask, rgb_image, model_name, scenario_name):
        """Create visualization of ground detection results."""
        fig = plt.figure(figsize=(15, 5))
        
        # 1. Original RGB image
        plt.subplot(1, 3, 1)
        plt.imshow(rgb_image)
        plt.title(f"RGB Input\n{scenario_name}")
        plt.axis('off')
        
        # 2. 3D point cloud with ground detection
        ax = fig.add_subplot(132, projection='3d')
        
        ground_pts = points[ground_mask]
        object_pts = points[~ground_mask]
        
        # Downsample for visualization
        n_viz = min(1000, len(ground_pts))
        if len(ground_pts) > 0:
            idx = np.random.choice(len(ground_pts), n_viz, replace=False)
            ax.scatter(ground_pts[idx, 0], ground_pts[idx, 1], ground_pts[idx, 2], 
                      c='green', s=1, alpha=0.5, label='Ground')
        
        n_viz = min(1000, len(object_pts))
        if len(object_pts) > 0:
            idx = np.random.choice(len(object_pts), n_viz, replace=False)
            ax.scatter(object_pts[idx, 0], object_pts[idx, 1], object_pts[idx, 2], 
                      c='red', s=1, alpha=0.5, label='Objects')
        
        ax.set_xlabel('X')
        ax.set_ylabel('Y')
        ax.set_zlabel('Z')
        ax.set_title(f"{model_name} Detection")
        ax.legend()
        
        # 3. Top-down view (bird's eye)
        plt.subplot(1, 3, 3)
        if len(ground_pts) > 0:
            plt.scatter(ground_pts[:, 0], ground_pts[:, 1], c='green', s=1, alpha=0.3, label='Ground')
        if len(object_pts) > 0:
            plt.scatter(object_pts[:, 0], object_pts[:, 1], c='red', s=1, alpha=0.5, label='Objects')
        plt.xlabel('X (m)')
        plt.ylabel('Y (m)')
        plt.title('Top-Down View')
        plt.legend()
        plt.axis('equal')
        plt.grid(True, alpha=0.3)
        
        plt.tight_layout()
        
        # Save figure
        filename = self.session_dir / f"{scenario_name}_{model_name}.png"
        plt.savefig(filename, dpi=100, bbox_inches='tight')
        plt.close()
        
        return str(filename)
    
    def run_visual_benchmark(self):
        """Run benchmark with visualization."""
        print("="*70)
        print("VISUAL BENCHMARK: Semantic Segmentation vs RANSAC")
        print("="*70)
        
        scenarios = [
            "simple_flat",
            "complex_indoor", 
            "outdoor_terrain"
        ]
        
        models = ['original_ransac', 'midas', 'segformer', 'detectron2']
        
        # Initialize mapper
        mapper = GTrackMapper()
        mapper.params['debug_info'] = True
        
        results = {}
        
        for scenario in scenarios:
            print(f"\n{'='*50}")
            print(f"Scenario: {scenario}")
            print('='*50)
            
            # Generate scene
            points, rgb_image, depth_image, n_ground, desc = self.generate_realistic_scene(scenario)
            print(f"Description: {desc}")
            print(f"Points: {len(points)} (Ground: {n_ground}, Objects: {len(points)-n_ground})")
            
            # Save input images
            cv2.imwrite(str(self.session_dir / f"{scenario}_input_rgb.png"), 
                       cv2.cvtColor(rgb_image, cv2.COLOR_RGB2BGR))
            
            # Normalize and save depth
            depth_viz = (depth_image - depth_image.min()) / (depth_image.max() - depth_image.min())
            depth_viz = (depth_viz * 255).astype(np.uint8)
            cv2.imwrite(str(self.session_dir / f"{scenario}_input_depth.png"), depth_viz)
            
            mapper.set_rgb_image(rgb_image)
            mapper.set_depth_image(depth_image)
            
            scenario_results = {}
            
            for model_name in models:
                print(f"\nTesting {model_name}...")
                
                # Configure model
                if model_name == 'original_ransac':
                    mapper.set_detection_method(use_model=False)
                else:
                    mapper.set_detection_method(use_model=True, model_name=model_name)
                
                # Run detection
                start_time = time.time()
                success = mapper.apply_pointcloud(points.copy())
                inference_time = time.time() - start_time
                
                if success and 'ground_mask' in mapper.debug_info:
                    ground_mask = mapper.debug_info['ground_mask']
                    
                    # Calculate metrics
                    n_detected = np.sum(ground_mask)
                    accuracy = 1.0 - abs(n_detected - n_ground) / len(points)
                    
                    # Visualize
                    viz_file = self.visualize_detection(
                        mapper.debug_info['valid_pts'],
                        ground_mask,
                        rgb_image,
                        model_name,
                        scenario
                    )
                    
                    scenario_results[model_name] = {
                        'time_ms': inference_time * 1000,
                        'accuracy': accuracy,
                        'detected_ground': int(n_detected),
                        'expected_ground': int(n_ground),
                        'visualization': viz_file
                    }
                    
                    print(f"  Time: {inference_time*1000:.2f} ms")
                    print(f"  Accuracy: {accuracy:.2%}")
                    print(f"  Detected/Expected: {n_detected}/{n_ground}")
                    print(f"  Saved: {viz_file}")
                else:
                    print(f"  Failed!")
                    scenario_results[model_name] = {'error': 'Detection failed'}
            
            results[scenario] = scenario_results
        
        # Save results JSON
        results_file = self.session_dir / "benchmark_results.json"
        with open(results_file, 'w') as f:
            json.dump(results, f, indent=2)
        
        print(f"\n{'='*70}")
        print(f"Results saved to: {self.session_dir}")
        print(f"{'='*70}")
        
        self.generate_summary_report(results)
        
        return results
    
    def generate_summary_report(self, results):
        """Generate markdown summary report."""
        report_file = self.session_dir / "VISUAL_REPORT.md"
        
        with open(report_file, 'w') as f:
            f.write("# Visual Benchmark Report\n\n")
            f.write(f"Generated: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}\n\n")
            
            f.write("## Scenario Descriptions\n\n")
            f.write("### Simple Flat\n")
            f.write("- **Real-world analog**: Parking lot, open field\n")
            f.write("- **Characteristics**: Mostly flat ground with 2 vehicle-sized obstacles\n")
            f.write("- **Challenge**: Minimal - clear ground/object separation\n\n")
            
            f.write("### Complex Indoor\n")
            f.write("- **Real-world analog**: Cluttered room, warehouse\n")
            f.write("- **Characteristics**: Floor with 10 furniture pieces of varying heights\n")
            f.write("- **Challenge**: Multiple objects, occlusions, varying object sizes\n\n")
            
            f.write("### Outdoor Terrain\n")
            f.write("- **Real-world analog**: Park, forest, rough terrain\n")
            f.write("- **Characteristics**: Uneven/sloped ground with trees, rocks, bushes\n")
            f.write("- **Challenge**: Non-flat ground, natural objects, terrain variation\n\n")
            
            f.write("## Performance Summary\n\n")
            
            # Create comparison table
            f.write("| Scenario | Model | Time (ms) | Accuracy | Ground Points |\n")
            f.write("|----------|-------|-----------|----------|---------------|\n")
            
            for scenario, scenario_results in results.items():
                for model, metrics in scenario_results.items():
                    if 'error' not in metrics:
                        f.write(f"| {scenario} | {model} | "
                               f"{metrics['time_ms']:.2f} | "
                               f"{metrics['accuracy']:.2%} | "
                               f"{metrics['detected_ground']}/{metrics['expected_ground']} |\n")
            
            f.write("\n## Visualizations\n\n")
            f.write("See the generated PNG files in this directory for visual comparisons.\n")
            f.write("Each image shows:\n")
            f.write("- Left: Original RGB input\n")
            f.write("- Middle: 3D point cloud with detected ground (green) vs objects (red)\n")
            f.write("- Right: Top-down view of the detection\n")
        
        print(f"Report saved to: {report_file}")


def main():
    benchmark = VisualBenchmark()
    results = benchmark.run_visual_benchmark()


if __name__ == "__main__":
    main()