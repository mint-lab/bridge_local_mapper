
import numpy as np
import cv2 as cv
import cv2
import open3d as o3d
import matplotlib.pyplot as plt
from pathlib import Path
import yaml
from ai_detectors import build as build_detector
_cfg_file = Path(__file__).parent / "config" / "detector.yaml"
DETECTOR  = build_detector(**yaml.safe_load(open(_cfg_file)))
try:
    from gtrack_mapper import GTrackMapper, generate_pointcloud, test_pointcloud
except ImportError:
    from bridge_local_planner.gtrack_mapper import GTrackMapper, generate_pointcloud, test_pointcloud

class GMSACMapper(GTrackMapper):
    """Local mappper with ground plane constraints"""

    def __init__(self, map_x_length=10., map_y_length=10., map_cellsize=0.1) -> None:
        """Initialize the local mapper."""
        super().__init__(map_x_length, map_y_length, map_cellsize)

        self.params['ransac_num_iters'] = 1000
        self.params['ransac_num_samples'] = 3
        self.params['ransac_threshold'] = 0.05 # Unit: [m]
        self.params['ransac_min_iters'] = 10
        self.params['ransac_confidence'] = 0.99
        self.params['ransac_refinement'] = True
        self.params['plane_norm_threshold'] = 1e-6
        self.params['plane_z_threshold'] = 0.5
        self.params['plane_max_height'] = 1.5 # Unit: [m]

    def detect_ground(self, pts: np.array) -> tuple:
        """Detect the ground plane with ground plane constraints."""
        best_plane, best_mask, best_loss = None, None, np.inf
        ransac_num_iters = self.params['ransac_num_iters']
        iter = 0
        while iter < ransac_num_iters:
            # Generate a random plane
            iter += 1
            sample_index = np.random.choice(len(pts), self.params['ransac_num_samples'], replace=False)
            sample_pts = pts[sample_index, :]
            plane = np.cross(sample_pts[1] - sample_pts[0], sample_pts[2] - sample_pts[0])
            if np.linalg.norm(plane) < self.params['plane_norm_threshold']:
                continue
            plane /= np.linalg.norm(plane)
            if -self.params['plane_z_threshold'] < plane[2] < self.params['plane_z_threshold']:
                continue
            if plane[2] < 0:
                plane = -plane
            plane = np.hstack((plane, -plane.dot(sample_pts[0])))
            if plane[3] > self.params['plane_max_height'] or plane[3] < -self.params['plane_max_height']:
                continue

            # Evaluate the plane
            dist = pts @ plane[:3] + plane[-1]
            mask = np.abs(dist) < self.params['ransac_threshold']
            loss = np.sum(dist[mask]**2) + (len(pts) - np.sum(mask)) * self.params['ransac_threshold']**2
            if loss < best_loss:
                best_plane = plane
                best_mask = mask
                best_loss = loss
                inlier_ratio = np.sum(best_mask) / len(pts)
                new_num_iters = np.log(1 - self.params['ransac_confidence']) / np.log(1 - inlier_ratio ** self.params['ransac_num_samples'])
                ransac_num_iters = max(min(new_num_iters, self.params['ransac_num_iters']), self.params['ransac_min_iters'])

        if self.params['ransac_refinement']:
            # Refine the plane using all inliers
            best_pts = pts[best_mask, :]
            if len(best_pts) > 3:
                best_plane = self.find_plane(best_pts)
                if best_plane[2] < 0:
                    best_plane = -best_plane

        if self.params['debug_info']:
            self.debug_info['ransac_num_iters'] = ransac_num_iters
        return best_plane, best_mask

    def apply_pointcloud(self, pts: np.array) -> bool:               # <<< NEW
        """Copy of parent method + optional AI pre-filter."""        # <<< NEW

        # ---- (a) sample & depth-filter  ---------------------------  # <<< NEW
        step = self.params.get('pts_sampling_step', 1)               # <<< NEW
        sample_idx = range(0, len(pts), step)                        # <<< NEW
        sample_pts = pts[sample_idx, :] if step > 1 else pts         # <<< NEW

        depth_max = self.params.get('pts_max_depth', np.inf)         # <<< NEW
        valid_mask = sample_pts[:, 2] < depth_max                    # <<< NEW
        valid_pts  = sample_pts[valid_mask, :]                       # <<< NEW
        if len(valid_pts) < self.params.get('pts_min_pts', 30):      # <<< NEW
            return False                                             # <<< NEW

        # transform to robot frame                                    # <<< NEW
        valid_pts = valid_pts @ self.sensor2robot_T[:3,:3].T + self.sensor2robot_T[:3,-1]  # <<< NEW

        # ---- (b) OPTIONAL call to DINO / SAM / YOLO ---------------  # <<< NEW
        if hasattr(self, "current_rgb_frame") and self.current_rgb_frame is not None:      # <<< NEW
            det_out = DETECTOR.detect(self.current_rgb_frame)                             # <<< NEW
            boxes   = det_out["boxes"]                                                    # <<< NEW
            if boxes.shape[0] == 0:   # model says “no ground”                            # <<< NEW
                return False
            vis_img=self.current_rgb_frame.copy()                                                             # <<< NEW
            for (x1,y1,x2,y2) in boxes.astype(int):
                cv2.rectangle(vis_img,(x1,y1),(x2,y2),color=(0,0,255),thickness=2)
            plt.figure(figsize=(10,6))
            plt.imshow(cv2.cvtColor(vis_img,cv2.COLOR_BGR2RGB))
            plt.title("GroundingDINO  Segmentation Output")
            plt.axis("off")
            plt.show()
            # ✅ INSERT HERE: filter valid_pts using detection mask
            from bridge_local_planner.utils import filter_pts_by_2d_boxes

            # --- Set camera intrinsics (replace with real values!)
            fx, fy = 600, 600
            cx, cy = 320, 240
            K = np.array([[fx, 0, cx],
                          [0, fy, cy],
                          [0,  0,  1]])

            image_shape = self.current_rgb_frame.shape  # (H, W, 3)
            mask = filter_pts_by_2d_boxes(valid_pts, K, boxes, image_shape)
            valid_pts = valid_pts[mask]
            
            # TODO: project frustums & crop valid_pts if you want even faster RANSAC      # <<< NEW
        # ------------------------------------------------------------------------------  # <<< NEW

        # ---- (c) run your existing RANSAC plane fit ---------------  # <<< NEW
        ground_plane, ground_mask = self.detect_ground(valid_pts)     # <<< NEW
        if ground_plane is None:                                      # <<< NEW
            return False                                              # <<< NEW

        # ---- (d) reuse parent’s map-update logic ------------------  # <<< NEW
        # You can copy GTrackMapper.apply_pointcloud()’s post-RANSAC   # <<< NEW
        # section here, or simply call super() if it accepts override  # <<< NEW
        return super().apply_pointcloud(pts)                          # <<< NEW
if __name__ == '__main__':
    
    # ------------------------------------------------------------------ #
    # 1️⃣  CLI flag to pick the .ply file                                #
    # ------------------------------------------------------------------ #
    import argparse, sys
    parser = argparse.ArgumentParser(
        description="Run the GMSAC local mapper on a point-cloud file"
    )
    parser.add_argument(
        "--cloud",
        required=True,
        help="Path to a .ply point cloud (absolute or relative to repo root)",
    )
    args = parser.parse_args()

    # ------------------------------------------------------------------ #
    # 2️⃣  Resolve path no matter where you launch                        #
    # ------------------------------------------------------------------ #
    BASE_DIR  = Path(__file__).resolve().parents[1]   # repo root
    cloudpath = Path(args.cloud).expanduser()
    if not cloudpath.is_absolute():
        cloudpath = BASE_DIR / cloudpath
    cloudpath = cloudpath.resolve()
    if not cloudpath.exists():
        print(f"[ERROR] Point cloud not found: {cloudpath}", file=sys.stderr)
        sys.exit(1)

    # ------------------------------------------------------------------ #
    # 3️⃣  Load cloud and run mapper                                      #
    # ------------------------------------------------------------------ #
    pcd = o3d.io.read_point_cloud(str(cloudpath))
    pts = np.asarray(pcd.points)

    mapper = GMSACMapper()
    mapper.set_params({
        "pts_sampling_step": 4,
        "debug_info": False,
    })
    rgb_path = cloudpath.with_suffix(".png")  
    if rgb_path.exists():
        mapper.current_rgb_frame = cv.imread(str(rgb_path))
    else:
        print(f"[WARN] RGB image not found at: {rgb_path}")
    test_pointcloud(
        mapper,
        pts,
        show_map=True,
        show_debug_info=mapper.params["debug_info"],
    )

    
    # Read a point cloud from a file.
    # pcd_file = '../data/231031_HYU_Yang/zed_17-01-03.ply'
    # pcd_file = '../data/231031_HYU_Yang/zed_17-21-38.ply'
    # pcd_file = '../data/231031_HYU_Yang/zed_17-21-38_336.ply'
    # pcd_file = '../data/231031_HYU_Yang/zed_17-21-38_460.ply'
    #pcd_file = '../data/231031_HYU_Yang/zed_17-21-38_476.ply'
    # pcd_file = '../data/231031_HYU_Yang/zed_17-21-38_478.ply'
    # pcd_file = '../data/231031_HYU_Yang/zed_17-21-38_565.ply'
    #pcd = o3d.io.read_point_cloud(str(cloudpath))
    #pts = np.asarray(pcd.points)

    # Test the local mapper.
    """
    mapper = GMSACMapper()
    mapper.set_params({
        'pts_sampling_step' : 4,
        'debug_info'        : False,
    })
    test_pointcloud(mapper, pts, show_map=True, show_debug_info=mapper.params['debug_info'])
    """    
    
    # Read a point cloud from a file.
    # pcd_file = '../data/231031_HYU_Yang/zed_17-01-03.ply'
    # pcd_file = '../data/231031_HYU_Yang/zed_17-21-38.ply'
    # pcd_file = '../data/231031_HYU_Yang/zed_17-21-38_336.ply'
    # pcd_file = '../data/231031_HYU_Yang/zed_17-21-38_460.ply'
    #pcd_file = '../data/231031_HYU_Yang/zed_17-21-38_476.ply'
    # pcd_file = '../data/231031_HYU_Yang/zed_17-21-38_478.ply'
    # pcd_file = '../data/231031_HYU_Yang/zed_17-21-38_565.ply'
    #pcd = o3d.io.read_point_cloud(str(cloudpath))
    #pts = np.asarray(pcd.points)

    # Test the local mapper.
    """
    mapper = GMSACMapper()
    mapper.set_params({
        'pts_sampling_step' : 4,
        'debug_info'        : False,
    })
    test_pointcloud(mapper, pts, show_map=True, show_debug_info=mapper.params['debug_info'])
    """