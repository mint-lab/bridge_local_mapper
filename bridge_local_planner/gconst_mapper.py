import numpy as np
import cv2 as cv
import open3d as o3d
import matplotlib.pyplot as plt

try:
    from gtrack_mapper import GTrackMapper, generate_pointcloud, test_pointcloud
except ImportError:
    from bridge_local_planner.gtrack_mapper import GTrackMapper, generate_pointcloud, test_pointcloud


class GConstMapper(GTrackMapper):
    """Local mappper with ground plane constraints"""

    def __init__(self, map_x_length=10., map_y_length=10., map_cellsize=0.1) -> None:
        """Initialize the local mapper."""
        super().__init__(map_x_length, map_y_length, map_cellsize)

        self.params['ransac_num_iters'] = 1000
        self.params['ransac_num_samples'] = 3
        self.params['ransac_threshold'] = 0.05 # Unit: [m]
        self.params['ransac_confidence'] = 0.99
        self.params['plane_norm_threshold'] = 1e-6
        self.params['plane_z_threshold'] = 0.5
        self.params['plane_max_height'] = 1.5 # Unit: [m]

    def detect_ground(self, pts: np.array) -> tuple:
        """Detect the ground plane with ground plane constraints."""
        best_plane, best_mask, best_score = None, None, 0
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
            score = np.sum(mask)
            if score > best_score:
                best_plane = plane
                best_mask = mask
                best_score = score
                inlier_ratio = score / len(pts)
                new_num_iters = np.log(1 - self.params['ransac_confidence']) / np.log(1 - inlier_ratio ** self.params['ransac_num_samples'])
                ransac_num_iters = min(new_num_iters, self.params['ransac_num_iters'])

        if self.params['debug_info']:
            self.debug_info['ransac_num_iters'] = ransac_num_iters
        return best_plane, best_mask


if __name__ == '__main__':
    # Read a point cloud from a file.
    # pcd_file = '../data/231031_HYU_Yang/zed_17-01-03.ply'
    # pcd_file = '../data/231031_HYU_Yang/zed_17-21-38.ply'
    # pcd_file = '../data/231031_HYU_Yang/zed_17-21-38_336.ply'
    # pcd_file = '../data/231031_HYU_Yang/zed_17-21-38_460.ply'
    pcd_file = '../data/231031_HYU_Yang/zed_17-21-38_476.ply'
    # pcd_file = '../data/231031_HYU_Yang/zed_17-21-38_478.ply'
    # pcd_file = '../data/231031_HYU_Yang/zed_17-21-38_565.ply'
    pcd = o3d.io.read_point_cloud(pcd_file)
    pts = np.asarray(pcd.points)

    # Test the local mapper.
    mapper = GConstMapper()
    mapper.set_params({
        'pts_sampling_step' : 4,
        'ground_mapping'    : True,
        'debug_info'        : False,
    })
    test_pointcloud(mapper, pts, show_map=True, show_debug_info=mapper.params['debug_info'])