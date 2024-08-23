import numpy as np
import cv2 as cv
import open3d as o3d
import matplotlib.pyplot as plt

try:
    from gtrack_mapper import GTrackMapper, generate_pointcloud, test_pointcloud
except ImportError:
    from bridge_local_planner.gtrack_mapper import GTrackMapper, generate_pointcloud, test_pointcloud


class O3DMapper(GTrackMapper):
    """RGB-D Local Mappper using Open3D Plane Detection"""

    def __init__(self, map_x_length=10, map_y_length=10, map_cellsize=0.1) -> None:
        """Initialize the local mapper."""
        super().__init__(map_x_length, map_y_length, map_cellsize)
        self.params['ransac_num_iters'] = 1000
        self.params['ransac_num_samples'] = 3
        self.params['ransac_threshold'] = 0.05 # Unit: [m]

    def detect_ground(self, pts: np.array) -> tuple:
        """Detect the ground plane using Open3D."""
        pcd = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(pts))
        ground_plane, inlier_index = pcd.segment_plane(distance_threshold=self.params['ransac_threshold'],
                                                       ransac_n=self.params['ransac_num_samples'], num_iterations=self.params['ransac_num_iters'])
        if ground_plane[2] < 0:
            # Make the normal vector is pointing upwards
            ground_plane = -ground_plane
        ground_mask = np.zeros(len(pts), dtype=bool)
        ground_mask[inlier_index] = True

        return ground_plane, ground_mask


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

    # Generate a point cloud synthetically.
    # pts = generate_pointcloud(show_o3d=False)

    # Test the local mapper.
    mapper = O3DMapper()
    mapper.set_params({
        'pts_sampling_step' : 4,
        'ground_mapping'    : True,
        'debug_info'        : False,
    })
    test_pointcloud(mapper, pts, show_map=True, show_debug_info=mapper.params['debug_info'])