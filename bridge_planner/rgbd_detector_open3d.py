import numpy as np
import open3d as o3d


class RGBDDetectorOpen3D:
    """RGB-D obstacle detector using Open3D plane detection"""

    def __init__(self) -> None:
        """Initialize the detector"""
        self.params = {
            'ransac_threshold'      : 0.05,
            'ransac_num_samples'    : 3,
            'ransac_num_iters'      : 1000,
            'filter_height_range'   : (-1, 0),
        }

    def detect_ground(self, pcd: o3d.geometry.PointCloud) -> tuple:
        """Detect the ground plane"""
        ground_plane, inlier_index = pcd.segment_plane(distance_threshold=self.params['ransac_threshold'],
                                                       ransac_n=self.params['ransac_num_samples'], num_iterations=self.params['ransac_num_iters'])
        return ground_plane, inlier_index

    def detect_obstacles(self, pcd: o3d.geometry.PointCloud) -> tuple:
        """Detect obstacles"""

        # Detect obstacles on the ground plane
        ground_plane, ground_index = self.detect_ground(pcd, self.params)
        ground_pcd = pcd.select_by_index(ground_index)
        others_pcd = pcd.select_by_index(ground_index, invert=True)

        # Filter obstacles by the range of target height from the ground plane
        object_pts = np.asarray(others_pcd.points)
        object_height = ground_plane[:3] @ object_pts.T + ground_plane[-1]
        object_mask = np.logical_and((object_height > self.params['filter_height_range'][0]), (object_height < self.params['filter_height_range'][1]))
        object_pcd = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(object_pts[object_mask,:]))
        object_pcd.colors = o3d.utility.Vector3dVector(np.asarray(others_pcd.colors)[object_mask,:])

        return object_pcd, ground_pcd

    @staticmethod
    def paint_pointcloud(pcd: o3d.geometry.PointCloud, color: tuple, alpha: float=1) -> o3d.geometry.PointCloud:
        """Paint the point cloud with the given color and alpha value"""
        pcd_colors = alpha * np.asarray(color) + (1-alpha) * np.asarray(pcd.colors)
        pcd.colors = o3d.utility.Vector3dVector(pcd_colors)
        return pcd


def print_o3d_camera_position(o3d_viz):
    """Print the camera viewpoint of Open3D visualization GUI"""
    Rt = o3d_viz.scene.camera.get_view_matrix()
    R, t = Rt[:3,:3], Rt[:3,-1]
    p = -R.T @ t
    print(f'* eye at {p}')


def test_pointcloud_file(pcd_file: str, added_params: dict={}):
    import time

    # Update the given parameters
    params = {
        'viz_color_alpha'   : 0.5,
        'viz_color_object'  : (1, 0, 0), # Red
        'viz_color_ground'  : (0, 1, 0), # Green
        'viz_viewpoint': {
            'eye'           : [-0, -0.4, -1],
            'lookat'        : [0, 0, 3],
            'up'            : [0, -1, 0],
        },
    }
    params.update(added_params)

    # Read the point cloud file
    pcd = o3d.io.read_point_cloud(pcd_file)

    # Detect obstacles
    detector = RGBDDetectorOpen3D()
    time_start = time.time()
    object_pcd, ground_pcd = detector.detect_obstacles(pcd)
    time_elapse = time.time() - time_start
    print(f'* Time elapse: {time_elapse * 1000} [msec]')

    # Visualize point clouds
    detector.paint_pointcloud(object_pcd, params['viz_color_object'], params['viz_color_alpha'])
    detector.paint_pointcloud(ground_pcd, params['viz_color_ground'], params['viz_color_alpha'])

    objects = [{'name': 'all',      'geometry': pcd},
               {'name': 'object',   'geometry': object_pcd},
               {'name': 'ground',   'geometry': ground_pcd},
    ]
    actions = [('Print Camera Viewpoint',   print_o3d_camera_position)]
    o3d.visualization.draw(objects, actions=actions, show_skybox=False, show_ui=False, **params['viz_viewpoint'])


if __name__ == '__main__':
    pcd_file = '../data/zed_17-01-03.ply'
    pcd_file = '../data/zed_17-21-38.ply'

    # pcd_file = '../data/zed_17-21-38_336.ply'
    # pcd_file = '../data/zed_17-21-38_460.ply'
    pcd_file = '../data/zed_17-21-38_476.ply'
    # pcd_file = '../data/zed_17-21-38_478.ply'
    # pcd_file = '../data/zed_17-21-38_565.ply'

    test_pointcloud_file(pcd_file)