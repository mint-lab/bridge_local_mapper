import numpy as np
import open3d as o3d
import matplotlib.pyplot as plt


class O3DLocalMapper:
    """RGB-D obstacle detector using Open3D plane detection"""

    def __init__(self, map_x_width=10, map_y_width=10, map_cellsize=0.1) -> None:
        """Initialize the local mapper"""
        self.map_nx = int(map_x_width // map_cellsize)
        self.map_ny = int(map_y_width // map_cellsize)
        self.map_cx = self.map_nx // 2 - 1
        self.map_cy = self.map_ny // 2 - 1
        self.map_cellsize = map_cellsize
        self.map_data = {
            'n_hits'        : np.zeros((self.map_ny, self.map_nx),    dtype=np.uint32),
            'elevation'     : np.zeros((self.map_ny, self.map_nx),    dtype=np.float32),
            'obstacle'      : np.zeros((self.map_ny, self.map_nx),    dtype=np.uint8),
            'ground_rgb'    : np.zeros((self.map_ny, self.map_nx, 3), dtype=np.float32),
        }

        self.debug_info = {}
        self.params = {
            'robot2sensor_T'        : [[0, -1, 0, 0], [0, 0, -1, 0], [1, 0, 0, 0], [0, 0, 0, 1]],
            'pcd_sampling_step'     : 1,
            'ground_correct'        : True,
            'ground_mapping'        : True,
            'debug_info'            : True,
            'ransac_threshold'      : 0.05,     # Unit: [m]
            'ransac_num_samples'    : 3,
            'ransac_num_iters'      : 1000,
            'filter_max_depth'      : 10,       # Unit: [m]
            'filter_height_range'   : (-1, 1),  # Unit: [m]
        }
        self.apply_params()

    def apply_params(self):
        """Apply the parameters to the member variables"""
        self.sensor2robot_T = np.linalg.inv(self.params['robot2sensor_T'])

    def set_params(self, params: dict):
        """Set the parameters"""
        self.params.update(params)
        self.apply_params()

    def detect_ground(self, pcd: o3d.geometry.PointCloud) -> tuple:
        """Detect the ground plane"""
        ground_plane, inlier_index = pcd.segment_plane(distance_threshold=self.params['ransac_threshold'],
                                                       ransac_n=self.params['ransac_num_samples'], num_iterations=self.params['ransac_num_iters'])
        return ground_plane, inlier_index

    @staticmethod
    def get_ground_transform(ground_plane: np.ndarray) -> np.ndarray:
        """Get the transformation matrix of the ground plane to make the XY reference plane"""
        ground_normal = ground_plane[:3]
        ground_origin = np.array([0, 0, -ground_plane[-1] / np.linalg.norm(ground_normal)])
        ground_z = ground_normal
        ground_x = np.array([1, 0, 0])
        ground_y = np.cross(ground_z, ground_x)
        ground_R = np.vstack((ground_x, ground_y, ground_z)).T
        ground_T = np.eye(4)
        ground_T[:3, :3] = ground_R
        ground_T[:3, -1] = ground_origin
        return np.linalg.inv(ground_T)

    def conv_rc2xy(self, row, col):
        """Convert the array index (row and column) to the local position (x and y)"""
        x = (col - self.map_cx) * self.map_cellsize
        y = (row - self.map_cy) * self.map_cellsize
        return x, y

    def conv_xy2rc(self, x, y, z=0):
        """Convert the local position (x and y) to the array index (row and column)"""
        col = int(x / self.map_cellsize + self.map_cx + 0.5) # Rouding (nearest neighbor)
        row = int(y / self.map_cellsize + self.map_cy + 0.5) # Rouding (nearest neighbor)
        return row, col

    def conv_xy2rc_array(self, xs, ys):
        """Convert the local positions to the array indices"""
        cols = (xs / self.map_cellsize + self.map_cx + 0.5).astype(np.int32) # Rouding (nearest neighbor)
        rows = (ys / self.map_cellsize + self.map_cy + 0.5).astype(np.int32) # Rouding (nearest neighbor)
        return rows, cols

    def apply_pointcloud(self, pcd: o3d.geometry.PointCloud) -> bool:
        """Apply the detector to the given point cloud"""

        # Filter the point cloud with the maximum depth
        sample_pcd = o3d.geometry.PointCloud()
        if self.params['pcd_sampling_step'] > 1:
            # Sample points every `pcd_sampling_step`
            sample_index = range(0, len(pcd.points), self.params['pcd_sampling_step'])
            sample_pcd = pcd.select_by_index(sample_index)
        else:
            sample_pcd.points = pcd.points
            sample_pcd.colors = pcd.colors
        valid_mask = np.asarray(sample_pcd.points)[:, 2] < self.params['filter_max_depth']
        valid_pts = np.asarray(sample_pcd.points)[valid_mask, :]
        valid_color = np.asarray(sample_pcd.colors)[valid_mask, :]

        # Compensate the sensor pose
        valid_pts = valid_pts @ self.sensor2robot_T[:3, :3].T + self.sensor2robot_T[:3, -1]
        valid_pcd = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(valid_pts))
        valid_pcd.colors = o3d.utility.Vector3dVector(valid_color)

        # Detect the ground plane
        ground_plane, ground_index = self.detect_ground(valid_pcd)
        if ground_plane is None:
            return False
        if ground_plane[2] < 0:
            # Make the normal vector is pointing upwards
            ground_plane = -ground_plane
        ground_mask = np.zeros(len(valid_pts), dtype=bool)
        ground_mask[ground_index] = True

        # Compensate the ground plane to make it as the XY reference plane
        if self.params['ground_correct']:
            ground_T = self.get_ground_transform(ground_plane)
            valid_pts = valid_pts @ ground_T[:3, :3].T + ground_T[:3, -1]
            valid_pcd.points = o3d.utility.Vector3dVector(valid_pts)

        # Filter the point cloud with the height range and map range
        range_mask = (valid_pts[:, -1] >= self.params['filter_height_range'][0]) & (valid_pts[:, -1] <= self.params['filter_height_range'][1])
        range_pts = valid_pts[range_mask, :]
        range_color = valid_color[range_mask, :]
        range_rows, range_cols = self.conv_xy2rc_array(range_pts[:, 0], range_pts[:, 1])
        map_mask = (range_rows >= 0) & (range_rows < self.map_ny) & (range_cols >= 0) & (range_cols < self.map_nx)

        # Update the map data
        self.map_data['n_hits'].fill(0)
        ground_map_mask = ground_mask[range_mask] & map_mask
        if self.params['ground_mapping']:
            for r, c, pt, clr in zip(range_rows[ground_map_mask], range_cols[ground_map_mask], range_pts[ground_map_mask, :], range_color[ground_map_mask, :]):
                self.map_data['n_hits'][r, c] += 1
                self.map_data['elevation'][r, c] = min(pt[-1], self.map_data['elevation'][r, c])
                self.map_data['ground_rgb'][r, c] = clr

        self.map_data['obstacle'].fill(0)
        object_map_mask = ~ground_mask[range_mask] & map_mask
        for r, c, pt in zip(range_rows[object_map_mask], range_cols[object_map_mask], range_pts[object_map_mask, :]):
            self.map_data['n_hits'][r, c] += 1
            self.map_data['elevation'][r, c] = max(pt[-1], self.map_data['elevation'][r, c])
            self.map_data['obstacle'][r, c] = 1

        # Keep the debug information
        if self.params['debug_info']:
            self.debug_info['valid_pcd'] = valid_pcd
            self.debug_info['ground_plane'] = ground_plane
            self.debug_info['ground_index'] = ground_index
            range_pcd = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(range_pts))
            range_pcd.colors = o3d.utility.Vector3dVector(range_color)
            self.debug_info['range_pcd'] = range_pcd
            ground_pcd = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(range_pts[ground_map_mask, :]))
            ground_pcd.colors = o3d.utility.Vector3dVector(range_color[ground_map_mask, :])
            self.debug_info['ground_pcd'] = ground_pcd
            object_pcd = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(range_pts[object_map_mask, :]))
            object_pcd.colors = o3d.utility.Vector3dVector(range_color[object_map_mask, :])
            self.debug_info['object_pcd'] = object_pcd

        return True

    def imshow_map_data(self, data, title=None, cmap='jet', n_xticks=5, n_yticks=5):
        """Plot the map data using Matplotlib PyPlot"""
        plt.figure()
        if title:
            plt.title(title)
        if data.ndim == 2:
            plt.imshow(data, cmap=cmap)
        else:
            plt.imshow(data)
        plt.gca().invert_yaxis()
        plt.xticks(np.linspace(0, self.map_nx-1, n_xticks), np.linspace(-self.map_nx//2, self.map_nx//2+1, n_xticks))
        plt.yticks(np.linspace(0, self.map_ny-1, n_yticks), np.linspace(-self.map_ny//2, self.map_ny//2+1, n_yticks))
        plt.xlabel('X [m]')
        plt.ylabel('Y [m]')
        plt.colorbar()

    @staticmethod
    def paint_pointcloud(pcd: o3d.geometry.PointCloud, color: tuple, alpha: float=1) -> o3d.geometry.PointCloud:
        """Paint the point cloud with the given color and alpha value"""
        pcd_colors = alpha * np.asarray(color) + (1-alpha) * np.asarray(pcd.colors)
        pcd.colors = o3d.utility.Vector3dVector(pcd_colors)
        return pcd


def generate_pointcloud(added_params: dict={}, show_o3d=True):
    """Generate a point cloud with a ground plane and a cylinder object"""
    params = {
        'ground_x_range'    : (0, 5, 0.1),
        'ground_y_range'    : (-5, 5, 0.1),
        'cyliner_radius'    : 0.5,
        'cyliner_height'    : 2.0,
        'cyliner_position'  : [3, 2, 1],
        'cyliner_n_pts'     : 1000,
        'robot2sensor_T'    : np.array([[0, -1,  0, 0],
                                        [0,  0, -1, 1],
                                        [1,  0,  0, 0],
                                        [0,  0,  0, 1]]),
    }
    params.update(added_params)

    # Generate the ground plane
    ground_x, ground_y = np.meshgrid(np.arange(*params['ground_x_range']), np.arange(*params['ground_y_range']))
    ground_z = np.zeros_like(ground_x)
    ground_pts = np.vstack((ground_x.flatten(), ground_y.flatten(), ground_z.flatten())).T
    ground_pcd = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(ground_pts))
    ground_pcd.paint_uniform_color([0, 1, 0])

    # Generate an object
    object_mesh = o3d.geometry.TriangleMesh.create_cylinder(radius=params['cyliner_radius'], height=params['cyliner_height'])
    object_mesh.paint_uniform_color([1, 0, 0])
    object_mesh.translate(params['cyliner_position'])
    object_pcd = object_mesh.sample_points_uniformly(number_of_points=params['cyliner_n_pts'])

    # Apply senosr pose
    all_pcd = ground_pcd + object_pcd
    all_pts = np.asarray(all_pcd.points) @ params['robot2sensor_T'][:3, :3].T + params['robot2sensor_T'][:3, -1]
    all_pcd.points = o3d.utility.Vector3dVector(all_pts)

    if show_o3d:
        # Visualize the point clouds
        geometries = [
            {'name': 'ground pcd',  'geometry': ground_pcd,  'is_visible': False},
            {'name': 'object mesh', 'geometry': object_mesh, 'is_visible': False},
            {'name': 'object pcd',  'geometry': object_pcd,  'is_visible': False},
            {'name': 'point cloud', 'geometry': all_pcd},
        ]
        o3d.visualization.draw(geometries, show_skybox=False, show_ui=True)

    return all_pcd


def test_pointcloud(mapper: O3DLocalMapper, pcd: o3d.geometry.PointCloud, added_params: dict={}, show_map=True, show_debug_info=True):
    """Test the given local mapper with the given point cloud"""
    import time

    # Update the given parameters
    params = {
        'viz_color_alpha'   : 0.5,
        'viz_color_object'  : (1, 0, 0), # Red
        'viz_color_ground'  : (0, 1, 0), # Green
    }
    params.update(added_params)

    # Detect obstacles
    time_start = time.time()
    mapper.apply_pointcloud(pcd)
    time_elapse = time.time() - time_start
    print(f'* Time elapse: {time_elapse * 1000} [msec]')

    # Visualize the map data
    if show_map:
        mapper.imshow_map_data(mapper.map_data['elevation'],  'Elavation Map')
        mapper.imshow_map_data(mapper.map_data['n_hits'],     'The Number of Hits')
        mapper.imshow_map_data(mapper.map_data['obstacle'],   'Obstacle Map')
        mapper.imshow_map_data(mapper.map_data['ground_rgb'], 'Ground RGB Map')
        plt.show()

    # Visualize point clouds in the debug information
    if show_debug_info:
        print(f'* The number of valid points: {len(mapper.debug_info["valid_pcd"].points)} / {len(pcd.points)}')
        valid_pcd = mapper.debug_info['valid_pcd']
        object_pcd = mapper.debug_info['object_pcd']
        ground_pcd = mapper.debug_info['ground_pcd']
        mapper.paint_pointcloud(object_pcd, params['viz_color_object'], params['viz_color_alpha'])
        mapper.paint_pointcloud(ground_pcd, params['viz_color_ground'], params['viz_color_alpha'])

        geometries = [{'name': 'input',    'geometry': pcd,         'is_visible': False},
                      {'name': 'valid',    'geometry': valid_pcd,   'is_visible': False},
                      {'name': 'object',   'geometry': object_pcd},
                      {'name': 'ground',   'geometry': ground_pcd},
        ]
        o3d.visualization.draw(geometries, show_skybox=False, show_ui=True)


if __name__ == '__main__':
    # Read a point cloud from a file
    # pcd_file = '../data/HYU_Yang/zed_17-01-03.ply'
    # pcd_file = '../data/HYU_Yang/zed_17-21-38.ply'
    # pcd_file = '../data/HYU_Yang/zed_17-21-38_336.ply'
    # pcd_file = '../data/HYU_Yang/zed_17-21-38_460.ply'
    pcd_file = '../data/HYU_Yang/zed_17-21-38_476.ply'
    # pcd_file = '../data/HYU_Yang/zed_17-21-38_478.ply'
    # pcd_file = '../data/HYU_Yang/zed_17-21-38_565.ply'
    pcd = o3d.io.read_point_cloud(pcd_file)

    # Generate a point cloud synthetically
    # pcd = generate_pointcloud(show_o3d=False)

    # Test the local mapper
    mapper = O3DLocalMapper()
    mapper.params['pcd_sampling_step'] = 1
    mapper.params['ground_mapping'] = False
    mapper.params['debug_info'] = False
    test_pointcloud(mapper, pcd, show_map=True, show_debug_info=mapper.params['debug_info'])