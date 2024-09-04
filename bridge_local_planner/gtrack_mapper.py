import numpy as np
from scipy.spatial.transform import Rotation
import cv2 as cv
import matplotlib.pyplot as plt
import yaml


class GTrackMapper:
    """Local mapper with ground plane constraints, asymmetric MSAC, and plane tracking"""

    def __init__(self, map_x_length=10., map_y_length=10., map_cellsize=0.1) -> None:
        """Initialize the local mapper."""
        self.map_nx = int(map_x_length / map_cellsize)
        self.map_ny = int(map_y_length / map_cellsize)
        self.map_cx = self.map_nx // 2 - 1
        self.map_cy = self.map_ny // 2 - 1
        self.map_cellsize = map_cellsize
        self.map_data = {
            'obstacles' : np.zeros((self.map_ny, self.map_nx), dtype=np.int8),
            'elevation' : np.zeros((self.map_ny, self.map_nx), dtype=np.float32),
            'histogram' : np.zeros((self.map_ny, self.map_nx), dtype=np.uint32),
        }

        self.debug_info = {}
        self.params = {
            'robot2sensor_T'        : [[0, -1, 0, 0], [0, 0, -1, 0], [1, 0, 0, 0], [0, 0, 0, 1]],
            'pts_sampling_step'     : 1,
            'pts_max_depth'         : 10,       # Unit: [m]
            'pts_min_pts'           : 1000,
            'ransac_num_iters'      : 1000,
            'ransac_num_samples'    : 3,
            'ransac_threshold'      : 0.05,     # Unit: [m]
            'ransac_min_iters'      : 10,
            'ransac_confidence'     : 0.99,
            'ransac_above_threshold': 2,        # Unit: [m]
            'ransac_refinement'     : True,
            'plane_norm_threshold'  : 1e-6,
            'plane_z_threshold'     : 0.5,
            'plane_max_height'      : 1.5,      # Unit: [m]
            'ground_correct'        : True,
            'ground_mapping'        : True,
            'filter_height_range'   : (-1, 1),  # Unit: [m]
            'debug_info'            : False,
        }
        self.apply_params()

    def apply_params(self):
        """Apply the parameters to the member variables."""
        self.sensor2robot_T = np.linalg.inv(self.params['robot2sensor_T'])

    def set_params(self, params: dict):
        """Set the parameters."""
        self.params.update(params)
        self.apply_params()

    def load_params_from_yaml(self, yaml_file: str, key: str=None):
        """Load the parameters from a file."""
        with open(yaml_file, 'r') as f:
            params = yaml.safe_load(f)
        if key and key in params:
            self.set_params(params[key])
        else:
            self.set_params(params)
        self.apply_params()

    def save_params_to_yaml(self, yaml_file: str):
        """Save the parameters to a file."""
        with open(yaml_file, 'w') as f:
            yaml.dump(self.params, f)

    def detect_ground(self, pts: np.array) -> tuple:
        """Detect the ground plane."""
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
            mask_below = (-self.params['ransac_threshold'] < dist) & (dist <= 0)
            mask_above = (0 < dist) & (dist < self.params['ransac_above_threshold'])
            loss = np.sum(dist[mask_below]**2) / self.params['ransac_threshold']**2 + np.sum(dist[mask_above]**2) / self.params['ransac_above_threshold']**2 + (len(pts) - np.sum(mask_below) - np.sum(mask_above))
            if loss < best_loss:
                best_plane = plane
                best_mask = np.abs(dist) < self.params['ransac_threshold'] # Find ground points with the tight threshold again
                best_loss = loss
                inlier_ratio = np.sum(best_mask) / len(pts)
                new_num_iters = np.log(1 - self.params['ransac_confidence']) / np.log(1 - inlier_ratio ** self.params['ransac_num_samples'])
                ransac_num_iters = max(min(new_num_iters, self.params['ransac_num_iters']), self.params['ransac_min_iters'])

        if self.params['ransac_refinement'] and best_loss > 0:
            # Refine the plane using all inliers
            best_pts = pts[best_mask, :]
            if len(best_pts) > 3:
                best_plane = self.find_plane(best_pts)
                if best_plane[2] < 0:
                    best_plane = -best_plane

        if self.params['debug_info']:
            self.debug_info['ransac_num_iters'] = ransac_num_iters
        return best_plane, best_mask

    @staticmethod
    def find_plane(pts: np.array) -> np.array:
        """Find the plane using the least squares method."""
        mean = pts.mean(axis=0)
        pts_centered = pts - mean
        _, _, Vh = np.linalg.svd(pts_centered, full_matrices=False)
        normal_vector = Vh[-1, :]
        d = -normal_vector.dot(mean)
        return np.hstack((normal_vector, d))

    @staticmethod
    def get_ground_transform(ground_plane: np.ndarray) -> np.ndarray:
        """Get the transformation matrix of the ground plane to make the XY reference plane."""
        normal_vector = ground_plane[:3]
        z_axis = np.array([0, 0, 1])
        rotation_axis = np.cross(normal_vector, z_axis)
        abs_sin_theta = np.linalg.norm(rotation_axis)
        if abs_sin_theta < 1e-6:
            R = np.eye(3)
        else:
            abs_theta = np.arcsin(np.clip(abs_sin_theta, -1.0, 1.0))
            R = Rotation.from_rotvec(rotation_axis / abs_sin_theta * abs_theta).as_matrix()
        ground_T = np.eye(4)
        ground_T[:3, :3] = R
        ground_T[:3, -1] = R @ (ground_plane[-1] * normal_vector)
        return ground_T

    def conv_rc2xy(self, row, col):
        """Convert the array index (row and column) to the local position (x and y)."""
        x = (col - self.map_cx) * self.map_cellsize
        y = (row - self.map_cy) * self.map_cellsize
        return x, y

    def conv_xy2rc(self, x, y, z=0):
        """Convert the local position (x and y) to the array index (row and column)."""
        col = int(x / self.map_cellsize + self.map_cx + 0.5) # Rouding (nearest neighbor)
        row = int(y / self.map_cellsize + self.map_cy + 0.5) # Rouding (nearest neighbor)
        return row, col

    def conv_xy2rc_array(self, xs, ys):
        """Convert the local positions to the array indices."""
        cols = (xs / self.map_cellsize + self.map_cx + 0.5).astype(np.int32) # Rouding (nearest neighbor)
        rows = (ys / self.map_cellsize + self.map_cy + 0.5).astype(np.int32) # Rouding (nearest neighbor)
        return rows, cols

    def apply_pointcloud(self, pts: np.array) -> bool:
        """Apply the given point cloud."""

        # Sample the point cloud.
        if self.params['pts_sampling_step'] > 1:
            # Sample points every `pts_sampling_step`
            sample_index = range(0, len(pts), self.params['pts_sampling_step'])
            sample_pts = pts[sample_index, :]
        else:
            sample_pts = pts

        # Filter the point cloud with the maximum depth.
        valid_mask = sample_pts[:, 2] < self.params['pts_max_depth']
        valid_pts = sample_pts[valid_mask, :]
        if len(valid_pts) < self.params['pts_min_pts']:
            return False

        # Compensate the sensor pose.
        valid_pts = valid_pts @ self.sensor2robot_T[:3, :3].T + self.sensor2robot_T[:3, -1]

        # Detect the ground plane.
        ground_plane, ground_mask = self.detect_ground(valid_pts)
        if ground_plane is None:
            return False

        # Compensate the ground plane to make it as the XY reference plane.
        if self.params['ground_correct']:
            ground_T = self.get_ground_transform(ground_plane)
            valid_pts = valid_pts @ ground_T[:3, :3].T + ground_T[:3, -1]

        # Filter the point cloud with the height range and map range.
        range_mask = (valid_pts[:, -1] >= self.params['filter_height_range'][0]) & (valid_pts[:, -1] <= self.params['filter_height_range'][1])
        range_pts = valid_pts[range_mask, :]
        range_rows, range_cols = self.conv_xy2rc_array(range_pts[:, 0], range_pts[:, 1])

        # Update the map data.
        self.map_data['elevation'].fill(0)
        self.map_data['histogram'].fill(0)
        map_mask = (range_rows >= 0) & (range_rows < self.map_ny) & (range_cols >= 0) & (range_cols < self.map_nx)
        ground_map_mask = ground_mask[range_mask] & map_mask
        if self.params['ground_mapping']:
            self.map_data['obstacles'].fill(-1)
            for r, c, pt in zip(range_rows[ground_map_mask], range_cols[ground_map_mask], range_pts[ground_map_mask, :]):
                self.map_data['obstacles'][r, c] = 0
                self.map_data['elevation'][r, c] = min(pt[-1], self.map_data['elevation'][r, c])
                self.map_data['histogram'][r, c] += 1
        else:
            self.map_data['obstacles'].fill(0)

        object_map_mask = ~ground_mask[range_mask] & map_mask
        for r, c, pt in zip(range_rows[object_map_mask], range_cols[object_map_mask], range_pts[object_map_mask, :]):
            self.map_data['obstacles'][r, c] = 100
            self.map_data['elevation'][r, c] = max(pt[-1], self.map_data['elevation'][r, c])
            self.map_data['histogram'][r, c] += 1

        # Keep the debug information if necessary.
        if self.params['debug_info']:
            self.debug_info['valid_pts'] = valid_pts
            self.debug_info['ground_plane'] = ground_plane
            self.debug_info['ground_mask'] = ground_mask
            self.debug_info['ground_pts'] = range_pts[ground_map_mask, :]
            self.debug_info['object_pts'] = range_pts[object_map_mask, :]

        return True

    def imshow_map_pyplot(self, data, title=None, cmap='jet', n_xticks=5, n_yticks=5):
        """Plot the map data using Matplotlib PyPlot."""
        plt.figure()
        if title:
            plt.title(title)
        if data.ndim == 2:
            plt.imshow(data, cmap=cmap)
        else:
            plt.imshow(data)
        plt.gca().invert_yaxis()
        plt.xticks(np.linspace(0, self.map_nx-1, n_xticks), np.linspace(-self.map_nx//2, self.map_nx//2, n_xticks))
        plt.yticks(np.linspace(0, self.map_ny-1, n_yticks), np.linspace(-self.map_ny//2, self.map_ny//2, n_yticks))
        plt.xlabel('X [m]')
        plt.ylabel('Y [m]')
        plt.colorbar()

    @staticmethod
    def convert_cv_map(map, alpha=1, beta=0, cvt_gray2bgr=True):
        """Rescale the map for OpenCV."""
        img = (alpha * map + beta).clip(0, 255).astype(np.uint8)
        if cvt_gray2bgr:
            img = cv.cvtColor(img, cv.COLOR_GRAY2BGR)
        return img


def generate_pointcloud(added_params: dict={}, show_o3d=False):
    """Generate a point cloud with a ground plane and a cylinder object."""
    import open3d as o3d

    # Define default parameters and update the given parameters.
    # Note) The ground coordinate system is same with the robot coordinate system.
    params = {
        'plane_x_range'     : (0, 5, 0.1),  # [m]
        'plane_y_range'     : (-5, 5, 0.1), # [m]
        'cyliner_radius'    : 0.5,          # [m]
        'cyliner_height'    : 2.0,          # [m]
        'cyliner_position'  : [3, 2, 0],    # [m]
        'cyliner_n_pts'     : 1000,
        'ground_tilt_angle' : 0,            # [deg] (0: horizontal, -30: uphill 30 degrees)
        'ground2sensor_T'   : np.array([[ 0,  0, 1, 0],
                                        [-1,  0, 0, 0],
                                        [ 0, -1, 0, 1],
                                        [ 0,  0, 0, 1]]),
    }
    params.update(added_params)

    # Generate a plane.
    ground_x, ground_y = np.meshgrid(np.arange(*params['plane_x_range']), np.arange(*params['plane_y_range']))
    ground_z = np.zeros_like(ground_x)
    ground_pts = np.vstack((ground_x.flatten(), ground_y.flatten(), ground_z.flatten())).T
    ground_pcd = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(ground_pts))
    ground_pcd.paint_uniform_color([0, 1, 0])
    all_pcd = ground_pcd

    # Generate an cylinderical object.
    if params['cyliner_radius'] > 0 and params['cyliner_height'] > 0:
        object_mesh = o3d.geometry.TriangleMesh.create_cylinder(radius=params['cyliner_radius'], height=params['cyliner_height'])
        object_mesh.paint_uniform_color([1, 0, 0])
        object_mesh.translate(params['cyliner_position'] + np.array([0, 0, params['cyliner_height']/2]))
        object_pcd = object_mesh.sample_points_uniformly(number_of_points=params['cyliner_n_pts'])
        all_pcd += object_pcd

    # Apply the ground rotation.
    all_pts = np.asarray(all_pcd.points)
    rot_angle = np.deg2rad([0, params['ground_tilt_angle'], 0])
    R = o3d.geometry.get_rotation_matrix_from_zyx(rot_angle)
    all_pts = all_pts @ R.T # Same with `(all_pts.T = R @ all_pts.T).T`

    # Observe points in the senosr coordinate system.
    Rt = np.linalg.inv(params['ground2sensor_T'])
    all_pts = all_pts @ Rt[:3, :3].T + Rt[:3, -1]
    all_pcd.points = o3d.utility.Vector3dVector(all_pts)

    # Visualize the point clouds if necessary.
    if show_o3d:
        geometries = [
            {'name': 'ground pcd',  'geometry': ground_pcd,  'is_visible': False},
            {'name': 'object mesh', 'geometry': object_mesh, 'is_visible': False},
            {'name': 'object pcd',  'geometry': object_pcd,  'is_visible': False},
            {'name': 'point cloud', 'geometry': all_pcd},
        ]
        o3d.visualization.draw(geometries, show_skybox=False, show_ui=True)

    return all_pts


def print_debug_info(debug_info: dict, num_all_pts: int):
    if 'valid_pts' in debug_info and 'ground_mask' in debug_info:
        num_valid_pts = len(debug_info['valid_pts'])
        num_ground_pts = sum(debug_info['ground_mask'])
        print(f'* The number of valid  points: {num_valid_pts} / {num_all_pts} ({num_valid_pts/num_all_pts*100:.0f}%)')
        print(f'* The number of ground points: {num_ground_pts} / {num_valid_pts} ({num_ground_pts/num_valid_pts*100:.0f}%)')
    if 'ground_plane' in debug_info:
        print(f'* Ground plane: {debug_info["ground_plane"]}')
    if 'ransac_num_iters' in debug_info:
        print(f'* The number of RANSAC iterations: {debug_info["ransac_num_iters"]}')


def test_pointcloud(mapper: GTrackMapper, pts: np.array, added_params: dict={}, show_map=True, show_debug_info=True):
    """Test the given local mapper with the given point cloud."""
    import time

    # Define the default parameters and update the given parameters.
    params = {
        'viz_color_object'  : (1, 0, 0), # Red
        'viz_color_ground'  : (0, 1, 0), # Green
    }
    params.update(added_params)

    # Apply the given point cloud (build the local map).
    time_start = time.time()
    success = mapper.apply_pointcloud(pts)
    time_elapse = time.time() - time_start
    print(f'* Computing time: {time_elapse * 1000} [msec] (success: {success})')

    # Show the local map data.
    if show_map:
        mapper.imshow_map_pyplot(mapper.map_data['obstacles'], 'Obstacles')
        mapper.imshow_map_pyplot(mapper.map_data['elevation'], 'Elavation')
        mapper.imshow_map_pyplot(mapper.map_data['histogram'], 'Histogram')
        plt.show()

    # Show the debug information.
    if show_debug_info:
        print_debug_info(mapper.debug_info, len(pts))

        if 'valid_pts' in mapper.debug_info and 'object_pts' in mapper.debug_info and 'ground_pts' in mapper.debug_info:
            import open3d as o3d
            valid_pcd = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(mapper.debug_info['valid_pts']))
            valid_pcd.paint_uniform_color([0.5, 0.5, 0.5])
            object_pcd = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(mapper.debug_info['object_pts']))
            object_pcd.paint_uniform_color(params['viz_color_object'])
            ground_pcd = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(mapper.debug_info['ground_pts']))
            ground_pcd.paint_uniform_color(params['viz_color_ground'])
            geometries = [{'name': 'valid',    'geometry': valid_pcd,   'is_visible': False},
                          {'name': 'object',   'geometry': object_pcd},
                          {'name': 'ground',   'geometry': ground_pcd},
            ]
            o3d.visualization.draw(geometries, show_skybox=False, show_ui=True)


if __name__ == '__main__':
    # Read a point cloud from a file.
    import open3d as o3d
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
    # pts_params = {
    #     'cyliner_height'   : 0.5, # [m]
    #     'ground_tilt_angle': -10, # [deg]
    #     'ground2sensor_T'  : np.array([[ 0,  0, 1, -1],
    #                                    [-1,  0, 0,  0],
    #                                    [ 0, -1, 0,  1],
    #                                    [ 0,  0, 0,  1]])}
    # pts = generate_pointcloud(pts_params, show_o3d=False)

    # Test the local mapper.
    mapper = GTrackMapper()
    mapper.set_params({
        'pts_sampling_step' : 4,
        'ground_mapping'    : True,
        'debug_info'        : False,
    })
    test_pointcloud(mapper, pts, show_map=True, show_debug_info=mapper.params['debug_info'])