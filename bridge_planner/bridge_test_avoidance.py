import time
import numpy as np
import open3d as o3d
from scipy.interpolate import BSpline

def print_camera_viewpoint(o3dvis):
    Rt = o3dvis.scene.camera.get_view_matrix()
    R, t = Rt[:3,:3], Rt[:3,-1]
    p = -R.T @ t
    print(f'* eye at {p}')

if __name__ == '__main__':
    # Configure the experiments
    map_file = 'data/zed_17-01-03.ply'
    map_file = 'data/zed_17-21-38.ply'
    # map_file = 'data/zed_17-21-38_336.ply'
    # map_file = 'data/zed_17-21-38_460.ply'
    map_file = 'data/zed_17-21-38_476.ply'
    # map_file = 'data/zed_17-21-38_478.ply'
    # map_file = 'data/zed_17-21-38_565.ply'
    map_viewpoint = {
        'eye'   : [-0, -0.4, -1],
        'lookat': [0, 0, 3],
        'up'    : [0, -1, 0]}
    ransac_dist_threshold = 0.05
    filter_dist_range = (-1, 0)
    path_width = 4
    path_n_candidates = 11
    path_length = 5
    path_height = 0.5
    path_smooth_start = 0.5
    path_smooth_end = 2
    path_t_step = 0.02
    path_obstacle_penalty = 1
    viz_color_alpha = 0.5
    viz_ground_color = (0, 1, 0)
    viz_obstacle_color = (1, 0, 0)
    viz_path_best_color = (0, 0, 1)
    viz_path_candidate_color = (0.3, 0.3, 0.3)
    viz_path_collision_color = (1, 0, 0)

    # Load a map
    map_pcd = o3d.io.read_point_cloud(map_file)

    # Detect the ground plane
    ground_plane, inliers = map_pcd.segment_plane(distance_threshold=ransac_dist_threshold, ransac_n=3, num_iterations=1000)
    ground_pcd = map_pcd.select_by_index(inliers)
    ground_color = viz_color_alpha * np.asarray(viz_ground_color) + (1-viz_color_alpha) * np.asarray(map_pcd.colors)[inliers,:]
    ground_pcd.colors = o3d.utility.Vector3dVector(ground_color)

    # Detect obstacles
    start_time = time.time()
    outlier_pcd = map_pcd.select_by_index(inliers, invert=True)
    outlier_pts = np.asarray(outlier_pcd.points)
    distance = ground_plane[:3] @ outlier_pts.T + ground_plane[-1]
    obstacle_mask = np.logical_and((distance > filter_dist_range[0]), (distance < filter_dist_range[1])) # Filter with respect to `distance`
    obstacle_pcd = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(outlier_pts[obstacle_mask,:]))
    obstacle_color = viz_color_alpha * np.asarray(viz_obstacle_color) + (1-viz_color_alpha)* np.asarray(outlier_pcd.colors)[obstacle_mask,:]
    obstacle_pcd.colors = o3d.utility.Vector3dVector(obstacle_color)
    obstacle_voxel = o3d.geometry.VoxelGrid.create_from_point_cloud(obstacle_pcd, voxel_size=0.1)

    # Generate candidate paths
    spline_ts = np.arange(0, 1, path_t_step)
    path_pts, path_cost = [], []
    knots = [0, 0, 0, 0, 1, 1, 1, 1]
    for x in np.linspace(-path_width/2, path_width/2, path_n_candidates):
        ctrl_pts = [(0, path_height, 0), (0, path_height, path_smooth_start), (x, path_height, path_length-path_smooth_end), (x, path_height, path_length)]
        spline = BSpline(knots, ctrl_pts, 3)
        path_pts.append(np.array([spline(t) for t in spline_ts]))
        path_cost.append(abs(x))
    path_line = []
    for pts in path_pts:
        line = o3d.geometry.LineSet(o3d.utility.Vector3dVector(pts), o3d.utility.Vector2iVector(np.array([(idx, idx+1) for idx in range(len(pts)-1)])))
        line.paint_uniform_color(viz_path_candidate_color)
        path_line.append(line)

    # Check collision
    for path_idx, path in enumerate(path_line):
        collision = obstacle_voxel.check_if_included(path.points)
        if any(collision):
            path.paint_uniform_color(viz_path_collision_color)
            path_cost[path_idx] = np.inf

    # Select the best path
    for idx in range(1, len(path_cost)-1):
        if path_cost[idx] == np.inf:
            path_cost[idx-1] += path_obstacle_penalty
            path_cost[idx+1] += path_obstacle_penalty
    path_best = np.argmin(path_cost)
    if path_cost[path_best] is not np.inf:
        path_line[path_best].paint_uniform_color(viz_path_best_color)
    elapse = time.time() - start_time
    print(f'* Time elapse: {elapse * 1000} [msec]')

    # Show the map and planning results
    objects = [{'name': 'map',              'geometry': map_pcd},
               {'name': 'ground plane',     'geometry': ground_pcd},
               {'name': 'obstacles',        'geometry': obstacle_pcd},
               {'name': 'obstacle map',     'geometry': obstacle_voxel, 'is_visible': False}]
    for path_idx, line in enumerate(path_line):
        objects.append({'name': f'candidate path {path_idx}', 'geometry': line})
    actions = [('Print Camera Viewpoint',   print_camera_viewpoint)]
    o3d.visualization.draw(objects, actions=actions, show_skybox=False, show_ui=False, **map_viewpoint)