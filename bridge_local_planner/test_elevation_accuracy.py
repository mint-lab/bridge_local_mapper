
import numpy as np
import cv2 as cv
import open3d as o3d
from gtrack_mapper import create_mapper, test_pointcloud


if __name__ == '__main__':
    # Read a point cloud from a file.
    pcd_file = '../data/240819_HYU_Corridor/VGA_SN37875346_14-56-22_307.ply'
    pcd = o3d.io.read_point_cloud(pcd_file)
    pts = np.asarray(pcd.points)

    # Configure the parameters for the local mapper.
    params = {
        'mapper_name'           : 'GTrackMapper',
        'map_x_length'          : 10,
        'map_y_length'          : 5,
        'map_cellsize'          : 0.05,
        'pts_sampling_step'     : 1,
        'ransac_threshold'      : 0.02,
        'ransac_above_threshold': 2,
        'debug_info'            : True,
    }

    # Test the local mapper.
    mapper = create_mapper(params['mapper_name'], params)
    mapper.set_params(params)
    test_pointcloud(mapper, pts, show_map=True, show_debug_info=mapper.params['debug_info'])