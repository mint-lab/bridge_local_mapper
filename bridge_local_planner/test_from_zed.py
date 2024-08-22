import time
import numpy as np
import cv2 as cv
import open3d as o3d
from o3d_local_mapper import O3DLocalMapper
from sensorpy.zed import ZED, print_zed_info


def test_from_zed(mapper: O3DLocalMapper, svo_file: str='', added_params: dict={}, print_info=True, print_time=True, print_debug=False, show_image=True, show_map=True):
    """Test the given local mapper with a ZED camera or a SVO file."""

    # Define the default parameters and update them with the given parameters.
    params = {
        'map_zoom'          : 2,
        'obstacle_alpha'    : 100,
        'obstacle_beta'     : 100,
        'elevation_alpha'   : 100,
        'elevation_beta'    : 100,
        'n_hits_alpha'      : 1,
        'n_hits_beta'       : 0,
        'image_zoom'        : 0.5,
        'zed_params'        : {
            'svo_realtime'  : False,
            'depth_mode'    : 'neural',
        }
    }
    params.update(added_params)

    # Open a ZED camera or a SVO file.
    zed = ZED()
    zed.open(svo_file=svo_file, **params['zed_params'])
    if print_info:
        print_zed_info(zed)

    while zed.is_open():
        time_start = time.time()
        # Grab the ZED measurements.
        if zed.grab():
            pts = zed.get_xyz().reshape(-1, 3)
            time_zed_grab = time.time()

            # Apply the point cloud to the mapper.
            pcd = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(pts))
            pcd.paint_uniform_color([0.5, 0.5, 0.5])
            time_o3d_pcd = time.time()
            mapper.apply_pointcloud(pcd)
            time_mapping = time.time()
            if print_time:
                print(f'* Time elapse: {(time_zed_grab-time_start)*1000:.0f} + {(time_o3d_pcd-time_zed_grab)*1000:.0f} + {(time_mapping-time_o3d_pcd)*1000:.0f} [msec]')
            if print_debug:
                if 'valid_pcd' in mapper.debug_info:
                    print(f'* The number of valid points: {len(mapper.debug_info["valid_pcd"].points)} / {len(pcd.points)}')

            # Show the images.
            if show_image:
                color, _, depth = zed.get_images()
                depth_color = cv.applyColorMap(depth, cv.COLORMAP_JET)
                merge = np.vstack((color, depth_color))
                merge = cv.resize(merge, (0, 0), fx=params['image_zoom'], fy=params['image_zoom'])
                cv.imshow('test_from_zed: Color+Depth Images', merge)

            # Show the local map data.
            if show_map:
                obstacle_map = (params['obstacle_alpha'] * mapper.map_data['obstacle'] + params['obstacle_beta']).clip(0, 255).astype(np.uint8)
                obstacle_map = cv.cvtColor(obstacle_map, cv.COLOR_GRAY2BGR)
                elevation_map = (params['elevation_alpha'] * mapper.map_data['elevation'] + params['elevation_beta']).clip(0, 255).astype(np.uint8)
                elevation_map = cv.cvtColor(elevation_map, cv.COLOR_GRAY2BGR)
                n_hits_map = (params['n_hits_alpha'] * mapper.map_data['n_hits'] + params['n_hits_beta']).clip(0, 255).astype(np.uint8)
                n_hits_map = cv.cvtColor(n_hits_map, cv.COLOR_GRAY2BGR)
                merge = np.flipud(np.hstack((obstacle_map, elevation_map, n_hits_map))) # Make the Y-axis upward.
                merge = cv.resize(merge, (0, 0), fx=params['map_zoom'], fy=params['map_zoom'])
                cv.imshow('test_from_zed: Obstacle+Elevation+Hits Maps', merge)

        key = cv.waitKey(1)
        if key == ord(' '): # Space
            key = cv.waitKey(0)
        if key == 27:       # ESC
            break

    cv.destroyAllWindows()
    zed.close()


if __name__ == '__main__':
    svo_file = '../data/231031_HYU_Yang/HD720_SN30097042_17-01-03.svo'
    svo_file = '../data/231031_HYU_Yang/HD720_SN30097042_17-01-38.svo'
    svo_file = '../data/231031_HYU_Yang/VGA_SN30097042_15-18-02.svo'
    svo_file = '../data/231031_HYU_Yang/VGA_SN30097042_15-19-51.svo'
    svo_file = '../data/231031_HYU_Yang/VGA_SN30097042_15-21-15.svo'
    # svo_file = '' # Use a ZED camera directly.

    # Test the local mapper.
    mapper = O3DLocalMapper()
    mapper.params['pcd_sampling_step'] = 4
    mapper.params['ground_mapping'] = True
    mapper.params['debug_info'] = False
    test_from_zed(mapper, svo_file, print_info=False, print_debug=mapper.params['debug_info'])