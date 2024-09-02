import time
import numpy as np
import cv2 as cv
from sensorpy.zed import ZED, print_zed_info
from gtrack_mapper import GTrackMapper, print_debug_info
from o3d_mapper import O3DMapper


def mouse_callback(event, x, y, flags, params):
    if event == cv.EVENT_LBUTTONDBLCLK:
        # Print the map data at the clicked point.
        r = params['mapper'].map_ny - int((y / params['map_zoom']) % params['mapper'].map_ny + 0.5)
        c = int((x / params['map_zoom']) % params['mapper'].map_nx + 0.5)
        x, y = params['mapper'].conv_rc2xy(r, c)
        print(f'* Index (row, col): [{r}, {c}] / Location (x, y): [{x:.3f}, {y:.3f}] [m]')
        if 0 <= r < params['mapper'].map_ny and 0 <= c < params['mapper'].map_nx:
            print(f'  * Obstacles: {params["mapper"].map_data["obstacles"][r, c]:.0f}')
            print(f'  * Elevation: {params["mapper"].map_data["elevation"][r, c]:.3f} [m]')
            print(f'  * Histogram: {params["mapper"].map_data["histogram"][r, c]:.0f}')


def test_from_zed(mapper: GTrackMapper, svo_file: str='', added_params: dict={}, print_info=True, print_time=True, print_debug=False, show_image=True, show_map=True):
    """Test the given local mapper with a ZED camera or a SVO file."""

    # Define default parameters and update them with the given parameters.
    zed_params = {
        'svo_realtime'      : False,
        'depth_mode'        : 'neural',
    }
    if 'zed_params' in added_params:
        zed_params.update(added_params['zed_params'])

    test_params = {
        'map_zoom'          : 2,
        'obstacles_alpha'   : 200,
        'obstacles_beta'    : 0,
        'elevation_alpha'   : 100,
        'elevation_beta'    : 100,
        'histogram_alpha'   : 1,
        'histogram_beta'    : 0,
        'image_zoom'        : 0.5,
    }
    test_params.update(added_params)

    # Create an window and set its mouse callback.
    cv.namedWindow('test_from_zed: Obstacles + Elevation + Histogram Maps', cv.WINDOW_NORMAL)
    mouse_param = { 'mapper': mapper, 'map_zoom': test_params['map_zoom'] }
    cv.setMouseCallback('test_from_zed: Obstacles + Elevation + Histogram Maps', mouse_callback, param=mouse_param)

    # Open a ZED camera or a SVO file.
    zed = ZED()
    success = zed.open(svo_file=svo_file, **zed_params)
    if not success:
        return
    if print_info:
        print_zed_info(zed)

    while True:
        # Grab the ZED measurements.
        time_start = time.time()
        if not zed.grab():
            break
        pts = zed.get_xyz().reshape(-1, 3)
        time_grab = time.time()

        # Apply the point cloud to the mapper.
        success = mapper.apply_pointcloud(pts)
        time_mapping = time.time()
        if print_time:
            print(f'* Computing time: {(time_grab-time_start)*1000:.0f} + {(time_mapping-time_grab)*1000:.0f} [msec] (success: {success})')
        if print_debug:
            print_debug_info(mapper.debug_info, len(pts))

        # Show the images.
        if show_image:
            color, _, depth = zed.get_images()
            depth_color = cv.applyColorMap(depth, cv.COLORMAP_JET)
            merge = np.vstack((color, depth_color))
            merge = cv.resize(merge, (0, 0), fx=test_params['image_zoom'], fy=test_params['image_zoom'])
            cv.imshow('test_from_zed: Color + Depth Images', merge)

        # Show the local map data.
        if show_map:
            obstacles = mapper.convert_cv_map(mapper.map_data['obstacles'], alpha=test_params['obstacles_alpha'], beta=test_params['obstacles_beta'])
            elevation = mapper.convert_cv_map(mapper.map_data['elevation'], alpha=test_params['elevation_alpha'], beta=test_params['elevation_beta'])
            histogram = mapper.convert_cv_map(mapper.map_data['histogram'], alpha=test_params['histogram_alpha'], beta=test_params['histogram_beta'])
            merge = np.flipud(np.hstack((obstacles, elevation, histogram))) # Make the Y-axis upward.
            merge = cv.resize(merge, (0, 0), fx=test_params['map_zoom'], fy=test_params['map_zoom'])
            cv.imshow('test_from_zed: Obstacles + Elevation + Histogram Maps', merge)

        key = cv.waitKey(1)
        if key == ord(' '): # Space
            key = cv.waitKey(0)
        if key == 27:       # ESC
            break

    cv.destroyAllWindows()
    zed.close()


if __name__ == '__main__':
    # svo_file = '../data/231031_HYU_Yang/HD720_SN30097042_17-01-03.svo'
    # svo_file = '../data/231031_HYU_Yang/HD720_SN30097042_17-21-38.svo'
    # svo_file = '../data/231031_HYU_Yang/VGA_SN30097042_15-18-02.svo'
    # svo_file = '../data/231031_HYU_Yang/VGA_SN30097042_15-19-51.svo'
    # svo_file = '../data/231031_HYU_Yang/VGA_SN30097042_15-21-15.svo'
    # svo_file = '../data/240819_HYU_Corridor/VGA_SN37875346_14-52-46.svo2'
    svo_file = '../data/240819_HYU_Corridor/VGA_SN37875346_14-56-22.svo2'
    # svo_file = '' # Use a ZED camera directly.

    # Test the local mapper.
    mapper = GTrackMapper(10, 10, 0.05)
    mapper.set_params({
        'pts_sampling_step' : 4,
        'ransac_threshold'  : 0.1,
        'ground_mapping'    : True,
        'debug_info'        : False,
    })
    test_params = {
        'zed_params'        : {
            'svo_realtime'  : True,
        },
        'map_zoom'          : 2,
        'image_zoom'        : 0.5,
    }
    test_from_zed(mapper, svo_file, test_params, print_info=False, print_debug=mapper.params['debug_info'])