import numpy as np
import cv2 as cv
from zed import ZED, print_zed_info
import sys, argparse

def save_ply(filename, pointcloud):
    '''Save the given point cloud to a PLY file'''
    header = '''ply
    format ascii 1.0
    element vertex {vertex_count}
    property float x
    property float y
    property float z
    end_header
    '''.format(vertex_count=len(pointcloud))

    with open(filename, 'w') as f:
        f.write(header)
        for point in pointcloud:
            f.write('{} {} {}\n'.format(point[0], point[1], point[2]))

def play_svo(svo_file, svo_realtime=False, depth_mode='neural', zoom=0.5):
    '''Play the given SVO file'''

    zed = ZED()
    success = zed.open(svo_file=svo_file, svo_realtime=svo_realtime, depth_mode=depth_mode)
    if not success:
        return
    print_zed_info(zed)
    sys.stdout.flush()

    cv.namedWindow('ZED SVO Player: Color and Depth')
    while zed.grab():
        # Get all images
        color, _, depth = zed.get_images()

        # Show the images
        depth_color = cv.applyColorMap(depth, cv.COLORMAP_JET)
        merge = cv.resize(np.vstack((color, depth_color)), (0, 0), fx=zoom, fy=zoom)
        cv.imshow('ZED SVO Player: Color and Depth', merge)

        key = cv.waitKey(1)
        if key == ord(' '): # Space
            key = cv.waitKey(0)
            if key == ord('s') or key == ord('S'):
                # Save the current frame as image, depth, and point cloud files
                frame_number = zed.camera.get_svo_position()
                file_prefix = svo_file.rsplit('.', 1)[0]
                cv.imwrite(f'{file_prefix}_{frame_number}.png', color)
                np.savez_compressed(f'{file_prefix}_{frame_number}.npz', depth)
                pointcloud = zed.get_xyz().reshape(-1, 3)
                save_ply(f'{file_prefix}_{frame_number}.ply', pointcloud)
        if key == 27:       # ESC
            break

    cv.destroyAllWindows()
    zed.close()



if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='ZED SVO Player')
    parser.add_argument('svo_file',             nargs=1, type=str,                       help='the name of the SVO file (e.g. test.svo)')
    parser.add_argument('--svo_realtime', '-r', nargs=1, type=bool,  default=[False],    help='the flag to enable realtime SVO play (default: False)')
    parser.add_argument('--depth_mode',   '-d', nargs=1, type=str,   default=['neural'], help='the depth mode (default: "neural")')
    parser.add_argument('--zoom',         '-z', nargs=1, type=float, default=[0.5],      help='the zoom ratio of images (default: 0.5)')
    args = parser.parse_args()

    play_svo(args.svo_file[0], svo_realtime=args.svo_realtime[0], depth_mode=args.depth_mode[0], zoom=args.zoom[0])
