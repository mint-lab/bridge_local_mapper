import numpy as np
import open3d as o3d

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, MultiArrayDimension
import sensor_msgs_py.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2
from grid_map_msgs.msg import GridMap

try:
    from o3d_mapper import O3DMapper
except ImportError:
    from bridge_planner.o3d_mapper import O3DMapper, generate_pointcloud


class LocalMapperNode(Node):
    """Local Mapper Wrapper (ROS 2 Node)"""

    def __init__(self):
        """Initialize the local mapper node."""
        super().__init__('local_mapper_node')

        # Load parameters.
        self.declare_parameter('map_x_width', 10)
        self.declare_parameter('map_y_width', 10)
        self.declare_parameter('map_cellsize', 0.1)

        self.map_x_width = float(self.get_parameter('map_x_width').value)
        self.map_y_width = float(self.get_parameter('map_y_width').value)
        self.map_cellsize = float(self.get_parameter('map_cellsize').value)

        # Initialize the local mapper.
        self.local_mapper = O3DMapper(map_x_width=self.map_x_width, map_y_width=self.map_y_width, map_cellsize=self.map_cellsize)
        self.local_mapper.set_params({'robot2sensor_T': np.eye(4)})
        self.local_mapper.set_params({'ground_correct': False})
        self.local_mapper.set_params({'ground_mapping': True})
        self.local_mapper.set_params({'pcd_sampling_step': 4})

        # Initialize subscribers and publishers.
        self.pcd_subscriber = self.create_subscription(PointCloud2, '/local_mapper_node/point_cloud', self.callback_pointcloud2, 2)
        self.map_publisher = self.create_publisher(GridMap, '/local_mapper_node/local_map', 2)

        self.get_logger().info('[local_mapper_node] Started.')

    @staticmethod
    def conv_pointcloud2_to_o3dpcd(cloud_msg):
        """Convert ROS PointCloud2 message to Open3D PointCloud."""
        field_names = [field.name for field in cloud_msg.fields]
        points = pc2.read_points(cloud_msg, field_names=field_names, skip_nans=True)
        xyz = np.vstack((points['x'], points['y'], points['z'])).T
        #rgb = ...

        pcd = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(xyz))
        pcd.paint_uniform_color([0.5, 0.5, 0.5])
        return pcd

    @staticmethod
    def conv_nparray_to_multiarray(np_array):
        """Convert a NumPy array to Float32MultiArray."""
        multi_array = Float32MultiArray()
        if np_array.ndim == 2:
            rows, cols = np_array.shape
            multi_array.layout.dim.append(MultiArrayDimension(label='dim1', size=cols, stride=1))
            multi_array.layout.dim.append(MultiArrayDimension(label='dim0', size=rows, stride=cols))
            multi_array.layout.data_offset = 0
            multi_array.data = np_array.flatten().tolist()
        return multi_array

    def callback_pointcloud2(self, msg):
        """Callback function for PointCloud2 messages."""
        pcd = self.conv_pointcloud2_to_o3dpcd(msg)
        # pcd = generate_pointcloud()
        start_time = self.get_clock().now()
        success = self.local_mapper.apply_pointcloud(pcd)
        elapsed_time = self.get_clock().now() - start_time
        self.get_logger().info(f'[local_mapper_node] Computing time: {elapsed_time.nanoseconds / 1e6} [msec]')
        if not success:
            self.get_logger().warn('[local_mapper_node] Failed to apply point cloud.')

        self.publish_map(msg.header.stamp)

    def publish_map(self, timestamp):
        map = GridMap()
        map.info.resolution = self.map_cellsize
        map.info.length_x = self.map_x_width
        map.info.length_y = self.map_y_width

        map.layers.append('elevation')
        map.data.append(self.conv_nparray_to_multiarray(self.local_mapper.map_data['elevation']))
        # map.layers.append('n_hits')
        # map.data.append(self.conv_nparray_to_multiarray(self.local_mapper.map_data['n_hits']))
        # map.layers.append('obstacle')
        # map.data.append(self.conv_nparray_to_multiarray(self.local_mapper.map_data['obstacle']))

        map.header.frame_id = 'map' # 'local_map'
        map.header.stamp = timestamp
        self.map_publisher.publish(map)


def main(args=None):
    """Start the local_mapper_node"""
    rclpy.init(args=args)
    node = LocalMapperNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()