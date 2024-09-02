# Python packages
import numpy as np

# ROS 2 packages
import rclpy
from rclpy.node import Node
import sensor_msgs_py.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2
from nav_msgs.msg import OccupancyGrid

# Local packages
try:
    from gtrack_mapper import GTrackMapper
    # from o3d_mapper import O3DMapper
except ImportError:
    from bridge_local_planner.gtrack_mapper import GTrackMapper
    # from bridge_local_planner.o3d_mapper import O3DMapper


class LocalMapperNode(Node):
    """Local mapper wrapper (ROS 2 node)"""

    def __init__(self):
        """Initialize the local mapper node."""
        super().__init__('local_mapper_node')

        # Load parameters.
        self.declare_parameter('map_x_length', 10)
        self.declare_parameter('map_y_length', 10)
        self.declare_parameter('map_cellsize', 0.1)
        self.declare_parameter('elevation_alpha', 100)
        self.declare_parameter('config_file', '')

        self.map_x_length = float(self.get_parameter('map_x_length').value)
        self.map_y_length = float(self.get_parameter('map_y_length').value)
        self.map_cellsize = float(self.get_parameter('map_cellsize').value)
        self.elevation_alpha = float(self.get_parameter('elevation_alpha').value)

        # Initialize the local mapper.
        self.mapper = GTrackMapper(self.map_x_length, self.map_y_length, self.map_cellsize)
        self.mapper.set_params({
            'robot2sensor_T'    : np.eye(4),
            'pts_sampling_step' : 4,
            'ground_correct'    : True,
            'ground_mapping'    : True,
        })
        config_file = self.get_parameter('config_file').value
        if config_file:
            self.mapper.load_config(config_file)

        # Initialize the local (occupancy grid) maps.
        self.map_msg = {}
        self.map_array = {}
        for layer in self.mapper.map_data:
            map = OccupancyGrid()
            map.info.width = self.mapper.map_nx
            map.info.height = self.mapper.map_ny
            map.info.resolution = self.mapper.map_cellsize
            map.info.origin.position.x = -self.map_x_length / 2
            map.info.origin.position.y = -self.map_y_length / 2
            self.map_msg[layer] = map
            self.map_array[layer] = None

        # Initialize subscribers and publishers.
        self.pcd_subscriber = self.create_subscription(PointCloud2, '/local_mapper_node/point_cloud', self.subscribe_pointcloud2, 2)
        self.map_publishers = {}
        for layer in self.mapper.map_data:
            self.map_publishers[layer] = self.create_publisher(OccupancyGrid, f'/local_mapper_node/{layer}_map', 2)

        self.get_logger().info('[local_mapper_node] Started.')

    @staticmethod
    def conv_pointcloud2_to_array(pcd_msg):
        """Convert ROS PointCloud2 message to NumPy array."""
        field_names = [field.name for field in pcd_msg.fields]
        points = pc2.read_points(pcd_msg, field_names=field_names, skip_nans=True)
        pts = np.vstack((points['x'], points['y'], points['z'])).T
        return pts

    def subscribe_pointcloud2(self, msg):
        """Subscribe a PointCloud2 messages (callback function)."""
        pts = self.conv_pointcloud2_to_array(msg)

        # Apply the point cloud to the local mapper.
        start_time = self.get_clock().now()
        success = self.mapper.apply_pointcloud(pts)
        elapsed_time = self.get_clock().now() - start_time
        self.get_logger().info(f'[local_mapper_node] Computing time: {elapsed_time.nanoseconds / 1e6} [msec]')
        if not success:
            self.get_logger().warn('[local_mapper_node] Failed to apply the point cloud.')

        # Publish the local maps.
        self.publish_occupancy_maps(msg.header.stamp)

    def publish_occupancy_maps(self, timestamp):
        # Update the map messages.
        self.map_array['obstacles'] = self.mapper.map_data['obstacles']
        self.map_array['elevation'] = (self.mapper.map_data['elevation'] * self.elevation_alpha).clip(-128, 127).astype(np.int8)
        self.map_array['histogram'] = self.mapper.map_data['histogram'].clip(0, 127).astype(np.int8)

        # Publish the map messages.
        for layer in self.map_msg:
            map = self.map_msg[layer]
            map.header.frame_id = 'map' # TODO: 'local_map'
            map.header.stamp = timestamp
            map.data = self.map_array[layer].flatten().tolist()
            self.map_publishers[layer].publish(map)


def main(args=None):
    """Start the local mapper node."""
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