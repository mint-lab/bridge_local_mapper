# Python packages
import numpy as np
import yaml

# ROS 2 packages
import rclpy
from rclpy.node import Node
import sensor_msgs_py.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PointStamped

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
        self.declare_parameter('map_frame_id', 'map')
        self.declare_parameter('elevation_scale', 100)
        self.declare_parameter('verbose', True)
        self.declare_parameter('config_params', '')
        self.declare_parameter('config_file', '')

        self.map_x_length = float(self.get_parameter('map_x_length').value)
        self.map_y_length = float(self.get_parameter('map_y_length').value)
        self.map_cellsize = float(self.get_parameter('map_cellsize').value)
        self.map_frame_id = str(self.get_parameter('map_frame_id').value)
        self.elevation_scale = float(self.get_parameter('elevation_scale').value)
        self.verbose = bool(self.get_parameter('verbose').value)

        # Initialize the local mapper.
        self.mapper = GTrackMapper(self.map_x_length, self.map_y_length, self.map_cellsize)
        self.mapper.set_params({
            'robot2sensor_T'    : np.eye(4),
            'pts_sampling_step' : 4,
        }) # The default paraeters for ROS 2 environment

        config_params = self.get_parameter('config_params').value
        if config_params:
            config_dict = yaml.safe_load(config_params)
            self.mapper.set_params(config_dict)
        config_file = self.get_parameter('config_file').value
        if config_file:
            self.mapper.load_params_from_yaml(config_file)

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
        self.node_name = self.get_name()
        self.pcd_subscriber = self.create_subscription(PointCloud2, f'/{self.node_name}/point_cloud', self.subscribe_pointcloud2, 2)
        self.map_publishers = {}
        for layer in self.mapper.map_data:
            self.map_publishers[layer] = self.create_publisher(OccupancyGrid, f'/{self.node_name}/{layer}_map', 2)
        self.obs_publisher = self.create_publisher(PointStamped, f'/{self.node_name}/front_obstacle', 2)

        if self.verbose:
            self.get_logger().info(f'[{self.node_name}] Started.')

    @staticmethod
    def conv_pointcloud2_to_array(pcd_msg):
        """Convert ROS PointCloud2 message to NumPy array."""
        field_names = [field.name for field in pcd_msg.fields]
        points = pc2.read_points(pcd_msg, field_names=field_names, skip_nans=True)
        pts = np.vstack((points['x'], points['y'], points['z'])).T
        return pts

    def subscribe_pointcloud2(self, msg):
        """Subscribe a PointCloud2 message (callback function)."""
        pts = self.conv_pointcloud2_to_array(msg)
        if len(pts) <= 0:
            self.get_logger().warn(f'[{self.node_name}] Empty point cloud.')

        # Apply the point cloud to the local mapper.
        start_time = self.get_clock().now()
        success = self.mapper.apply_pointcloud(pts)
        elapsed_time = self.get_clock().now() - start_time
        if self.verbose:
            self.get_logger().info(f'[{self.node_name}] Computing time: {elapsed_time.nanoseconds / 1e6} [msec]')
        if not success:
            self.get_logger().warn(f'[{self.node_name}] Failed to apply the point cloud.')

        # Publish the local maps and the front obstacle.
        self.publish_occupancy_maps(msg.header.stamp)
        self.publish_front_obstacle(msg.header.stamp)

    def publish_occupancy_maps(self, timestamp):
        """Publish the local maps as OccupancyGrid messages."""
        # Update the map messages.
        self.map_array['obstacles'] = self.mapper.map_data['obstacles']
        self.map_array['elevation'] = (self.mapper.map_data['elevation'] * self.elevation_scale).clip(-128, 127).astype(np.int8)
        self.map_array['histogram'] = self.mapper.map_data['histogram'].clip(0, 127).astype(np.int8)

        # Publish the map messages.
        for layer in self.map_msg:
            map = self.map_msg[layer]
            map.header.frame_id = self.map_frame_id
            map.header.stamp = timestamp
            map.data = self.map_array[layer].flatten().tolist()
            self.map_publishers[layer].publish(map)

    def publish_front_obstacle(self, timestamp):
        """Publish the front obstacle as a Point message."""
        r_center, c_center = self.mapper.conv_xy2rc(0, 0)
        c_length = len(self.mapper.map_data['obstacles'][0])
        obs = PointStamped()
        obs.header.stamp = timestamp
        obs.header.frame_id = self.map_frame_id
        obs.point.x = -1.
        obs.point.y = 0.
        obs.point.z = -1.
        for c in range(c_center, c_length):
            if self.mapper.map_data['obstacles'][r_center, c] > 50:
                obs.point.x, _ = self.mapper.conv_rc2xy(r_center, c)
                obs.point.z = float(self.mapper.map_data['elevation'][r_center, c])
                break
        self.obs_publisher.publish(obs)


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