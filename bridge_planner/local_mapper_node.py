import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, MultiArrayDimension
from grid_map_msgs.msg import GridMap

class ElevationMapPublisher(Node):
    def __init__(self):
        super().__init__('elevation_map_publisher')

        # 퍼블리셔 초기화
        self.publisher = self.create_publisher(GridMap, 'elevation_map', 1)

        # 타이머 설정: 2초마다 맵을 퍼블리시
        self.timer = self.create_timer(2.0, self.publish_map)

        # 맵 정보 설정
        self.map_width = 20
        self.map_height = 10
        self.resolution = 0.5  # 각 셀의 크기 (meters)

        self.get_logger().info('Elevation map publisher node started.')

    def generate_elevation_data(self):
        """임의의 고도 데이터를 생성합니다."""
        return np.random.rand(self.map_width, self.map_height) * 2.0  # 0 to 2 meters

    def publish_map(self):
        """GridMap 메시지를 퍼블리시합니다."""
        # GridMap 메시지 생성
        map = GridMap()

        # 헤더 설정
        map.info.resolution = self.resolution
        map.info.length_x = self.map_width * self.resolution
        map.info.length_y = self.map_height * self.resolution
        map.info.pose.position.x = self.map_width * self.resolution / 2
        map.info.pose.position.y = self.map_height * self.resolution / 2

        # Elevation 레이어 추가
        map.layers.append('elevation')
        elevation_layer = Float32MultiArray()
        elevation_layer.layout.dim.append(MultiArrayDimension(label='x', size=self.map_width,  stride=1))
        elevation_layer.layout.dim.append(MultiArrayDimension(label='y', size=self.map_height, stride=self.map_width))
        elevation_layer.layout.data_offset = 0
        elevation_layer.data = self.generate_elevation_data().flatten().tolist()
        map.data.append(elevation_layer)

        # 프레임 설정
        map.header.frame_id = 'map'
        map.header.stamp = self.get_clock().now().to_msg()

        # 퍼블리시
        self.publisher.publish(map)
        self.get_logger().info('Published elevation map as GridMap.')

def main(args=None):
    rclpy.init(args=args)
    node = ElevationMapPublisher()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    # 노드 종료
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()