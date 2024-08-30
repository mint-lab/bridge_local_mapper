## bridge_local_planner

_bridge\_local\_planner_는 NRF-Bridge 프로젝트에서 **지역 지도 작성 (local mapping)** 및 **지역 경로 생성 (local path planning; LPP)**을 위한 소프트웨어이다.



### 1. 주요 기술 소개
* **Local mapping**
  * `gtrack_mapper.py`: Local mapper with ground plane constraints and tracking (**추천**)
    * 지면(ground plane)의 제약조건(constraints)과 추적(tracking)을 이용한 지면 검출과 장애물 분리를 이용한 local mapping 알고리즘
  * `gconst_mapper.py`: Local mapper with ground plane constraints
    * 지면(ground plane)의 제약조건(constraints)만 이용한 local mapping 알고리즘
  * `o3d_mapper.py`: Local mapper with Open3D plane detection
    * 지면 검출에 Open3D의 RANSAC 기반 plane detection을 이용한 local mapping 알고리즘

* **Local path planning**
  * `straight_planner.py`



### 2. 설치 방법
[INSTALL.md](INSTALL.md) 참고



### 3. 사용 방법
#### 3.1. Pure Python 환경
* **Local mapper 기본 예제**

  ```python
  from gtrack_mapper import GTrackMapper, generate_pointcloud
  
  # Instantiate the local mapper and configure it
  mapper = GTrackMapper()
  mapper.set_params({'pts_sampling_step': 2, 'debug_info': True})
  
  # Prepare a point cloud
  pts = generate_pointcloud()
  
  # Apply the point cloud
  success = mapper.apply_pointcloud(pts)
  
  # Access the updated map data
  r, c = mapper.conv_xy2rc(3, 2) # meter to index
  is_object = mapper.map_data['obstacles'][r, c] == 0
  elevation = mapper.map_data['elevation'][r, c]
  histogram = mapper.map_data['histogram'][r, c]
  ```
  
* **PLY point cloud 파일을 이용한 local mapper 데모**
  [bridge_local_planner/gtrack_mapper.py](https://github.com/mint-lab/bridge_local_planner/blob/master/bridge_local_planner/test_from_zed.py)의 `test_pointcloud()` 함수 참고

* **ZED 카메라 또는 SVO 동영상 파일을 이용한 local mapper 데모**
  [bridge_local_planner/test_from_zed.py](https://github.com/mint-lab/bridge_local_planner/blob/master/bridge_local_planner/test_from_zed.py)의 `test_from_zed()` 함수 참고



#### 3.2. ROS 2 노드
  * `local_mapper_node.py`: ROS 2의 기본 메세지를 이용한 local mapper의 ROS 2 node
    * Subscribed topics
      * Point cloud: `local_mapper_node/point_cloud` (type: `sensor_msgs/PointCloud2`, unit: [m]) 
    * Publishing topics
      * Obstacles map: `local_mapper_node/obstacles_map` (type: `nav_msgs/OccupancyGrid`)
        * Values: -1 (unknown), 0 (empty), 1-100 (occupancy probability)
      * Elevation map: `local_mapper_node/elevation_map` (type: `nav_msgs/OccupancyGrid`)
        * Value range: [-128, 127]
        * Unit: [cm]
      * Histogram map: `local_mapper_node/histogram_map` (type: `nav_msgs/OccupancyGrid`)
        * Value range: [0, 127]



#### 3.3. ROS 2 환경
* **Local mapper 노드 실행**
  * `ros2 run bridge_planner local_mapper_node [--ros-args --remap /local_mapper_node/point_cloud:=/your/published/point_cloud/topic]`
    * `--remap` 옵션:  노드에 입력할 point cloud 토픽 설정
      * 예: `--remap /local_mapper_node/point_cloud:=/zed2i/zed_node/point_cloud/cloud_registered`
  * rviz2에서 지도 가시화 가능



### 사사
본 소프트웨어는 과학기술정보통신부 및 한국연구재단의 ‘BRIDGE융합연구개발’사업의 지원으로 작성되었습니다. (과제명: AI기반 3차원 곡면에서의 위치 인식 및 이동 경로 생성 기술, 과제번호: 2021M3C1C3096810)