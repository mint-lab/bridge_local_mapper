## bridge_local_planner

_bridge\_local\_planner_는 NRF-Bridge 프로젝트에서 **지역 지도 작성 (local mapping)** 및 **지역 경로 생성 (local path planning; LPP)**을 위한 소프트웨어이다.



### 1. 주요 기술 소개
* **Local mapping**
  * `gtrack_mapper.py`: Local mapper with ground plane constraints, asymmetric MSAC, and plane tracking (**추천**)
    * 지면의 제약조건과 비대칭 MSAC, 그리고 평면 추적을 모두 이용한 지면 검출과 장애물 분리를 이용한 local mapping 알고리즘
  * `o3d_mapper.py`: Local mapper using Open3D plane detection
    * 지면 검출에 Open3D의 RANSAC 기반 plane detection을 이용한 local mapping 알고리즘
    * 매개변수
      * `ransac_num_iters`: RANSAC 기반 plane detection의 (최대) 반복 횟수 (기본값: 1000)
      * `ransac_num_samples`: Plane fitting에 사용될 점의 개수 (기본값: 3)
      * `ransac_threshold`: Plane fitting 결과와 점 사이의 거리 임계값 (기본값: 0.05, 단위: [m])
  * `gconst_mapper.py`: Local mapper with ground plane constraints
    * 지면의 제약조건과 RANSAC의 adaptive termination을 적용한 local mapping 알고리즘
    * 추가 매개변수
      * `ransac_min_iters`: RANSAC의 최소 반복 횟수 (기본값: 10)
      * `ransac_confidence`: RANSAC의 반복 횟수를 계산하기 위한 신뢰도 값 (기본값: 0.99)
      * `ransac_refinement`: RANSAC 후에 inlier 점들을 이용한 plane fitting 재수행 여부 (기본값: True)
      * `plane_norm_threshold`: Plane fitting의 cross product의 크기 (plane fitting의 stability) (기본값: 1e-6)
      * `plane_z_threshold`: Plane fitting 결과의 법선 벡터의 Z축 값 (기본값: 0.5)
      * `plane_max_height`: Plane fitting 결과와 로봇좌표계 사이의 최대 직선거리 (기본값: 1.5, 단위: [m])
  * `gmsac_mapper.py`: Local mapper with ground plane constraints and MSAC
    * 지면의 제약조건과 MSAC을 적용한 local mapping 알고리즘
  
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
  * `local_mapper_node.py`: Local mapper wrapper (ROS 2 node)
    * Local map을 `nav_msgs/OccupancyGrid` 메세지로 publish하는 ROS 2 node
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