## bridge_local_planner

_bridge_local_planner_는 NRF-Bridge 프로젝트에서 **지역 지도 작성 (local mapping)** 및 **지역 경로 생성 (local path planning; LPP)**을 위한 소프트웨어이다.



### 1. 주요 기술 소개
* **Local Mappers**
  * `gtrack_mapper.py`
    * 지면(ground plane)의 제약조건(constraints)과 추적(tracking)을 이용한 지면 검출과 장애물 분리를 이용한 local mapping 알고리즘
  * `o3d_mapper.py`
    * 지면 검출에 Open3D의 RANSAC 기반 plane detection을 이용한 local mapping 알고리즘
* **Local Path Planners**
* **ROS 2 Nodes**
  * `local_mapper_node.py`
    * ROS 2의 기본 메세지를 통한 local mapping 노드
    * Subscribers: `local_mapper_node/point_cloud`
    * Publishers: `local_mapper_node/local_map`



### 2. 설치 방법
[INSTALL.md](INSTALL.md) 참고



### 3. 사용 방법
#### 3.1. Pure Python 환경에서 동작
* Local mappers 기본 예제

  ```python
  from gtrack_mapper import GTrackMapper, generate_pointcloud
  
  # Instantiate the local mapper and configure it
  mapper = GTrackMapper()
  mapper.set_params({'pts_sampling_step': 2, 'debug_info': True})
  
  # Prepare a point cloud
  pts = generate_pointcloud()
  
  # Apply the point cloud and access the updated map data
  success = mapper.apply_pointcloud(pts)
  print(mapper.map_data)
  ```

* ZED 카메라 또는 SVO 파일을 이용한 데모: `test_from_zed.py`



#### 3.2. ROS 2 환경에서 동작
* Local mapper 노드 실행: `ros2 run bridge_planner local_mapper_node [--ros-args --remap /local_mapper_node/point_cloud:=/your/published/point_cloud/topic]`
  * `--remap` 옵션:  노드에 입력할 point cloud 토픽 설정
    * 예: `--remap /local_mapper_node/point_cloud:=/zed2i/zed_node/point_cloud/cloud_registered`



### 사사
본 소프트웨어는 과학기술정보통신부 및 한국연구재단의 ‘BRIDGE융합연구개발’사업의 지원으로 작성되었습니다. (과제명: AI기반 3차원 곡면에서의 위치 인식 및 이동 경로 생성 기술, 과제번호: 2021M3C1C3096810)