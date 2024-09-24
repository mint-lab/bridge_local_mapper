## bridge_local_planner의 ROS 2 환경 사용법

### local_mapper_node 노드 소개
* `local_mapper_node.py`: **Local mapper wrapper**
  * Local map을 `nav_msgs/OccupancyGrid` 메세지로 publish하는 ROS 2 node
  * Subscribed topics
    * Point cloud: `local_mapper_node/point_cloud`
      * Message type: `sensor_msgs/PointCloud2`
      * Unit: [m]
  * Publishing topics
    * Obstacles map: `local_mapper_node/obstacles_map`
      * Message type: `nav_msgs/OccupancyGrid`
      * Values: -1 (unknown), 0 (empty), 1-100 (occupancy probability)
      * Unit: [%]
    * Elevation map: `local_mapper_node/elevation_map`
      * Message type: `nav_msgs/OccupancyGrid`
      * Value range: [-128, 127]
      * Unit: [cm]
    * Histogram map: `local_mapper_node/histogram_map`
      * Message type: `nav_msgs/OccupancyGrid`
      * Value range: [0, 127]
    * Front (first) obstacle: `local_mapper_node/front_obstacle`
      * Message type: `geometry_msgs/Point`
      * Value examples: No obstacle (-1, 0, -1), Obstacle (distance, 0, height)
      * Unit: [m]
  * Parameters
    * `map_x_length`: 지도의 X 방향 길이 (Default: `10`, Unit: [m])
    * `map_y_length`: 지도의 Y 방향 길이 (Default: `10`, Unit: [m])
    * `map_cellsize`: 지도의 셀의 크기 (Default: `0.1`, Unit: [m])
    * `map_frame_id`: Publish되는 지도의 frame ID (Default: `"map"`)
    * `elevation_scale`: Elevation map의 스케일 (Default: `100`)
      * 참고) 스케일 값이 100으로 설정된 경우, [m] 단위의 값을 100배하여 [cm] 단위로 바꾼다.
    * `verbose`: 알고리즘의 실행시간 등 추가적인 정보를 화면에 출력 (Default: `True`)
    * `config_params`: 위의 1장에 설명된 세부 parameters를 YAML 형식의 텍스트로 설정 가능 (Default: `""`)
      * 설정 예: [config/bridge_orin.yaml](https://github.com/mint-lab/bridge_local_planner/blob/master/config/bridge_orin.yaml) 파일 내 `config_params` 항목
    * `config_file`: 위의 1장에 설명된 세부 parameters를 YAML 파일로 설정 가능 (Default: `""`)



### local_mapper_node 노드 사용예
* `local_mapper_node` 실행
  * `ros2 run bridge_local_planner local_mapper_node [--ros-args <options>]`
    * `--remap`  (`-r`) 옵션:  노드에 입력할 토픽 이름 맵핑
      * 사용 예: `--remap /local_mapper_node/point_cloud:=/zed/zed_node/point_cloud/cloud_registered`
    * `--params_file` 옵션: 노드에 설정할 파라미터들이 정의된 파일 설정
      * 사용 예: `--params-file src/bridge_local_planner/config/bridge_orin.yaml`
    * `--param` (`-p`) 옵션: 위의 3.2 절에 소개된 세부 parameters 설정 가능
      * 사용 예: `--param map_frame_id:="robot"`
  * 사용 예 (`local_mapper_node`가 설치된 ROS 2 workspace에서 실행하는 경우)
    ```bash
    ros2 run bridge_local_planner local_mapper_node --ros-args
      --remap /local_mapper_node/point_cloud:=/zed/zed_node/point_cloud/cloud_registered
      --params-file src/bridge_local_planner/config/bridge_orin.yaml
    ```
* `rviz2`에서 지도 등 publish된 모든 토픽들 가시화 가능



### FAQ
Q) ZED의 데이터가 제대로 들어오지 않는다.
* A) ZED의 USB 케이블이 USB 3.0 이상을 지원하는지 확인한다. 또 USB 케이블이 허브를 통해 연결되어 있는지 확인한다. USB 3.0을 지원하는 허브도 경우에 따라 ZED 카메라가 작동하지 않으므로 직접 연결하는 것을 추천한다.

Q) NVIDIA Jetson AGX Orin의 ROS 2 환경에서 ZED에서 publish하는 `depth` 토픽이나 `point_cloud` 토픽이 제대로 된 값을 주지 않는다.
* A) ZED의 `NEURAL` 모드 depth estimation이 Orin에서 작동하지 않는다. (인터넷을 통한 신경망 가중치 다운로드가 불가능했는지 이유는 알 수 없다.) ZED 노드의 yaml 파일 설정에서 `NEURAL` 대신 `ULTRA`를 사용한다.