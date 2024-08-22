## bridge_local_planner 설치

### Repository Clone 및 Python 패키지 설치
* (옵션) ROS workspace의 src 디렉토리로 이동
* Repository clone
  * `git clone https://github.com/mint-lab/bridge_local_planner.git`

* Python 패키지 설치
  * `pip install -r bridge_local_planner/bridge_local_planner/requirements.txt`



### (옵션) ROS 2 설치
* ROS 2 설치가 필요한 경우: `local_mapper_node.py` 실행
* [ROS 2](https://docs.ros.org/en/humble/Installation.html) 설치
  * 현재 ROS 2 Humble 기준으로 테스트 완료
* ROS workspace로 이동하여 빌드
  * `colcon build [--symlink-install] [--packages-select bridge_local_planner]`
    * `--symlink-install` 옵션: Symbolic link를 이용하여 Python에서 추가적인 빌드없이 수정 사항 바로 적용 가능
    * `--packages-select bridge_local_planner` 옵션: bridge_local_planner 패키지만 빌드




### (옵션) ZED SDK 설치
* ZED SDK 설치가 필요한 경우: `test_from_zed.py` 실행
* [ZED SDK](https://www.stereolabs.com/en-tw/developers/release) 설치
  * 해당 SDK에 요구하는 [NVIDIA CUDA ToolKit](https://developer.nvidia.com/cuda-toolkit-archive) 설치 필요
* ZED SDK의 Python 패키지 설치
  * ZED SDK가 설치된 디렉토리로 이동
    * Windows의 경우 디렉토리 위치 예: `C:\Program Files (x86)\ZED SDK`
  * `get_python_api.py` 실행
    * Windows의 C 드라이브에서 실행하는 경우, 파일 다운로드를 위해 `관리자 권한으로 실행` 필요
  * 다운로드된 whl 파일 삭제