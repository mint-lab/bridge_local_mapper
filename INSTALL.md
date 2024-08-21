## bridge_local_planner 설치 방법

### Repository Clone 및 Python 필수 패키지 설치
* (ROS workspace의 src 디렉토리로 이동)
* `git clone https://github.com/mint-lab/bridge_local_planner.git`
* `cd bridge_local_planner/bridge_local_planner`
* `pip install -r requirements.txt`



### (옵션) ZED SDK 설치
* 필요한 경우: `test_from_zed.py` 실행
* [ZED SDK](https://www.stereolabs.com/en-tw/developers/release) 설치
  * 해당 SDK에 요구하는 [NVIDIA CUDA ToolKit](https://developer.nvidia.com/cuda-toolkit-archive) 설치 필요
* ZED SDK의 Python에서 사용을 위해
  * ZED SDK가 설치된 디렉토리로 이동
    * Windows의 경우 디렉토리 위치 예: `C:\Program Files (x86)\ZED SDK`
  * `get_python_api.py` 실행
    * Windows의 C 드라이브에서 실행하는 경우, 파일 다운로드를 위해 `관리자 권한으로 실행` 필요
    * 다운로드된 whl 파일 삭제