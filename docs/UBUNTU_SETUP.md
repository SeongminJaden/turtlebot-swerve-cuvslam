# Ubuntu 환경 구축

## 고정 버전

| 계층 | 기준 |
|---|---|
| OS | Ubuntu 24.04 x86_64, 우선 네이티브 설치 |
| Sim | Isaac Sim 6.1.0.0, Python 3.12 |
| ROS | Isaac ROS release-5.0, ROS 2 Lyrical |
| GPU 런타임 | Isaac ROS 5.0 요구: CUDA 13.2+, NVIDIA 드라이버 595+ |
| Lab | Isaac Lab v3.0.0-EA — Sim 6.1 대상 태그 |

Lab 3.0 GA는 Sim 6.2를 대상으로 하므로 `main` 또는 최신 태그로 무조건 바꾸지 않습니다.
NVIDIA 문서의 이후 수정에 대비해 설치 직전에 아래 공식 링크를 다시 확인합니다.
Sim 최소 사양표는 VRAM 16 GB입니다. RTX 5060 Laptop 8 GB에서 큰 장면/센서/병렬 학습 성공을 보장하지 않습니다.

## 1. 시스템 확인

```bash
lsb_release -a
nvidia-smi
python3 --version
```

기존 조사 PC의 드라이버는 592.01로 ROS 5.0 요구 조건보다 낮았습니다. 대상 Ubuntu PC에서 다시 확인합니다.
WSL을 사용한다면 Windows 호스트 드라이버를 사용하며 Linux 디스플레이 드라이버를 WSL에 추가 설치하지 않습니다.

## 2. Sim 6.1

공식 NVIDIA 라이선스를 확인한 뒤 별도 환경에 설치합니다.

```bash
python3.12 -m venv .venv-sim61
source .venv-sim61/bin/activate
python -m pip install --upgrade pip
python -m pip install 'isaacsim[all,extscache]==6.1.0.0' --extra-index-url https://pypi.nvidia.com
python -c "from importlib.metadata import version; print(version('isaacsim'))"
python scripts/render_stereo.py --headless
```

Omniverse EULA 동의는 설치/실행자가 직접 처리합니다. 이 저장소는 자동 동의를 설정하지 않습니다.
정적 검사에 필요한 `usd-core`는 Sim 환경에 중복 설치하지 말고 별도 검사용 venv에 설치합니다.

```bash
python3 -m venv .venv-check
.venv-check/bin/pip install usd-core
.venv-check/bin/python scripts/validate_model.py
```

## 3. Isaac ROS 5.0

아래 공식 release-5.0 문서의 x86/Ubuntu 24.04 절차로 NVIDIA APT 저장소, CUDA 13.2, ROS 2 Lyrical 및 Isaac ROS CLI를 설치합니다.
기존 Humble/Jazzy/release-3.2 명령을 섞지 않습니다. Docker/venv/bare-metal 중 공식 지원 경로 하나를 선택합니다.
Docker는 GPU 전달과 `docker info` 성공을 먼저 확인합니다.

설치된 ROS 환경 안에서 패키지 존재를 확인합니다.

```bash
source /opt/ros/lyrical/setup.bash
ros2 pkg prefix isaac_ros_cuvslam
ros2 launch ./launch/cuvslam_sim.launch.py
```

release-5.0에서 **패키지명은 `isaac_ros_cuvslam`**, 저장소명은 `isaac_ros_visual_slam`입니다.
위 launch는 입력을 발행하지 않습니다. Sim의 ROS 그래프와 TF가 준비된 뒤 실행해야 합니다.
공식 예제와 비교하려면:

```bash
ros2 launch isaac_ros_cuvslam isaac_ros_visual_slam_isaac_sim.launch.py
```

## 4. Isaac Lab

Sim 환경을 보존하고 별도 checkout/venv로 구성합니다.

```bash
git clone --depth 1 --branch v3.0.0-EA https://github.com/isaac-sim/IsaacLab.git vendor/IsaacLab
cd vendor/IsaacLab
uv sync --extra isaacsim
uv run isaaclab --help
```

이 태그의 `pyproject.toml` 및 README가 의존성 기준입니다. 설치가 Sim 6.2나 구형 5.1로 바뀌면 중단하고 버전을 점검합니다.
이 저장소에는 로봇용 RL task를 구현하지 않았습니다. USD 로드 후 1개 환경에서 articulation, observation/action, reset, reward, 종료조건을 정의해야 합니다.
실물 안전 한계가 아직 검증되지 않았으므로 학습 결과를 실제 모터에 바로 배포하지 않습니다.

## 공식 자료

- https://docs.isaacsim.omniverse.nvidia.com/6.1.0/installation/requirements.html
- https://docs.isaacsim.omniverse.nvidia.com/6.1.0/installation/install_python.html
- https://nvidia-isaac-ros.github.io/v/release-5.0/getting_started/index.html
- https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_visual_slam/tree/release-5.0
- https://github.com/isaac-sim/IsaacLab/releases/tag/v3.0.0-EA
