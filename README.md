# TurtleBot 기반 스워브 로봇 — 개발과 강의 실습

Isaac Sim **6.1.0.0**, Isaac ROS **release-5.0**, Isaac Lab **3.0 계열 소스 스냅샷(78b12ae)** 기준.
Jetson이나 실제 카메라 없이 Ubuntu PC에서 시뮬레이션을 준비하기 위한 URDF·USD와 가이드입니다.
**제공 미로를 Isaac Sim PhysX로 구성하고 물리 주행, Isaac Lab PPO 학습·정책 평가를 실행했습니다. 결과는 실행별 보고서로 구분합니다. 최종 v5 정책 + 측정 센서 + RGBD cuVSLAM + EKF·AMCL 폐루프로 두 미로의 지정 경로를 완주했습니다. 최신 판정과 재현 명령은 docs/VALIDATION.md를 따릅니다.**

## 수정 예정: 고정 센서 기반 주행과 팔 카메라 역할 분리 (2026-10-27)

**아래 항목은 앞으로 수정·검증해야 할 내용이며, 구현 또는 학습 완료를 의미하지 않습니다.**
위의 지정 경로 완주 결과는 기존 구성에 대한 결과입니다. 현재 실험 중인 nvblox·Nav2 재계획, 직진 차체 정렬, 연속 조향 구성은 전체 미로 완주와 속도 향상을 아직 검증하지 못했습니다. 상세 결과는 [nvblox 개발 현황](docs/NVBLOX_PROGRESS.md)을 참고합니다.

현재 팔 카메라가 진행 방향을 관측해야 주행할 수 있는 구조에서는 팔 이동 시간과 영상 지연이 주행 속도를 제한합니다. 추가로 사용 가능한 LDS-03 4대 중 **2대를 대각선 배치하고 RealSense D435 2대**를 활용해, 기본 주행 관측과 팔의 추가 관측을 분리하는 방향으로 수정합니다.

| 구성 | 수정 방향 | 남은 확인 사항 |
|---|---|---|
| LDS-03 2대 | 차체 대각선 모서리에 고정 장착, 주변 벽·장애물을 Nav2 장애물 레이어에 전달 | 실제 장착 위치·높이, 자기 가림, 유효 관측 영역, 센서별 시각·TF |
| D435 2대 | 전방·후방 고정 장착을 우선 검토하고 cuVSLAM·nvblox에 연결 | 장착 좌표, 카메라 보정, 다중 카메라 동기화·USB 대역폭·이미터 설정 |
| OpenCR IMU 2개·엔코더 | 차체 상태 추정에 계속 사용 | 카메라와의 시간 동기, 장착 좌표 및 실물 오차 보정. D435에는 내장 IMU가 없음 |
| 팔 카메라 | 고정 센서의 사각 영역과 필요한 목표를 추가 관측 | 관절 한계·자기충돌·가림을 반영한 관측 자세와 이동 궤적 |
| 주행 제어 | Nav2 MPPI 전방향 모델을 기준 제어기로 비교하고 스워브 제어에 연결 | 직선 정렬, 코너 주행, 가감속·제동거리, 실제 접촉 및 완주 시간 |
| 강화학습 | 속도 조절과 팔의 관측 위치·시점 선택을 최적화 | 고정 센서 구성에 맞춘 관측·보상 재설계, 재학습 및 실제 영상 기반 평가 |

### 라이다 2대 대각선 배치 우선안

**우선 구성은 LDS-03 2대 + 고정 D435 2대 + 팔 카메라 1대**다. 라이다는 앞왼쪽·뒤오른쪽 또는 앞오른쪽·뒤왼쪽 모서리에 배치한다. 남은 라이다 2대는 사각지대 검증 결과 추가 센서가 필요한 경우에만 검토한다.

각 라이다의 360° 스캔 사양이 차체 주변의 빈틈없는 관측을 보장하지는 않는다. 두 센서가 차체·바퀴·팔에 가리는 영역을 서로 보완하도록 실제 외형과 스캔 높이를 기준으로 배치한다. 낮은 장애물과 스캔 평면 밖 돌출물은 카메라 관측도 함께 검토하며, 미관측 공간을 자유 공간으로 처리하지 않는다.

- [ ] Isaac Sim에서 두 라이다의 실제 장착 위치·높이와 차체·바퀴·팔 자기 가림을 반영한다.
- [ ] 팔 자세별로 두 스캔의 합집합과 공통 사각지대를 확인한다. 특히 차체에 가까운 장애물과 측면·후방 이동 방향을 검사한다.
- [ ] 두 독립 LaserScan을 각각의 TF·타임스탬프로 Nav2 장애물 레이어에 반영한다.
- [ ] 미로 직선·코너·측면 이동에서 관측 신선도, 제동 여유, 접촉, 완주 시간을 검증한 후 2대 구성을 확정한다. 아직 360° 무사각 관측이나 전체 주행 성공을 검증한 것은 아니다.

라이다가 2대로 줄면 허브 데이터 포트는 라이다 2개 + OpenCR 2개로 **최소 4개**가 필요하다. 외부 전원 OpenCR은 무전원 허브를 우선 사용하되, 라이다 USB 급전과 허브 자체 소비까지 포함한 전류 예산을 확인한다. 라이다 수 감소만으로 무전원 허브 적합 판정을 내리지 않는다.

### 작업 항목

- [ ] URDF·USD에 고정 라이다와 D435 장착 프레임을 추가하고 Isaac Sim 센서를 구성한다. 실물 장착 치수 확인 전에는 가정값임을 표시한다.
- [ ] 라이다별 독립 토픽·TF·타임스탬프를 적용한다. 서로 다른 위치의 거리값을 단순 평균하거나, 2대의 10 Hz 스캔을 20 Hz 관측으로 취급하지 않는다.
- [ ] D435 스테레오 입력은 cuVSLAM, 깊이 입력은 nvblox에 연결하고 다중 카메라 입력을 검증한다. 고정 센서가 정상이어도 측면·돌출물의 미관측 영역을 자유 공간으로 간주하지 않는다.
- [ ] 기본 주행이 팔 카메라의 정렬 완료를 기다리는 조건을 제거한다. 고정 센서의 관측 범위·신선도와 제동 가능 거리를 기준으로 주행 허용 여부를 판단한다.
- [ ] 팔의 목표 자세 이동은 역기구학·관절 궤적 제어로 정확하게 수행하고, 학습 정책은 관측할 위치와 시점 선택부터 비교한다.
- [ ] 직선에서 차체 전방과 진행 방향을 정렬하고, 코너에서는 연속 스워브 조향을 활용한다. 기어 적용 후 최대 주행속도 **0.95 m/s**를 유지한다.
- [ ] 기존 팔 카메라 의존 구성과 새 구성을 같은 미로에서 비교한다. 완주 시간, 정지 시간, 위치 오차, 추적 실패, 비바퀴 접촉, 팔·차체 동시 동작 비율을 기록한다.
- [ ] 검증된 구성과 실패·개선 사례를 **별도의 후속 교안**에 반영한다. 기존 교안은 덮어쓰지 않는다.

팔의 높이·기울기까지 최적화하는 정책은 아직 학습하지 않았습니다. 현재 체크포인트의 출력은 가감속과 팔 yaw 속도이며, 영상 품질 지표를 기록하는 기능과 정책 학습 완료는 구분합니다.

### Jetson Orin Nano Super 8GB 적용 검토 (2026-10-27)

**판정: 센서 연결과 경량화한 구성은 구현 가능한 방향이다. 라이다 2대 + 카메라 3대 + cuVSLAM·nvblox·Nav2의 동시 실시간 성능은 실기 미검증이다.** 카메라는 고정 D435 2대와 기존 팔 카메라 1대이며, 팔 카메라의 실물 모델·출력 프로파일은 아직 확인하지 않았다. IMU 2개와 구동·조향 엔코더도 포함한다.

- 개발자 키트는 USB 3.2 Type-A 4포트와 8GB 메모리를 제공한다. 카메라를 USB 3에 연결하고 외부 전원 OpenCR에는 무전원 허브를 우선 검토하고, 라이다 전원 조건에 따라 허브 구성을 결정한다. 포트별 연결 속도·공유 버스·전류 한계는 실제 연결 후 확인해야 한다. 25W 모듈 전력과 센서·허브를 포함한 전체 전력은 다르다.
- 대역폭 예산 예시: 세 카메라 모두 640×480, 30fps, Y8 좌우 영상 + Z16 깊이라고 가정하면 순수 영상 합계는 약 **110.6MB/s**이다. RGB8도 세 대 모두 추가하면 **193.5MB/s**다. 프로토콜·복사 오버헤드 제외 계산이며, 팔 카메라 실제 사양 및 동시 지원 프로파일을 확인해야 한다. 이 계산만으로 USB 안정성이나 처리 성능이 검증되지는 않는다.
- 공식 nvblox 성능표의 Orin Nano 결과는 **5cm voxel** 데이터셋 기준이다. 현재 프로젝트는 **1.25cm voxel**이므로 직접 비교할 수 없다. 같은 부피를 조밀하게 표현할 때 셀 수는 64배이며, 실제 희소 할당 메모리·연산량이 정확히 64배라는 뜻은 아니다. 미로 통로 정확도를 유지하면서 지도 범위와 갱신 부하를 제한해야 한다.
- 고정 D435 2대를 cuVSLAM의 기본 위치 추정에 사용한다. 팔 카메라를 고정 외부 보정값으로 같은 카메라 묶음에 추가하지 않는다. 팔 영상은 촬영 시각의 관절 상태·동적 TF와 깊이 지원 여부를 검증한 후 추가 관측에 사용한다.
- 초기 성능 시험 설정(실측값 아님): 고정 카메라 640×480·30fps 수집, nvblox 깊이 통합 10~15Hz/카메라 및 ESDF 5~10Hz 목표, 팔 추가 관측 5~10Hz 처리부터 시작한다. 지원 프로파일·이미터 동기화 방식에 따라 조정한다. 불필요한 RGB·포인트클라우드·메시 발행은 끄고, RViz·녹화·Isaac Sim/Lab 학습은 PC에서 수행한다. Jetson에는 학습된 소형 정책의 추론만 배치한다.
- 실기 합격 전 필요한 측정: 카메라 프레임 손실·입력 지연, cuVSLAM 추적 실패, 지도 갱신 지연, Nav2 제어 주기, 메모리·스왑·온도·클럭 저하, 전체 미로 접촉·완주 시간. 0.95m/s에서 100ms 지연만으로 9.5cm 이동하므로 평균 FPS만으로 최대속도 주행을 승인하지 않는다.

공식 근거: [Jetson 사양](https://developer.nvidia.com/blog/nvidia-jetson-orin-nano-developer-kit-gets-a-super-boost/), [포트·하드웨어](https://docs.nvidia.com/jetson/orin-nano-devkit/user-guide/hardware_layout.html), [nvblox 성능표](https://nvidia-isaac-ros.github.io/repositories_and_packages/isaac_ros_nvblox/index.html), [다중 RealSense cuVSLAM](https://nvidia-isaac-ros.github.io/concepts/visual_slam/cuvslam/tutorial_multi_realsense.html), [다중 RealSense nvblox](https://nvidia-isaac-ros.github.io/concepts/scene_reconstruction/nvblox/tutorials/tutorial_multi_realsense.html). 공식 다중 카메라 예제는 이 로봇의 D435 2대·가동 팔 카메라 조합에 대한 성능 보증이 아니다.

### USB 허브 제품과 연결 계획

**사용자 확정 조건: OpenCR 2대에는 별도 전원을 공급하며, 사용 가능하면 무전원 허브를 우선 적용한다.** 이전 UH720 우선 구매안은 조건부 대안으로 변경한다. 카메라는 Jetson에 직접 연결한다. OpenCR 2대의 USB 통신은 무전원 허브부터 검증한다. LDS-03 2대까지 같은 허브에 연결할지는 라이다 전원 배선 확인 후 결정한다. 아래 제품은 제조사 사양에 따른 추천이며, 이 로봇·Jetson 조합에서 연결 안정성을 실측한 제품은 아니다.

| 제품 | 확인한 사양 | 적용 |
|---|---|---|
| [TP-Link UH720](https://www.tp-link.com/us/home-networking/usb-hub/uh720/) | USB 3.0 데이터 7포트 + 충전 전용 2포트, 외부 전원. 현재 공식 페이지는 12V·3.3A 어댑터 표기 | 조건부 대안: 라이다까지 USB 전원을 사용해 무전원 허브 전류 예산이 부족할 때 적용 |
| [StarTech ST7300USB3B](https://www.startech.com/en-us/usb-hubs/st7300usb3b) | USB 3.2 Gen 1(5Gbps) 데이터 7포트, 외부 전원 어댑터 사용 | UH720 대신 선택 가능한 7포트 유전원 허브 |

```text
Jetson USB-A 1 ─ 전방 D435 (USB 3 직접 연결)
Jetson USB-A 2 ─ 후방 D435 (USB 3 직접 연결)
Jetson USB-A 3 ─ 팔 카메라 (USB 방식일 경우, 모델·소비전력 확인)
Jetson USB-A 4 ─ 데이터 4포트 이상 허브 (무전원 우선, 아래 라이다 전원 조건 충족 시)
                  ├ LDS-03 USB 인터페이스 ×2 (전원 방식 미확정)
                  ├ OpenCR USB ×2 (보드 전원은 별도 공급)
                  └ 4포트 제품은 여유 없음 / 7포트 제품은 3개 여유
```

포트 계산은 **라이다마다 USB 인터페이스 1개, OpenCR 보드 2개가 각각 USB를 사용한다는 가정**이다. 엔코더는 보드에서 읽어 전송하면 별도 USB 포트를 추가하지 않는다. 실제 IMU 배선·팔 카메라 인터페이스가 다르면 배선을 수정한다. Jetson의 USB-A 4개로 위 구성이 가능하므로 카메라용 허브를 추가 구매할 필요는 우선 없다. 서로 다른 물리 포트가 독립 USB 컨트롤러라는 뜻은 아니다.

- **무전원 적용 조건:** OpenCR 외부 전원을 켠 상태에서 USB 열거·연속 통신·재연결을 검증한다. 외부 전원 사용만으로 USB VBUS 소비가 0이라고 가정하지 않으며, 일반 데이터 케이블을 사용한다. VBUS 선을 임의로 제거하지 않는다.
- **라이다 전원은 별도 확인:** [ROBOTIS LDS-03 공식 사양](https://emanual.robotis.com/docs/en/platform/turtlebot3/appendix_lds_03/)은 5V, 240~320mA/대다. 2대면 센서만 0.48~0.64A이며 허브·USB 변환기 소비는 별도다. 두 라이다까지 USB 급전하는 구성을 일반 무전원 허브로 확정하지 않는다. 라이다를 별도로 급전하려면 USB2LDS 전원 회로·역급전 방지·공통 접지를 먼저 확인한다. OpenCR의 외부 전원이 USB 허브까지 공급되는 구조로 해석하지 않는다.
- OpenCR 외부 전원과 USB 동시 사용은 [ROBOTIS 연결 안내](https://emanual.robotis.com/docs/en/platform/openmanipulator_x/quick_start_guide/)에 제시되어 있다. 단, 해당 안내는 이 로봇의 허브와 전체 센서 동시 동작을 검증한 자료는 아니다.
- UH720의 **충전 전용 2포트는 데이터 연결에 사용할 수 없다**. 데이터 9포트 제품으로 계산하지 않는다.
- UH720은 하드웨어 버전에 따라 어댑터 표기가 다를 수 있다. 구매한 제품의 라벨·매뉴얼을 기준으로 전압·극성·플러그·필요 전류를 맞춘다. 초기 검증은 동봉 어댑터로 수행한다. 로봇 배터리에서는 적합한 안정화 DC-DC 전원을 구성하며 배터리 원전압을 바로 연결하지 않는다.
- 카메라 3대를 하나의 허브에 몰아넣거나 허브를 연쇄 연결하지 않는 구성으로 시작한다. 허브의 5Gbps는 업스트림 공유 속도이며 포트마다 독립 5Gbps를 보장하지 않는다.
- 허브 전원 용량만으로 센서 동시 구동을 보증하지 않는다. 포트별 전류와 전체 부하, Jetson 직결 카메라의 전류 예산도 확인한다. 모터 전원과 USB 센서 전원의 순간 전압 강하를 함께 측정한다.
- 장착 후 `lsusb -t`로 D435가 SuperSpeed(5000M 이상)로 연결되는지 확인한다. 모든 센서를 켜고 모터·팔을 움직이며 USB 재연결, 카메라 프레임 누락, 라이다 수신 끊김을 기록한다. 장치 이름은 USB 연결 순서 대신 시리얼 번호·udev 규칙으로 고정한다.

## 파일

| 파일 | 내용 |
|---|---|
| `assets/m38_arm_description/urdf/m38_arm.urdf` | 차체·4개 스워브·5축 팔·가상 카메라 프레임 포함 |
| `assets/m38_arm_description/usd/m38_arm.usdc` | 원본 팔 포함 로봇 USD, 13개 가동 관절 |
| `assets/m38_stereo.usda` | 로봇 참조 + 렌더링용 스테레오 카메라 2개 |
| `assets/m38_arm_description/meshes/` | URDF 시각화 메시, 모델과 함께 유지 |
| `assets/m38_arm_description/config/model_parameters.json` | 원본 모델의 질량·관절·좌표 정의 |
| `scripts/validate_model.py` | URDF·USD 경로, 트리, 카메라 축과 기준선 정적 검사 |
| `scripts/render_stereo.py` | Sim 6.1 전용 스테레오 렌더링 시험 |
| `launch/cuvslam_sim.launch.py` | Isaac ROS 5.0용 스테레오 입력 설정 초안 |
| `docs/UBUNTU_SETUP.md` | 설치·실행·버전 가이드 |
| `docs/CAMERA_AND_TF.md` | 카메라·토픽·TF·시뮬레이션 연결 규약 |
| `docs/VALIDATION.md` | 검증 순서와 현재 한계 |

## 시작

1. [Ubuntu 설치 가이드](docs/UBUNTU_SETUP.md)대로 런타임을 준비합니다.
2. 저장소 전체를 복제합니다. URDF 파일 하나만 복사하면 메시가 누락됩니다.
3. `python scripts/validate_model.py`로 정적 모델을 검사합니다.
4. Sim 6.1 Python 환경에서 `python scripts/render_stereo.py --headless`를 실행합니다.
5. [카메라/TF 규약](docs/CAMERA_AND_TF.md)에 맞는 ROS 그래프를 만든 뒤 cuVSLAM을 실행합니다.

## 모델 제한

- CAD136/M38 내보내기를 기준으로 합니다. 이후 체결 수정 CAD142 등의 전체 형상이 자동 반영된 모델은 아닙니다.
- 총질량 4 kg과 관성은 가정값이며 실측 물성으로 승인되지 않았습니다.
- 그리퍼는 잠금 상태이고, 원본의 내부 충돌 비활성화가 유지됩니다.
- 팔 시험 범위는 임시 범위입니다. 실물 기계 한계·배선·자기충돌을 보증하지 않습니다.
- 카메라 위치·내부 파라미터는 **시뮬레이션용 가정**입니다. 실물 D435 보정값이나 CAD 정렬 완료를 뜻하지 않습니다.
- Sim/ROS 센서·명령 전체 연결과 이동 cuVSLAM 정확도는 별도 검증이 필요합니다. Lab PPO 학습·평가는 `rl/`에 구현했습니다.

CAD 유래 메시의 제3자 부품 형상은 원 제공자의 권리가 적용될 수 있습니다. 이 저장소는 비공개 검토용이며 임의의 오픈소스 라이선스를 부여하지 않았습니다.

## 개발 실습 01 (2026-10-26)

URDF의 서로 다른 바퀴 축을 반영한 스워브 역기구학과 ROS 제어 노드를 추가했습니다.

```bash
source /opt/ros/lyrical/setup.bash
colcon build --base-paths . --packages-select turtlebot_swerve
source install/setup.bash
python3 -m unittest discover -s tests -v
ROS_DOMAIN_ID=87 python3 scripts/check_ros_controller.py
ros2 launch turtlebot_swerve swerve_control.launch.py
```

- [팀별 로봇 공통 실습 가이드](teaching/TEAM_LAB.md)
- [M38 구현·시연 가이드](teaching/LAB_GUIDE.md)
- [실행 증거와 캡처](evidence/2026-10-26/README.md)
- [최신 Google Slides — 145장](https://docs.google.com/presentation/d/19UfjZpjlP2RR7iLeboQTa_h0JFBVk2P_-Yz-u_tQ-eA/edit)
- [최신 슬라이드 PDF](teaching/output/team-maze-145.pdf): 정적 열람용. 영상은 Google Slides에서 재생합니다.
- 기존 `teaching/output/팀별로봇_미로_RL_통합실습.pptx`는 78장 이전 버전입니다. 최신 145장 PPTX 내보내기는 연결 도구 403 및 브라우저 시간 초과로 완료되지 않았습니다.

팔 카메라 정렬 정책은 yaw 1축/128조건 평가를 마쳤다. 실제 Sim 영상→cuVSLAM→동시각 팔 FK 환산도 네 방향 짧은 이동에서 검증했다. 원본 무늬에서는 추적 상태 성공과 달리 위치 오차가 크게 발생했고, 합성 바닥 무늬를 추가한 비교 조건에서 오차가 줄었다. 전체 미로의 센서 기반 자율주행 성공과 구분한다. 기존 정지→시야 확보 게이트는 비교 기준이며 새 목표는 이동 중 경로를 예측하는 가감속+팔 yaw 공동 PPO이다.

새 정책: `rl/predictive_env.py`, `rl/train_predictive.py`, `rl/evaluate_predictive.py`, `rl/run_predictive_async.py`.
입력24개(경로·속도·시야·차체 기울기/각속도), 출력2개(가속도·팔 yaw 속도).
학습 거리 입력은 FOV 제한 CAD ray 모형이고 실제 영상 특징점 정책은 아니다.
중간 전체경로 평가에서 기울기 종료가 발생했으므로 완주 완료로 표시하지 않는다.
최대속도는 **기어 적용 후 0.95 m/s**; 48T→12T 속도4배, 바퀴30.9575 rad/s, 모터출력축7.7394 rad/s.
모터토크1Nm·효율1은 시뮬레이션 가정이며 실물 연속정격이 아니다.

## 두 OpenCR IMU

공식 ROBOTIS cIMU(MPU9250/ICM20648) 기반 USB 펌웨어와 ROS2 드라이버:

- `firmware/opencr_dual_imu/opencr_dual_imu.ino`: 공식 core1.5.3 컴파일 통과. 기존 보드에 플래시하지 않음.
- `turtlebot_swerve/opencr_imu_driver.py`: 보드별 `/opencr_1/imu`, `/opencr_2/imu` 발행.
- `turtlebot_swerve/dual_imu_node.py`: 시각 보간·축 변환·lever arm 보정 후 `/imu/base/data_raw` 평균.
- `config/dual_imu.yaml`: 위치·방향·바이어스·불일치 한계. 실물 장착 TF 확인 전 geometry_verified=false.
- [데이터 규약·실행법·공식 근거](docs/CAMERA_AND_TF.md): 가속도±2g, 자이로±2000deg/s, WXYZ→XYZW, 중력 포함.

단위/물리/통신검사29개 통과. 실제 ROS2와 두 PTY USB 에뮬레이터 연결에서도 평균·CRC오류·오래된 데이터 차단을 검증했다.
실물 센서 수신이나 실물 시각 동기 정확도 검증은 아직 아니다. 독립 AHRS yaw를 평균하지 않으며, 출력 orientation은 미제공(-1)이다.
차체 IMU를 움직이는 팔 카메라 cuVSLAM의 고정 IMU처럼 연결하지 않는다.
