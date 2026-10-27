# TurtleBot 기반 스워브 로봇 — 개발과 강의 실습

Isaac Sim **6.1.0.0**, Isaac ROS **release-5.0**, Isaac Lab **3.0 계열 소스 스냅샷(78b12ae)** 기준.
Jetson이나 실제 카메라 없이 Ubuntu PC에서 시뮬레이션을 준비하기 위한 URDF·USD와 가이드입니다.
**제공 미로를 Isaac Sim PhysX로 구성하고 물리 주행, Isaac Lab PPO 학습·정책 평가를 실행했습니다. 결과는 실행별 보고서로 구분합니다. 최종 v5 정책 + 측정 센서 + RGBD cuVSLAM + EKF·AMCL 폐루프로 두 미로의 지정 경로를 완주했습니다. 최신 판정과 재현 명령은 docs/VALIDATION.md를 따릅니다.**

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
