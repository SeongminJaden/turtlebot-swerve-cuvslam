# 가상 카메라와 ROS 연결 규약

## 좌표와 내부 파라미터

`base_link`: ROS +X 전방, +Y 좌측, +Z 상방.
광학 프레임: +X 영상 우측, +Y 영상 아래, +Z 전방.
URDF fixed joint RPY는 (-π/2, 0, -π/2).

| 프레임 | base_link 기준 위치(m) |
|---|---|
| sim_left_optical | (0.25, +0.025, 0.25) |
| sim_right_optical | (0.25, -0.025, 0.25) |

기준선 0.05 m. 640×480, fx=fy=400, cx=320, cy=240, D=0, R=identity.
K=[400,0,320; 0,400,240; 0,0,1].
왼쪽 P=[400,0,320,0; 0,400,240,0; 0,0,1,0].
오른쪽 P=[400,0,320,-20; 0,400,240,0; 0,0,1,0].
실제 렌더러가 주점/픽셀 중심을 다르게 보고하면 CameraInfo와 렌더러를 같은 값으로 맞춘 뒤 재검사한다.
이 값은 D435 실측 보정값이 아니다. 실물 이전 때 RealSense CameraInfo와 외부보정을 사용한다.

## 필요한 토픽

| 토픽 | 타입 | frame_id |
|---|---|---|
| /front_stereo_camera/left/image_rect_color | sensor_msgs/Image | sim_left_optical |
| /front_stereo_camera/right/image_rect_color | sensor_msgs/Image | sim_right_optical |
| /front_stereo_camera/left/camera_info | sensor_msgs/CameraInfo | sim_left_optical |
| /front_stereo_camera/right/camera_info | sensor_msgs/CameraInfo | sim_right_optical |
| /clock | rosgraph_msgs/Clock | 해당 없음 |
| /joint_states | sensor_msgs/JointState | 해당 없음 |
| /tf, /tf_static | tf2_msgs/TFMessage | 연결 트리 |

RGB8 또는 지원되는 mono8 영상을 발행한다. PNG 렌더링 파일만 저장하는 현재 스크립트는 ROS 발행기가 아니다.
좌우 Image와 CameraInfo는 같은 시뮬레이션 스텝의 timestamp를 사용한다. 우선 30 Hz, QoS 호환, use_sim_time=true로 시험한다.

## Sim 6.1에서 구현할 그래프

ROS 2 확장을 활성화하고 6.1 버전의 ROS sensor/graph 문서를 따른다. 구형 5.1 노드 이름을 그대로 붙이지 않는다.
각 USD Camera에 RenderProduct 생성 → 이미지 및 CameraInfo 발행 → /clock 발행 → 로봇 관절 상태 발행 순으로 연결한다.
현재 저장소의 render_stereo.py는 렌더링 단독 점검용이며, 그래프 저장본은 아직 포함하지 않았다.

`robot_state_publisher`에 URDF와 joint_states를 제공해 base_link 아래 TF를 만든다.
Sim ground truth는 `sim_world` 또는 별도 토픽으로 기록한다. cuVSLAM의 map→odom→base_link와 같은 TF를 중복 발행하지 않는다.
static TF를 다른 노드로 발행했다면 robot_state_publisher의 같은 변환은 중복 발행하지 않는다.

## 수신 점검

```bash
ros2 topic hz /front_stereo_camera/left/image_rect_color
ros2 topic hz /front_stereo_camera/right/image_rect_color
ros2 topic echo /front_stereo_camera/right/camera_info --once
ros2 topic echo /clock --once
ros2 run tf2_ros tf2_echo base_link sim_left_optical
ros2 node info /visual_slam_node
```

cuVSLAM은 ground truth를 입력으로 쓰지 않는다. 추정 결과와 ground truth를 별도 저장해 정확도를 비교한다.

## 두 OpenCR IMU의 실제 데이터 경로 (2026-10-26)

공식 OpenCR `cIMU`를 재사용한 IMU 전용 USB 펌웨어와 ROS 2 드라이버를 추가했다.
기존 주행 펌웨어에 통합하려면 `firmware/opencr_dual_imu/opencr_dual_imu.ino`의 읽기/전송 부분을 병합한다.
이 스케치를 그대로 업로드하면 기존 보드 펌웨어를 대체한다. 현재 실물 포트가 없어 업로드하지 않았다.
모터를 초기화하거나 토크를 켜는 코드는 포함하지 않는다.

- 보드 1: `/opencr_1/imu`, frame `opencr_front_imu`
- 보드 2: `/opencr_2/imu`, frame `opencr_rear_imu`
- 원점 보정 후 개별 비교: `/imu/base/corrected_1`, `/imu/base/corrected_2`
- 두 센서의 동일 시각 평균: `/imu/base/data_raw`, frame `base_link`
- 유효성/불일치: `/imu/diagnostics`; 보드별 통신 상태는 `/opencr_1/diagnostics`, `/opencr_2/diagnostics`

### 공식 소스에서 확인한 규약

[OpenCR 문서](https://emanual.robotis.com/docs/en/parts/controller/opencr10/)에 따르면 이전 보드는 MPU9250,
2020년 이후 보드는 ICM-20648을 사용한다. 공식 `imu_selector`가 두 장치를 탐색한다.
OpenCR 소스 조사 commit: `68ec75d8a400949580ecf263e0105ea9743b878e`.
실제 펌웨어 빌드는 공식 Arduino OpenCR core **1.5.3**으로 통과했다.

- `MPU9250.cpp`, `ICM20648.cpp`: 가속도 ±2 g, 자이로 ±2000 deg/s.
- `Define.h`: ROLL=0, PITCH=1, YAW=2. `SEN.accADC`와 `SEN.gyroADC`의 순서는 X,Y,Z.
- 가속도: `ADC * 9.80665 / 16384`, 각속도: `ADC * 2000 / 32768 * pi / 180`.
- `cIMU.aRes`에는 8 g 환산 상수가 있으므로 이 값을 SI 전송에 사용하지 않았다.
- `imu.quat`: W,X,Y,Z. ROS 메시지는 X,Y,Z,W로 변환한다. 각 보드 AHRS의 초기 yaw 기준은 같다고 가정하지 않는다.
- 가속도에는 중력이 포함된다(specific force). 정지·수평 상태에서는 +Z 약 9.80665 m/s².
- 펌웨어 200 Hz 읽기, PC 기본 100 Hz 요청. CRC32, sequence, ADC 포화/보정 상태를 검사한다.
- 샘플 시각은 센서 읽기 완료 시각이며, PC 시각은 USB 왕복 중간값으로 환산한다.
  하드웨어 동기화가 아니다. RTT/2 불확실성을 진단하고 RTT>20 ms이면 폐기한다.

근거: [공식 센서 설정](https://github.com/ROBOTIS-GIT/OpenCR/tree/master/arduino/opencr_arduino/opencr/libraries/IMU),
[공식 TurtleBot3 SI 환산](https://github.com/ROBOTIS-GIT/OpenCR/blob/master/arduino/opencr_arduino/opencr/libraries/turtlebot3_ros2/src/turtlebot3/turtlebot3_sensor.cpp),
[REP-145](https://ros.org/reps/rep-0145.html).

### 위치·방향 보정과 평균

`R_i`는 센서축→base_link 회전, `r_i`는 base_link 원점→센서 칩 위치(m)이다.
두 IMU가 같은 차체에 단단히 고정된 조건에서 다음을 계산한다.

```
omega_i = R_i (gyro_i - gyro_bias_i)
omega = (omega_1 + omega_2) / 2
alpha = low_pass(d omega / dt)
f_i_at_base = R_i (accel_i - accel_bias_i)
              - alpha × r_i - omega × (omega × r_i)
f_base = (f_1_at_base + f_2_at_base) / 2
```

먼저 동일 시각으로 선형 보간하며, 브래킷이 없는 오래된 측정값은 외삽하지 않는다.
각속도 차이 또는 위치 보정 후 가속도 차이가 한계를 넘으면 평균을 발행하지 않는다.
센서가 두 개뿐이면 불일치만으로 고장 난 쪽을 확정할 수 없다.
평균의 공분산에는 각속도·각가속도 보정의 공유 불확실성과 센서 불일치를 포함한다.
단순 평균만으로 공통 바이어스/진동/시간 오차가 제거되지는 않는다.

출력 원점은 **base_link 좌표 원점**이다. 변화하는 전체 질량중심(COM)과 같은 뜻이 아니다.
COM 기준이 필요하면 해당 지점으로 두 lever arm을 다시 계산해야 한다.
팔에 달린 IMU에는 위의 고정 차체 식을 그대로 쓰면 안 된다(관절 상대 가속도 항 필요).

공식 REVH BOM은 U9=MPU-9250, 공식 XY 자료는 U9=QFN24 3×3, Top, 90°, PCB 좌표(52.5,37.5) mm를 명시한다.
이를 M38 CAD의 동일 QFN 부품과 대조하면 base_link 기준 칩 패키지 중심은
앞 `(0.05700028,-0.00001860,0.16597000) m`, 뒤 `(-0.05700028,0.00001860,0.16597000) m`이다.
패키지 중심은 MEMS 감지점의 근사이며 조립 오차를 포함하지 않는다.
현재 설정의 회전은 시뮬레이션 센서 프레임 규약이다. 공식 ADC 축과 **실제 조립된 보드 방향**을 연결하는
TF는 장착 형상·핀1 방향 및 축별 기울임 검사로 확정해야 하므로, 실물 설정 `geometry_verified`는 false이다.
보드 위치가 원점이라고 가정하는 우회 설정은 하지 않는다.

### 실행과 검증

```bash
source /opt/ros/lyrical/setup.bash
cd ~/ros2_ws
colcon build --packages-select turtlebot_swerve --symlink-install
source install/setup.bash
ros2 launch turtlebot_swerve opencr_dual_imu.launch.py \
  front_port:=/dev/serial/by-id/실제_앞_OpenCR \
  rear_port:=/dev/serial/by-id/실제_뒤_OpenCR
# config/dual_imu.yaml의 실측 축/위치/바이어스 확정 후 실행
ros2 launch turtlebot_swerve dual_imu.launch.py config:=/절대경로/dual_imu.yaml
```

두 보드의 포트를 ACM0/ACM1 열거 순서로 임의 배정하지 않는다.
같은 USB 포트를 기존 TurtleBot3 노드와 동시에 열 수 없다.
현재 펌웨어는 IMU-only 패킷이므로 기존 TurtleBot3 DYNAMIXEL 전송과 호환된다고 가정하지 않는다.

`/imu/base/data_raw`의 orientation_covariance[0]은 -1이다. 독립된 두 AHRS yaw를 숫자 평균하지 않았다.
기울기/자세가 필요하면 이 보정된 gyro+specific force에 **단일** AHRS(예: imu_filter_madgwick)를 적용하고,
동적 가속 구간의 중력 추정 오차를 검증한다. 현재 패키지는 해당 AHRS를 자동 설치/실행하지 않는다.
Nav2/EKF에 raw 둘과 평균을 동시에 넣으면 같은 정보를 중복 사용하므로 평균 경로 하나를 선택한다.
팔 카메라 cuVSLAM의 `enable_imu_fusion`은 계속 false: 차체 IMU와 움직이는 카메라 사이에는 고정 extrinsic이 없다.

검증: `tests/test_dual_imu.py`, `tests/test_opencr_protocol.py`; 실제 ROS 2 + 두 PTY USB 에뮬레이터:
`ROS_DOMAIN_ID=95 python3 scripts/test_dual_imu_ros.py`.
보고서 `evidence/2026-10-26/dual-imu-audit/ros-transport-test.json`:
198개 유효 평균, 최대 가속도 오차 2.05e−7 m/s², 한쪽 CRC 오류 시 출력 중단 및 stale 진단 확인.
이는 실물 센서 노이즈·시각 동기 정확도·장착 TF 검증 결과가 아니다.

### Isaac Sim 센서 구현 및 2026-10-26 검증

`rl/robot_sensors.py`는 PhysX IMU 두 개를 위의 CAD 장착 위치에 생성합니다.
센서값은 중력을 포함한 specific force이며 OpenCR의 ±2 g / ±2000 deg/s
ADC 해상도로 양자화합니다. 포화값은 평균에 넣지 않습니다.
구동 4개, 조향 4개, 팔 5개 관절은 측정 관절 위치에서 엔코더 카운트를 생성합니다.
현재 4096 count/rev는 **설정 가능한 시뮬레이션 가정**입니다.
구동축은 wheel angle / 4로 모터 출력축 카운트를 만들며, 토크/백래시/슬립과
실물 엔코더의 래핑·통신 지연은 아직 완전한 하드웨어 모델이 아닙니다.

카메라의 base→rig 변환은 URDF와 동일 시각의 팔 엔코더 FK로 계산합니다.
렌더링용 실제 링크 pose는 영상 생성과 오차 평가에 사용하고, 새 비동기 경로의
카메라 위치 보정에는 엔코더 FK를 사용합니다. 정책의 기울기 입력은 보정 IMU의
gyro/중력 상보 필터로 만듭니다. 가속 중 기울기 편향이 생길 수 있고, 이 센서 입력으로
기존 PPO 정책을 다시 학습·검증해야 합니다. 움직이는 팔과 몸체 IMU는 고정 외부파라미터
관계가 아니므로 cuVSLAM의 고정 IMU 융합을 임의로 켜지 않습니다.

`isaac_sensor_bridge.py` 출력:
- `/opencr_1/imu`, `/opencr_2/imu`: 센서 좌표계 raw SI 측정
- `/joint_states`: 양자화된 위치와 위치 차분 속도
- `/encoders/motor_counts`: JointState와 같은 순서의 모터 출력축 카운트
- `/wheel/odometry`: 스워브 기구학 오도메트리, TF 발행 안 함, 슬립 보정 아님
- `/clock`: 물리 시각의 단일 발행자. 카메라 브리지는 `--external-clock` 사용

`config/dual_imu_sim.yaml`은 시뮬레이션 전용입니다. 실물 장착 검증을 의미하지 않습니다.
현재 통합 실행은 CPU PhysX **200 Hz**, 센서 UDP **200 Hz**, 정책 약 **15.38 Hz**,
GPU 카메라 렌더링을 별도 프로세스로 실행합니다. 센서·영상은 물리 시각을 공유하고
늦은 영상이나 IMU 누락을 새 측정으로 복제하지 않습니다. CPU 물리 / GPU 렌더 분리 후
실시간 배율 약 1.00을 확인했습니다. 이 값은 해당 노트북의 측정치입니다.

### 상태 추정과 실제 검증 상태

`config/ekf_swerve.yaml`의 robot_localization은 바퀴 vx/vy, 평균 IMU yaw rate,
영상 시각 FK로 보정한 cuVSLAM body pose를 융합합니다. 스워브의 vy를 0으로 묶지 않습니다.
카메라가 아래로 기울어 있으므로 첫 body pose 역변환으로 VO 좌표계를 정규화한 후
평면 EKF에 전달합니다. 단순히 pitch/roll을 지우면 전진 거리가 잘못 투영됩니다.
AMCL은 CAD 지도와 팔 depth의 실제 끝점으로 만든 `/arm_depth/scan`을 사용합니다.
미관측 각도는 infinity이며 거리 측정으로 꾸며 넣지 않습니다. 현재 cuVSLAM은 VO 모드입니다.

PPO v5는 균형 잡힌 바퀴 관성 모델에서 120 iteration / 245,760 transition을 추가 학습했습니다.
훈련은 PhysX IMU 관측을 사용하지만 영상 가시성·위치는 분석적 대리 모델입니다.
따라서 훈련 성공 수를 실제 영상 cuVSLAM 통합 완주 성공으로 해석하지 않습니다.

원본 USD 바퀴 COM이 회전축에서 벗어나 있어 `assets/m38_balanced_wheels.usda`에
동심 실린더 관성 가정을 별도로 적용했습니다. 원본 CAD는 변경하지 않았습니다.
바퀴 질량 0.15 kg, 전체 질량 약 4 kg, 모터 토크는 실물 계측 전 가정입니다.
최대 주행속도는 기어 적용 **후 0.95 m/s**입니다.

2026-10-26 현재 통합 검증은 **미완료**입니다.
- v5 + AMCL 시험: 추정 위치로는 도착했지만 실제 목표 오차 0.140 m,
  차체 접촉 53.1 N이 있어 불합격 처리했습니다.
- VO 좌표계와 차체 footprint 수정 후 시험: 실시간 배율 1.0005,
  센서 송신 9,932회 / 송신 drop 0회. 그러나 차체 접촉과 VO jump로 중단했습니다.
- 정책이 빠르게 가속한 뒤 영상이 오래되어 속도 명령이 즉시 0이 되면서,
  속도 서보가 강하게 제동하고 차체가 기울어지는 현상을 확인했습니다.
  바퀴 명령 변화율 제한을 넣어 재검증 중입니다.

통과 기준은 추정 도착만이 아니라 실제 목표 오차 < 0.10 m,
200 Hz에서 측정한 실제 속도 ≤ 0.951 m/s, 비바퀴 접촉 ≤ 0.5 N입니다.
시뮬레이션 ground truth는 영상 생성·평가에만 사용합니다.
현재 경로는 제공된 CAD의 첫 번째 닫힌 미로이며, 두 번째 구역까지 검증했다는 뜻이 아닙니다.
합성 바닥 텍스처가 적용되어 있으므로 무늬 없는 실물 바닥에서도 동일 성능이라고 할 수 없습니다.

재현:
```bash
USE_EKF=1 USE_AMCL=1 CHECKPOINT=runs/predictive_checkpoints/v5-final.pt \
  DURATION=150 bash scripts/run_predictive_async.sh evidence/새로운-시험-폴더
```
실물에서는 장착 축, 바이어스, 엔코더 분해능, 토크·질량, 지연·시간 동기 계측이 추가로 필요합니다.
