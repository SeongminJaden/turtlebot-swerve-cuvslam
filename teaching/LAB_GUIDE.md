# M38 스워브 로봇 개발 실습 01

모듈6 양식에 맞춘 프로젝트 보충 실습입니다. 원본 37~45강의 시수와 성취도를 대체하지 않습니다.
학습 목표: URDF의 바퀴 축을 해석하고 몸체 속도를 관절 명령으로 바꾸며, Git 이력과 실행 증거로 결과를 설명합니다.

## 1. 설치와 환경 확인

현재 검증 호스트: Ubuntu 24.04.5, ROS 2 Lyrical, NVIDIA 595.91.07, RTX 5060 Laptop 8GB.
`nvidia-smi`의 CUDA 13.2는 드라이버 지원 수준이며 CUDA Toolkit 설치 여부와 다릅니다.
Isaac Sim 6.1.0.0으로 스테레오 렌더를 실행했습니다. 이후 기존 venv가 외부에서 사라져 재실행 경로를 확인해야 합니다.

```bash
cat /etc/os-release
nvidia-smi
df -h .
source /opt/ros/lyrical/setup.bash
ros2 pkg prefix rclpy
ros2 pkg prefix isaac_ros_cuvslam
```

cuVSLAM 패키지 설치와 노드 로드를 확인했습니다. 저장 영상 재생으로 tracker 초기화도 확인했지만, 실제 이동 추적은 아직 검증하지 않았습니다. 설치판과 요구 사항은 `docs/UBUNTU_SETUP.md`의 공식 링크를 확인합니다. 기존 Sim 환경에 별도 usd-core를 덮어 설치하지 않습니다.

프로젝트 제어 노드 의존성 설치(시스템에 없을 때):

```bash
sudo apt install python3-colcon-common-extensions ros-lyrical-rclpy \
  ros-lyrical-geometry-msgs ros-lyrical-sensor-msgs ros-lyrical-ament-index-python
```

## 2. Git으로 실습 시작

저장소 루트에서 실행합니다. 이미 작업 브랜치에 있다면 새 브랜치를 또 만들 필요는 없습니다.

```bash
git status --short
git log --oneline -5
git switch -c practice/swerve-control
python3 -m unittest discover -s tests -v
git diff --stat
git diff -- turtlebot_swerve/kinematics.py
git add turtlebot_swerve/kinematics.py tests/test_kinematics.py
git diff --cached
git commit -m "feat: explain and verify wheel axis conversion"
```

변경을 비교할 때 `git show <commit>`, `git diff HEAD~1 HEAD`를 사용합니다. 실패한 구현을 보존한 채 수정 커밋을 남깁니다. 공유 이력 취소는 `git revert <commit>`으로 새 커밋을 만들고, 실습 중 `reset --hard`로 결과를 지우지 않습니다. push는 원격과 공개 범위를 확인한 뒤 별도로 수행합니다.

## 3. 프로젝트 구조

- `assets/`: CAD 유래 URDF, USD, 메시와 가정 물성
- `turtlebot_swerve/kinematics.py`: ROS 의존성이 없는 역기구학
- `turtlebot_swerve/controller.py`: cmd_vel 입력, 측정 joint_states, 명령 출력
- `launch/swerve_control.launch.py`: 제어 노드 실행
- `launch/cuvslam_sim.launch.py`: 미검증 cuVSLAM 연결 초안
- `tests/`: 속도 복원, 축 부호, 포화와 정지 검증
- `scripts/`: ROS 검사, 렌더링, 실행 기록
- `evidence/`: 실제 로그와 렌더 이미지

## 4. 원리와 코드 작성 순서

ROS 몸체 좌표는 x 전방, y 좌측, z 위입니다. 몸체 명령을 `(vx, vy, wz)`라 하고, i번 조향 중심의 좌표를 `(xi, yi)`라 하면:

```
ux_i = vx - wz * yi
uy_i = vy + wz * xi
speed_i = sqrt(ux_i² + uy_i²)
heading_i = atan2(uy_i, ux_i)
```

바퀴 반지름은 URDF collision cylinder에서 읽은 0.03068719889482 m입니다. 실제 타이어 유효 반지름의 측정값은 아닙니다. 조향축과 바퀴 접촉점 사이의 0.25 mm 오프셋, 타이어 변형과 미끄러짐은 이 평면 모델에서 무시합니다.

양의 바퀴 회전이 만드는 진행 방향은 `axis × +Z`입니다. 앞 두 바퀴 축 +X는 -Y 진행, 뒤 왼쪽 +Y는 +X 진행, 뒤 오른쪽 -Y는 -X 진행입니다. 따라서 기준 진행각 β를 구하고 `steering = heading - β`로 변환합니다. ±90° 밖이면 조향각에 π를 더하거나 빼고 바퀴 속도 부호를 뒤집습니다.

1. `Module`에 위치, 반지름, 기준 진행각, 최대 속도를 저장합니다.
2. `load_modules()`에서 URDF를 파싱합니다. 축과 반지름의 유효성을 검사합니다.
3. `solve()`에서 각 모듈의 목표를 계산합니다.
4. 어느 바퀴든 10 rad/s를 넘으면 모든 바퀴를 같은 비율로 줄입니다.
5. 정지할 때 조향각은 유지하고 바퀴 속도만 0으로 만듭니다.
6. 테스트에서 바퀴 목표를 다시 x/y 속도로 바꿔 원래 명령과 비교합니다.

## 5. 빌드와 ROS 실행

다음 명령은 저장소 루트에 독립적인 build/install을 생성합니다.

```bash
source /opt/ros/lyrical/setup.bash
colcon list --base-paths .
colcon build --base-paths . --packages-select turtlebot_swerve
source install/setup.bash
ros2 launch turtlebot_swerve swerve_control.launch.py
```

다른 터미널에서 같은 setup 두 개를 source한 뒤:

```bash
ros2 topic echo /swerve/joint_commands
ros2 topic pub --rate 10 /cmd_vel geometry_msgs/msg/Twist '{linear: {x: 0.1}}'
```

측정 `/joint_states`가 없으면 바퀴 명령은 0입니다. 이는 정상입니다. 이 단계에는 물리 시뮬레이터 구동 어댑터가 연결되지 않았습니다. 출력 `/swerve/joint_commands`를 `/joint_states`로 이름만 바꿔 측정값인 것처럼 사용하지 않습니다.

JointState 명령은 조향 position 배열과 바퀴 velocity 배열을 **별도 메시지**로 발행합니다. 수신 어댑터는 name으로 해당 관절을 찾아 적용해야 합니다. 조향 속도는 1 rad/s로 제한하며, 측정 조향 오차가 모든 바퀴에서 0.05 rad 미만일 때 회전을 허용합니다. 명령 또는 피드백이 0.5초 이상 끊기면 바퀴를 정지합니다. 명령 노드가 죽는 경우에는 수신 어댑터에도 독립 watchdog이 필요합니다.

## 6. 테스트와 캡처

```bash
python3 scripts/record_command.py --output evidence/my-run/unit -- \
  python3 -m unittest discover -s tests -v
ROS_DOMAIN_ID=87 python3 scripts/record_command.py --output evidence/my-run/ros -- \
  python3 scripts/check_ros_controller.py
```

기록 파일은 명령, 작업 디렉터리, UTC 시각, 종료 코드, Git 커밋/작업 상태, 로그 SHA256을 포함합니다. 테스트는 합성 피드백으로 ROS 발행/구독과 정지를 검사하며 실제 주행 성공을 의미하지 않습니다.

Sim 환경 복원 후 해당 Python으로 실행:

```bash
python scripts/render_stereo.py --headless
python scripts/capture_model.py
```

현재 확보된 `model-overview.png`는 USD 모델의 실제 렌더 캡처입니다. `sim61_left.png`, `sim61_right.png`는 각 센서의 실제 렌더 결과입니다. 좌우 비교 GIF는 같은 정적 장면의 두 카메라 영상을 번갈아 보여줍니다. 주행 동영상이나 SLAM 성공 증거가 아닙니다. 모델 공전 GIF 생성은 환경 소실로 완료되지 않았습니다.

## 7. 스테레오와 SLAM 데이터 흐름

기준선 B=0.05 m, fx=400 px인 가상 카메라에서 이상적인 깊이는 `Z = fx * B / disparity`입니다. 예를 들어 시차 20 px는 1 m를 뜻합니다. 실제 렌더에서 해당 숫자를 측정해 검증한 결과는 아닙니다.

카메라 Image와 CameraInfo, /clock, 로봇 TF가 cuVSLAM 입력입니다. 좌우 timestamp를 일치시키고 optical frame 축과 TF를 점검합니다. `map→odom→base_link` 추정 TF와 ground truth TF를 중복 발행하지 않습니다.

모듈6의 slam_toolbox/AMCL은 2D LaserScan/점유격자 실습입니다. cuVSLAM의 영상 기반 위치 추정을 완료해도 점유격자 지도, Nav2 costmap, 장애물 회피가 자동 완성되지는 않습니다.

## 8. 다음 구현과 완료 기준

1. Sim 실행 경로 복구 및 공식 지원 ROS bridge 확인
2. 조향 position/바퀴 velocity 명령을 articulation에 적용, 실제 joint_states 발행
3. 마찰·충돌·중력 하에서 정지, 직진, 횡이동, 회전 캡처
4. 좌우 영상/CameraInfo/clock/TF 통합 및 cuVSLAM 실행
5. ground truth와 추정 궤적 시간 정렬 후 ATE/RPE, tracking loss 측정
6. 별도 LaserScan/지도 입력을 설계하고 Nav2 목표 이동과 장애물 회피 검증

각 단계에는 재현 명령, 버전, 성공/실패 로그, 화면 PNG, 짧은 GIF, Git 커밋을 붙입니다. 현재 물리 주행, SLAM, Nav2는 미완료입니다.
