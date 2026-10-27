> 2026-10-26 최종 통합 업데이트: 아래 기초 실습과 과거 실험은 단계별 기록입니다. 최신 v5·이중 IMU·엔코더·RGBD cuVSLAM·EKF·AMCL 실행은 `scripts/run_predictive_async.sh evidence/repeat-first`이며, 두 번째 구역은 `ROUTE_FILE=assets/maze/route_clearance_second.json`을 앞에 붙입니다. 최종 판정은 `docs/VALIDATION.md`, 54장 최신 교안은 `teaching/output/google-slides.json`의 Google Slides를 사용합니다. 기존 로컬 PPTX는 이전 버전이며 이번 native PPTX 내보내기는 403으로 완료되지 않았습니다.

# 팀별 로봇 시뮬레이션 개발 실습 01

## 수업 전제

각 팀이 **자기 로봇의 URDF와 USD, 연결된 메시 파일**을 가지고 시작합니다. 시연은 M38로 과정과 실패 분석을 시연합니다. M38의 관절명, 치수, 구동 방식, 카메라 배치를 학생 로봇의 공통 규격으로 사용하지 않습니다.

수업의 공통 부분은 설치, 모델 검증, ROS 인터페이스, 실행 증거와 Git 기록입니다. 기구학·제어·센서 설정은 각 팀의 구조와 목표에 맞게 작성합니다. 제공 저장소의 `turtlebot_swerve`는 **시연용 스워브 구현 예제**이며 모든 팀에 그대로 사용할 수 있는 범용 제어기가 아닙니다.

## 1. 환경 설치와 확인

공통 실습 기준은 Ubuntu 24.04, Python 3.12, ROS 2 Lyrical, Isaac Sim 6.1.0.0입니다. GPU와 디스크 공간, 드라이버는 팀별 PC에서 확인합니다.

```bash
cat /etc/os-release
python3 --version
nvidia-smi
df -h .
source /opt/ros/lyrical/setup.bash
ros2 pkg prefix rclpy
```

Sim을 다른 Python 프로젝트와 분리합니다.

```bash
python3 -m venv ~/isaacsim6_venv
source ~/isaacsim6_venv/bin/activate
python -m pip install --upgrade pip
python -m pip install --no-cache-dir 'isaacsim[all,extscache]==6.1.0.0' \
  --extra-index-url https://pypi.nvidia.com
python -c 'from importlib.metadata import version; print(version("isaacsim"))'
```

대용량 설치에는 다운로드 임시파일과 압축 해제 공간이 동시에 필요합니다. 부족하면 설치 전에 충분한 공간의 경로를 `TMPDIR`로 지정합니다. 이미 사용하는 환경을 임의로 삭제하지 않습니다.

ROS 개발 도구가 없다면:

```bash
sudo apt install python3-colcon-common-extensions ros-lyrical-rclpy \
  ros-lyrical-sensor-msgs ros-lyrical-geometry-msgs
```

cuVSLAM을 선택한 팀만:

```bash
sudo apt install ros-lyrical-isaac-ros-cuvslam
source /opt/ros/lyrical/setup.bash
ros2 pkg prefix isaac_ros_cuvslam
```

Sim 6.1 공식 bridge 권장 배포판은 Humble/Jazzy입니다. Lyrical native 로딩은 실험적 경로이므로 별도의 센서·명령 왕복 검증이 필요합니다. Lyrical 제어 노드가 실행됐다는 사실만으로 Sim bridge까지 검증했다고 판단하지 않습니다.

공식 설치 근거:
- https://docs.isaacsim.omniverse.nvidia.com/6.1.0/installation/install_python.html
- https://docs.isaacsim.omniverse.nvidia.com/6.1.0/installation/install_ros.html

## 2. 팀별 파일과 Git 시작

권장 구조(이름은 팀에서 정함):

```text
team_robot_ws/src/team_robot/
  assets/<robot_name>/urdf/
  assets/<robot_name>/usd/
  assets/<robot_name>/meshes/
  config/
  <python_package>/
  launch/
  tests/
  docs/
  evidence/
```

모델 파일 한 개만 복사하지 않습니다. URDF의 mesh 경로와 USD의 reference/payload/texture를 함께 보존합니다.

```bash
git status --short
git switch -c practice/team-model
git add assets docs
git diff --cached
git commit -m "feat: add team robot model and assumptions"
```

새 저장소라면 먼저 `git init`을 실행합니다. 작성자 설정은 본인 정보로 저장소 로컬에 설정합니다. CAD 공급자의 배포 범위를 확인하고 원격 저장소의 접근 범위를 정합니다.

## 3. 팀 모델 정의표

다음 항목은 팀의 **실제 모델에서 조사**해 `docs/robot_contract.md`에 기록합니다.

| 항목 | 팀이 기록할 내용 |
|---|---|
| 파일 | URDF, USD, 메시 루트, CAD/모델 revision |
| 단위 | URDF m/rad, USD metersPerUnit/upAxis 및 변환 |
| 루트 | root prim과 root link, 월드 기준 초기 pose |
| 관절 | 이름, 종류, parent/child, 원점, 축, 영점 |
| 제한 | 위치/속도/힘·토크 한계, 실측/가정 구분 |
| 물리 | 질량, 관성, 충돌, 마찰, 자기충돌 |
| 구동 | 차동/스워브/팔/기타, 제어 입력과 출력 |
| 센서 | 종류, 해상도/주기, frame, 내부·외부 파라미터 |
| 검증 | 모델 로드/제어/센서/물리/추정 단계별 상태 |

## 4. 구조와 원리 확인

URDF에서 링크 트리와 관절 축을 그립니다. USD에서 같은 부품을 찾아 transform, articulation, rigid body, joint와 drive를 확인합니다. 두 파일의 관절 이름이 다르면 매핑표를 작성합니다.

링크가 보이는 것과 물리적으로 연결된 것은 다릅니다. 충돌 모양, 질량·관성, 고정/가동 관절을 별도로 점검합니다. 센서 optical frame을 사용하는 경우 몸체 축과 광학 축 변환을 설명합니다.

**첫 캡처:** 자기 로봇 전체 모습, 트리 또는 계층 화면, 대표 관절의 축·제한 설정. 캡처마다 원본 파일과 버전을 적습니다.

## 5. 로봇별 제어 코드 작성

1. 모델 고유 정보(관절명, 치수, 부호, 제한값)를 config로 분리합니다.
2. 입력 명령과 출력 관절 목표의 단위를 정의합니다.
3. ROS와 무관한 계산 함수를 먼저 구현합니다.
4. 영점, 양/음 방향, 제한값, 정지 명령과 잘못된 입력을 검사합니다.
5. 그다음 ROS 입력과 시뮬레이터 관절 적용 코드를 연결합니다.

차동구동 팀은 바퀴 반지름과 윤거, 스워브 팀은 모듈 위치와 조향축/회전축, 팔 팀은 관절 목표와 궤적·제한을 중심으로 시험합니다. 필요한 기구학이 서로 다릅니다.

시연 M38 예제에서 `load_modules()`는 `lh1/rh1/lh2/rh2` 명명 규칙, z 조향축, 수평 wheel axis, ±π/2 조향 범위와 cylinder 반지름에 의존합니다. 다른 URDF에 이 파일 경로만 바꿔 실행해서는 안 됩니다. 해당 팀의 조인트 transform과 구조에 맞게 파서·계산·테스트를 수정해야 합니다.

## 6. ROS와 시뮬레이터 연결

실측 상태와 명령은 서로 다른 토픽으로 설계합니다. 이름으로 관절을 매핑하며 명령 배열 순서와 USD joint 순서가 같다고 가정하지 않습니다. 단위 변환(rad/deg), 영점 및 부호를 확인합니다.

시뮬레이션을 사용할 때 /clock과 use_sim_time을 정하고, 센서별 stamp를 동일한 시뮬레이션 스텝에서 발행합니다. TF의 각 변환에는 하나의 발행 주체를 정합니다.

```bash
source /opt/ros/lyrical/setup.bash
colcon build --packages-select <팀_패키지>
source install/setup.bash
ros2 launch <팀_패키지> <팀_launch.py>
ros2 node list
ros2 topic list -t
ros2 topic hz <측정_토픽>
```

꺾쇠 이름은 실제 팀 이름으로 교체합니다. 시연용 실행 명령은 `LAB_GUIDE.md`에 있습니다.

명령 만료, 측정값 만료, 제어 노드 종료를 각각 시험합니다. 수신 어댑터에도 watchdog을 둡니다. M38 예제에서 채택한 0.5초/0.05 rad는 시연값이며 팀의 제어 주기와 기계 조건에 맞게 정해야 합니다.

## 7. 센서와 SLAM 선택

영상 기반 팀은 좌우 동기화, CameraInfo, optical TF와 이미지 품질을 확인합니다. LiDAR 기반 팀은 LaserScan의 frame, 거리·각도 범위, 시각, odom 관계를 확인합니다. 팔 등 이동 위치 추정이 목표가 아닌 팀은 과제에 필요한 센서·인식 기능을 정의합니다.

cuVSLAM 초기화는 2D 점유격자 지도나 Nav2 장애물 회피의 완료를 의미하지 않습니다. 프로젝트 목표와 모듈6의 공통 학습 목표를 연결하되, 입력과 평가 지표를 구체적으로 정합니다.

## 8. 작동 사진·움짤과 Git 증거

각 팀 로봇으로 다음 순서의 PNG와 GIF를 만듭니다.

1. 모델 로드와 대표 관절 검사
2. 단일 관절 또는 단일 모듈 동작
3. 로봇 구조에 맞는 통합 동작
4. 센서 발행과 TF 확인
5. 프로젝트 목표 동작 및 실패 사례

M38 전체 렌더와 스테레오 GIF는 **강사 예시**입니다. 자기 팀의 실제 동작 증거로 제출하지 않습니다. 정적인 두 센서 영상을 교대로 표시한 GIF와 시간에 따른 동작 GIF를 구분합니다.

실행 증거에는 Git commit, 미커밋 변경, 명령, 시각, 버전, 조건, 종료 코드와 판정을 남깁니다. 제공 `scripts/record_command.py`는 이런 로그 기록에 재사용할 수 있습니다. 같은 출력 폴더를 재사용하면 기록을 덮어쓰므로 실행별 새 이름을 사용합니다.

```bash
python3 scripts/record_command.py --output evidence/team-run-001 -- <검증_명령>
git diff --stat
git diff --check
git add <검증한_파일>
git diff --cached
git commit -m "test: verify team robot command and feedback"
```

## 9. 제출 기준

설치법, 파일 구조, 모델/제어/센서 원리, 코드 작성 설명, Git 이력, 재현 명령, 자기 로봇 캡처, 검증 결과, 실패 조건을 제출합니다. 가정 물성과 미완료 항목을 표시합니다. 다른 팀이 자료만 보고 같은 결과를 재현할 수 있어야 합니다.

## 공통 도구: URDF 구조 조사

시연용 저장소의 `scripts/inspect_urdf.py`는 M38 관절명을 가정하지 않습니다.

```bash
python3 scripts/inspect_urdf.py --urdf <팀_모델.urdf> \
  --output evidence/team-urdf.json
```

`package://` 경로가 있으면 실제 패키지 폴더를 지정합니다.

```bash
python3 scripts/inspect_urdf.py --urdf <팀_모델.urdf> \
  --package <패키지명>=<패키지_절대경로> --output evidence/team-urdf.json
```

중복 이름, parent-child 연결, 루트, 순환 참조, 메시 존재 여부를 검사하고 관절 정보를 나열합니다. PASS_STRUCTURE_ONLY는 축의 물리적 정합성, 관성, USD 동등성 또는 실제 구동 합격을 뜻하지 않습니다.

## 미로 강화학습 후속 실습

전체 학습 과정과 명령, 튜닝 팁은 [RL_WORKSHOP.md](RL_WORKSHOP.md), 실제 성공·실패 판정은 [검증표](../docs/VALIDATION.md)에 있다. PPO 학습과 미로 물리 주행을 실행했지만 저장 정책의 미로 완주는 아직 실패 상태다. 비교 제어기의 완주와 구분한다.
