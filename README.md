# TurtleBot 기반 스워브 로봇 — 개발과 강의 실습

Isaac Sim **6.1.0.0**, Isaac ROS **release-5.0**, Isaac Lab **v3.0.0-EA** 기준.
Jetson이나 실제 카메라 없이 Ubuntu PC에서 시뮬레이션을 준비하기 위한 URDF·USD와 가이드입니다.
**Ubuntu에서 정적 렌더, 제어 통신, 저장 영상 cuVSLAM 초기화까지 검증했습니다. 물리 주행과 실시간 SLAM은 아직 미완료입니다.**

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
- Sim/ROS 전체 연결, cuVSLAM 정확도, Lab 3 학습은 아직 검증되지 않았습니다.

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
- [강사용 M38 구현·시연 가이드](teaching/LAB_GUIDE.md)
- [실행 증거와 캡처](evidence/2026-10-26/README.md)
- [강의 슬라이드](teaching/output/팀별로봇_개발실습01.pptx)

측정 joint_states가 없거나 오래되면 바퀴 명령은 0입니다. 현재 출력은 명령 토픽이며 물리 어댑터와 cuVSLAM 연결은 후속 구현입니다.
