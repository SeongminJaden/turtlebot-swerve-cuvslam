# 최종 통합 검증 — 2026-10-26

최종 구성은 v5 PPO(120 iteration, sha256 bf890ae456483a5f56c4ffd200293f133b9c26505da194e80f08ef91cac5d5c2), 200 Hz CPU PhysX, GPU RGBD 영상, ROS 이중 IMU·양자화 엔코더, cuVSLAM VO, robot_localization 차분 VO+wheel vx/vy+gyro yaw rate, Nav2 AMCL이다. 실제 pose는 렌더와 평가에만 사용한다. 초기 지도 위치는 알려진 spawn이다. Nav2 AMCL을 쓰며 주행 제어는 PPO와 별도 스워브·곡률·제동 제한기의 조합이다. Nav2 전체 BT/controller stack의 검증을 뜻하지 않는다.

| 실행 | sim / wall 시간(s) | 실제 최고속도(m/s) | 실제 목표오차(m) | 비바퀴 접촉(N) |
|---|---:|---:|---:|---:|
| 첫 미로 GUI 1 | 60.45 / 60.43 | 0.91944 | 0.04135 | 0 |
| 첫 미로 GUI 2, 전체 녹화 | 63.18 / 63.17 | 0.92240 | 0.04254 | 0 |
| 두 번째 미로 | 57.395 / 57.434 | 0.82426 | 0.04431 | 0 |

모든 실행은 실제 속도 ≤0.951 m/s, 실제 목표오차 <0.10 m, 비바퀴 접촉 ≤0.5 N 기준을 통과했다. 두 닫힌 구역의 각 8.162 m 지정 경로를 검증했으며 모든 가능한 가지 탐색이나 다양한 seed 성공률을 뜻하지 않는다. 정상 완주에서 tracker reset은 0회다. 첫 미로 GUI 2의 전체 시간 중 팔·차체 동시 움직임 비율은 42.8%다(정지시간 포함). 조향 대전환이나 센서 오류 시 정지는 여전히 필요하다.

IMU2 2초 누락과 카메라 2.5초 누락 시험 모두 누락 시작 후 0.355초에 실제 속도 <0.03 m/s로 정지하고 데이터 복구 후 재개했다. 접촉 0 N. 카메라는 1회 reset 후 측정 EKF 증분으로 VO 원점을 이어 붙였다. 이는 ROS 측정 누락 주입 시험이며 실제 USB 단선 시험이 아니다. 전체 결과는 evidence/2026-10-26/predictive-final-summary.json과 각 실행 report.json에 보존한다.

## 재현

```bash
scripts/run_predictive_async.sh evidence/repeat-first
ROUTE_FILE=assets/maze/route_clearance_second.json scripts/run_predictive_async.sh evidence/repeat-second
GUI=1 scripts/run_predictive_async.sh evidence/repeat-gui
```

기본값은 최종 v5 checkpoint, RGBD tracking_mode=2, EKF·AMCL 사용이다. GUI 전체 녹화는 CAPTURE_BARRIER=1과 scripts/capture_gui_window.py를 함께 사용한다. /clock은 센서 브리지 하나만 발행한다. 평균 IMU 노드는 데이터 누락·불일치 때 임의로 정상 보드를 선택하지 않고 invalid 처리한다.

## 실물 적용 전 미확정 값

엔코더 4096 CPR, 모터 토크 1 Nm, 바퀴 질량/동심 관성, nominal IMU 축·위치, 가상 카메라 깊이/noise, 조향 ±90°는 시뮬 가정이다. 실제 로봇 실측과 재교정이 필요하다. 팔 정책은 yaw 1축이며 나머지 자세는 고정한다. 독립 OpenCR AHRS yaw는 평균하지 않는다. OpenCR firmware는 공식 core 1.5.3으로 컴파일하고 200 Hz ROS+PTY 시험을 통과했지만 실제 보드에는 플래시하지 않았다. 합성 바닥 무늬를 사용했으므로 무늬 없는 실물 바닥의 추적 안정성까지 보장하지 않는다.

주요 수정은 wheel angle unwrap, 영상 시각 FK, 초기 body 좌표 정규화, AMCL 위치 차분 대신 EKF twist 사용, 0.15초 비동기 응답 제한, 정지 중 조향 유지, 모듈 동기 정렬, 바퀴 관성 overlay다. 최신 단위 검사는 tests/에 있으며 ROS 통합 시험 scripts/test_*는 ROS 환경을 source한 후 개별 실행한다.

---

## 이전 단계 기록 (아래 미완료 상태는 당시 결과)

# 검증 상태와 실행 결과

2026-10-26, Ubuntu 24.04 / Isaac Sim6.1.0.0 / Isaac Lab 소스78b12ae / RTX5060 Laptop8GB. 사용자가 NVIDIA 약관에 동의한 뒤 실제 Sim·Lab을 실행했다. 이전 EULA 대기 상태는 종료되었다.

## 검증표

| 항목 | 실제 결과 | 증거 |
|---|---|---|
| 스워브 기구학·비정상 입력 | 단위 테스트5개 통과 | `evidence/2026-10-26/kinematics-final/` |
| ROS 제어 통신·watchdog | 합성 피드백 기반4개 통과 | `ros-final/` |
| 모델 floating base | base_link 루트 수정 후 바닥 안정 확인 | `lab-physics-probe-root.log` |
| 미로 USD | 벽80개·지지대83개, 두 미로 생성 | `assets/maze/maze.usda` |
| Lab 환경 | 관측[1,12], step·reset·실제 이동 확인 | `lab-maze-smoke.log` |
| PPO 학습 | 초기100회, 이어 학습·조건 수정 후546회 체크포인트 저장 | `ppo-*.log`, `runs/` |
| 비학습 비교 제어기 | 첫 미로 경로1회 완주,110.2초,목표오차0.118m | `maze-baseline-hysteresis/report.json` |
| PPO100회 정책 | 완주 실패,160초 후 목표오차2.525m | `maze-eval-100/report.json` |
| PPO316회 정책 | 후반 정체,73초 후 목표오차2.068m | `maze-eval-bidirectional-preview/report.json` |
| PPO546회 정책 | 중간 정체,52.47초 후 목표오차2.728m | `maze-eval-final/report.json` |
| Sim 영상·움짤 | 실제 PhysX body pose를 Sim으로 재생 렌더링 | `maze-eval-100/render/` |
| 미로 좌우 카메라 | 640×480 정적 렌더 확인 | `stereo-maze-render/stereo-*.png` |
| cuVSLAM | 기존 저장영상 tracker 초기화 확인 | `cuvslam-replay.log` |
| 이동 영상 cuVSLAM·정책 폐루프 | 미완료 | ground-truth pose를 정책에 사용 중 |

모든 증거의 기준 폴더는 `evidence/2026-10-26/`이다. 비교 제어기의 완주를 PPO 성공으로 해석하지 않는다. 위 평가는 각1회이며 여러 시작점·seed에서의 성공률을 뜻하지 않는다. 전체 시스템 PASS 상태가 아니다.

## 실제로 바꾼 것

원본 USD에서 articulation root를 base_link에 지정하는 참조 레이어를 만들었다. 원본 CAD 모델은 보존했다. 제공 미로는 원본 배치와 패널 치수를 유지하고 지지대를 보수적인 원기둥으로 단순화했다.

초기 학습은 경로 전방12점 목표만 사용했다.221회 체크포인트 이후 양방향12점과 시작점±0.05m 오차를 사용했다.521회 이후에는 조향 끝단 부근0.06rad 분기 유지가 포함된 환경에서25회를 더 학습했다. 마지막546회 정책도 완주하지 못했다. 모델·보상·경로 추종의 추가 개선이 필요하다.

각 평가의 CSV와 NPZ는 실제 PhysX 상태 기록이다. 이미지 생성 모델로 만든 주행 장면이나 임의로 그린 궤적을 검증 증거로 사용하지 않았다. GIF는 기록된 포즈의 빠른 재생이며 원래 시간은 capture.json에 있다.

## 남은 실행 검증

- PPO 정책의 여러 시작점·seed 완주율, 벽 접촉·정체 감소
- 카메라 지연과 위치추정 오차를 포함한 정책 입력
- 실시간 Sim ROS 센서·명령 왕복 및 cuVSLAM 이동 추적
- ATE/RPE와 tracking loss, 센서 timestamp·TF 오류 상황
- 다른 팀 로봇의 action 공간·운동학·물성에 맞춘 재학습

원본 질량·관성·카메라 파라미터와 관절 한계에는 시뮬레이션용 가정이 있다. 현재 결과는 실물 검증을 대신하지 않는다.
