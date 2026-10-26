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
