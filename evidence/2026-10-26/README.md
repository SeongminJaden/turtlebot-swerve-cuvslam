# 실행 증거 — 2026-10-26 (Asia/Seoul)

- `kinematics/`: 단위 테스트 5개, 정확한 명령과 종료 코드 기록
- `ros/`: Lyrical 로컬 통신 테스트. 합성 측정 피드백을 사용했으며 물리 주행이 아님
- `colcon-build.log`: ament_python 패키지 빌드 성공
- `stereo-render.log`, `sim61.json`: 설치돼 있던 Sim 6.1.0.0으로 렌더 실행
- `sim61_left.png`, `sim61_right.png`: 실제 좌우 카메라 렌더. 장면 대부분이 비어 있어 추적용 장면 개선이 필요
- `stereo-comparison.gif`: 위 정적 좌우 영상의 번갈아 보기. 시간에 따른 로봇 주행이 아님
- `model-overview.png`: USD 모델 전체 렌더 캡처
- `model-capture.log`: 이후 재시도에서 기존 Python 환경 경로 소실을 확인한 로그
- `isaac-install.log`: 사용자 요청에 따른 Sim 환경 재설치 로그

위 초기 자료만으로 물리 주행이나 cuVSLAM 이동 추적 성공을 주장할 수 없습니다. 후속 미로 실행 결과는 아래에 별도로 기록했습니다.
렌더링 중 RTX 아이콘 관련 오류와 플러그인 경고가 관찰됐습니다. 저장된 결과와 미완료 작업을 분리해서 판정합니다.

## 후속 확인

- `ros-final/`: 명령과 피드백 만료를 분리한 최종 통신 검사 4개
- `kinematics-final/`: 비정상 입력의 수치 overflow까지 거부하는 최종 테스트
- `cuvslam-startup.log`: GPU warmup과 노드 로드 성공
- `cuvslam-replay.log`, `stereo-replay.log`: 저장 영상 360쌍으로 tracker 초기화 성공. 프레임 지연 경고 포함
- `preflight.json`: 재설치된 Sim 6.1.0.0, GPU, ROS, cuVSLAM 패키지 확인
- `stereo-rebuild.log`: 새 환경 첫 실행의 EULA 동의 요구
- `team-urdf-example.json`: 팀 공통 구조 조사 도구를 M38에 적용한 예시

cuVSLAM launch 검사의 마지막 SIGINT/exit -2는 정해진 시간 후 시험 프로세스를 종료한 기록입니다. 이동 추적 성공을 뜻하지 않습니다.

## 미로와 Isaac Lab 실행 추가

이하 결과로 앞의 초기 준비 단계 기록을 갱신한다. 현재 판정표는 `docs/VALIDATION.md`이다.

- `lab-physics-probe-root.log`: 명시적 base_link root로 PhysX 바닥 안정 확인
- `lab-maze-smoke.log`: Lab 환경에서 실제 관절 구동과 이동 확인
- `ppo-training.log`: 최초100회 PPO 학습 완료
- `ppo-extended.log`: 단방향 목표 이어 학습.221회 체크포인트 보존 후 조건 수정을 위해 중단
- `ppo-bidirectional.log`: 양방향·시작 오차 조건으로521회까지 학습 완료
- `ppo-final.log`: 현재 조향 분기 처리를 포함해546회까지 학습 완료
- `maze-eval-100/`, `maze-eval-bidirectional-preview/`, `maze-eval-final/`: 세 정책 모두 정체 또는 시간 제한으로 완주 실패
- `maze-baseline-hysteresis/`: 비학습 비교 제어기는 첫 미로 경로를110.2초에1회 완주
- `maze-eval-100/render/`: 실제 기록 포즈의 Isaac Sim 재생 PNG·GIF. GIF는0~40초를 약7배속으로 보여줌
- `stereo-maze-render/`: 동일 미로의 실제 좌우 가상 카메라 정적 렌더
- `environment/`: Sim pip freeze와 Lab importlib.metadata 패키지 버전 목록

`capture-rebuilt.log`는 중단한 정적 모델 캡처 시도이며 성공 증거로 사용하지 않는다. 최신 슬라이드는 정상 생성된 미로 렌더를 사용한다.
