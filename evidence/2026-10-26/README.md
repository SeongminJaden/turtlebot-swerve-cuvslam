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

현재 자료만으로 물리 주행이나 cuVSLAM 추적 성공을 주장할 수 없습니다.
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
