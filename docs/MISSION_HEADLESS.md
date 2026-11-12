# 주사위 기억 미션: headless 개발

최신 전체 검증: [통과 항목과 도달성 실패 근거](MISSION_FULL_VALIDATION.md).

2026-10-30. 기준 자료: Downloads/mission.pdf v5, maze.zip,
R12_NO_USB_URDF_USD.zip. 원본 SHA-256은 config/mission/sources.json에 기록한다.
문서의 경기 규칙을 구현 대상으로 사용하며 문서의 제출·실기 지시를 자동 실행하지 않는다.

## 실행

저장소 루트에서 실행한다. 원본 ZIP은 Downloads에 보관한다.

```bash
python3 scripts/prepare_mission_assets.py
python3 -m unittest discover -s tests -p test_dice_mission.py -v
# 실행자가 NVIDIA Omniverse EULA에 동의한 경우에만 환경변수를 설정한다.
export OMNI_KIT_ACCEPT_EULA=YES
/home/seongmin/isaacsim6_venv/bin/python scripts/mission_headless.py
```

GUI를 열지 않는다. 결과는 runs/mission_headless/scene.usda,
allocation.json, report.json에 저장한다. 기본 물리 실행은 120Hz × 1200 step(10초)이다.
오류는 0이 아닌 종료코드를 반환한다. PASS_PHYSICS_SMOKE는 정지 상태의
13개 관절 로봇 위치 유한성·이동량·10도 이내 기울기, 마지막 1초 1mm 미만 흔들림, 퍽 지지 높이·5mm 미만 수평 이탈 검사만 의미한다.
자기충돌, 접촉력, 주행 안정성이나 미션 완주를 의미하지 않는다.

## 장면과 가정

- 사용자 선택: 제공 미로를 유지하고 PDF 좌표에 작업대를 추가한다.
- 미로의 왼쪽 아래 기둥 collision/geo_3 중심을 원점으로 평행 이동한다.
  벽 간격·회전·크기는 바꾸지 않는다. 원본 미로는 PDF 도면과 정확히 일치하지 않는다.
- 시작 박스 6개, L1/L3 박스 2개, 중앙 410mm 박스 4개를 만든다.
- 중앙 받침대 중심 (2.202, 3.497)m는 PDF 그림에서 추정했다.
  명시된 L2 격자 중심 (2.202, 3.700)m와 구분한다.
- config/mission/practice.json은 Q1 신호와 합성 퍽 목록이다.
  공식 placement.csv가 제공되지 않아 공식 ID/모양 목록을 재현했다고 주장하지 않는다.
- 퍽 질량 30g, 로봇 원본 질량·관성, 마찰은 시뮬레이션 가정이다.
- 로봇은 출발 중심 (.734,.950)m, -X 방향에 배치한다.
- USD의 그리퍼 개폐 모델과 팔 끝 보정 카메라가 없다.
  퍽 순간이동 또는 목표 좌표 복사로 집기/재검증 성공을 만들지 않는다.

## 구현된 규칙

12회 R/Y/G 기록, 각 1~6회, 3초 종료 소등, 반복색 사이 소등 구분,
불명색·관측 공백·초과/부족 오류 정지. SignalRecorder 입력은 분류 결과이며
실제 RGB 분류기는 아직 없다. max_gap 기본값은 0.25초이다.

4종 바코드 정렬 및 R→Y→G 동률 순서, Y 동일 모양과 R/G 색상 조건을
동시에 만족하는 전체 할당, 12개 ID 중복/누락 검사, +Y 위쪽 기준
주사위 격자를 구현했다. allocation.json은 계획이며 센서 관측 증거가 아니다.

## 완료 판정에 남은 관문

1. headless PhysX 정지·관절 응답·오프스크린 렌더링 통과. 아래 실행 증거 참조.
2. 작업대와 제공 미로 벽의 간섭 검사 및 R12 접근 가능 경로 생성.
3. 렌더링 카메라에서 신호·Code128·퍽 속성 관측, ROS 시각/TF 연결.
4. R12 관절 기반 이동과 팔 IK, 그리퍼 충돌·개폐 모델 및 접촉 집기.
5. 장소별 10mm 이내, 2초 안정, 추가 퍽 없음 판정.
6. 이동 후 새 카메라 관측으로 장소별 ID·좌표 검증 및 중복/누락 검사.
7. 출발 구역의 정확한 경계 정의 후 차체 전체 포함·3초 정지 판정.
8. 전 과정 로그·영상과 100점 채점기, 실패 조건 재현.

## 2026-10-30 실행 결과

사용자가 EULA 동의 후 GUI 없이 실제 Isaac Sim 6.1을 실행했다.
원본 USD의 기존 translate와 충돌하던 장면 조립을 상위 Xform으로 분리했다.
Kit 종료 시 예외가 종료코드 0으로 가려지던 문제도 수정했다.

```bash
OMNI_KIT_ACCEPT_EULA=YES /home/seongmin/isaacsim6_venv/bin/python scripts/mission_headless.py --render --output evidence/2026-10-30/mission-headless
OMNI_KIT_ACCEPT_EULA=YES /home/seongmin/isaacsim6_venv/bin/python scripts/mission_headless.py --actuators --output evidence/2026-10-30/mission-actuators
```

- 정지 10초: PASS_PHYSICS_SMOKE, 충돌 prim 586개, 회전 관절 13개.
- 시작 직후 중력 정착을 포함한 차체 이동 13.36mm. 마지막 1초 위치 변화 0mm.
- 퍽 12개 중심 Z=100mm 유지, 최대 수평 이동 0.001364mm.
- overview.png와 robot_front.png: 실제 640×480 RTX 렌더링, 빈 영상 검사 통과.
- 작은 구동 입력 1초 + 영점 명령 1초: 13개 관절 모두 양의 응답 확인.
  팔·조향 목표 0.04rad에 측정 변화 0.03998~0.04093rad,
  바퀴 목표 0.5rad/s에 측정 회전 0.4553~0.4790rad.
- Python unittest 발견 테스트 39개 통과. pytest 전체를 실행했다는 뜻은 아니다.

증거: evidence/2026-10-30/mission-headless 및 mission-actuators의 report.json,
scene.usda, allocation.json, runtime.log. source-manifest.json으로 코드 해시를 기록했다.

이 결과는 초기 장면과 작은 관절 구동의 시뮬레이션 검사다. 전체 미션 성공은
아직 아니다. 신호등·바코드 장면과 영상 인식, 그리퍼 개폐/접촉 집기,
센서 폐루프 주행, 배치 재인식·복귀는 남아 있다. robot_front.png는 로봇의
첫 번째 상단 카메라 영상이며 팔 카메라 보정이나 신호 판독 성공을 뜻하지 않는다.
