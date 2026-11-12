# RealSense 실시간 퍽 식별 + CenterPose

현재 기본 가중치는 카메라·조명·박스 변형을 반영한 v11이다. 검출기만 실행하려면 `--detector-only`를 추가한다. 실물 위치 정확도와 모든 시점의 형상 구별이 검증된 상태는 아니다.

실행:

```bash
./scripts/run_realsense_perception.sh
```

USB 3로 연결한 RealSense의 RGB/Depth 640×480 스트림(USB3:30fps, USB2:15fps 기본값)을 연다.
장치가 없으면 기본120초 동안 기다린다. 장치 지정은 `--serial SERIAL`, 대기 연장은 `--wait-seconds 600`.
GUI에서 Q/Esc로 종료, S로 현재 오버레이와 추론 JSON을 저장한다.
다른 프로그램이 동일 카메라 스트림을 사용하고 있다면 먼저 해당 프로그램에서 카메라를 해제한다.

- YOLO v11: 빨강/초록 × 원기둥/직육면체, 박스, 태그 영역 검출.
- 퍽별 CenterPose v11: 3개 크롭의 중앙값으로 중심 XYZ 추정. 같은 클래스 여러 객체도 각각 처리한다.
- tag36h11 해독 ID는 해당 퍽 검출 박스 안에서 읽힌 표식만 표시한다. 해독 실패 시 ID를 추측하지 않는다.
- 카메라의 실제 내부 파라미터를 읽고 RGB 및 정렬 깊이를 같은 좌표계로 왜곡 보정한다.
- 지원 RGB 왜곡 모델은 none/Brown-Conrady 및 왜곡 계수가 모두0인 보정된 스트림(D435 포함). Inverse Brown-Conrady는 SDK 투영으로 보정 맵을 만들고 역투영 왕복 오차0.1px 이내를 확인한다. 그 밖의 지원하지 않는 모델은 오류로 종료한다.
- XYZ는 카메라 광학 좌표: 오른쪽+X, 아래+Y, 전방+Z. GUI는mm, JSON은m.
- 퍽 치수 가정은30×30×60mm / 원기둥Ø30×60mm다. 실물 치수가 다르면 이 가정부터 수정해야 한다.
- 깊이 센서 값은 중심 광선 주변의 **표면 깊이**이며 퍽 기하학적 중심의 정답이 아니다. 두 값을 위치 오차로 보고하지 않는다.
- 회전은 검증되지 않은 PnP 후보로 JSON에만 기록하며 로봇 구동 명령은 보내지 않는다.

결과: `runs/perception/realsense-live/latest.json`, `status.json`, `camera.json`.
최신 오버레이 `latest-preview.jpg`와 보정 RGB `latest-rgb.png` 각 한 장을 계속 덮어쓴다. S 키 스냅샷 외에 영상을 누적 녹화하지 않는다.
GUI 상단 속도는 추론 처리 속도이며 센서부터 화면까지 전체 지연이 아니다.

실물 확인 순서: 퍽을 카메라 앞에 놓고 색/형상 레이블, 읽힌 태그ID, CP XYZ 표시를 확인한다.
가까이/멀리 이동할 때 Z가 일관되게 변하는지 확인하고, 정량 오차는 알려진 기준 위치로 별도 측정한다.
합성 시험 점수를 실물 정확도로 해석하지 않는다.

## 실행 환경 / 확인된 범위

`.venv-perception-live`에 pyrealsense2 2.59.1.11053을 설치했다.
학습 venv의 torch/OpenCV/Ultralytics는 `.pth`로 읽어 재사용하며 학습 환경 패키지는 변경하지 않았다.
의존 경로: `/home/seongmin/Projects/yolov8-ubuntu24-practice/.venv-yolo/lib/python3.12/site-packages`.

합성 검증 이미지로 네 종류 퍽의 검출·ID 해독·유효 중심 추정이 동시에 실행되는 smoke 검사를 통과했다.
이 검사는 실제 카메라, 실물 인식 정확도, 실시간 안정성 검증을 대신하지 않는다.
연결/스트리밍 여부는 실행 시 `status.json`에서 확인한다.

```bash
.venv-perception-live/bin/python perception/scripts/realsense_live.py \
  --smoke-data runs/perception/data-v9-accuracy --output runs/perception/realsense-smoke
```

SDK 구성은 [공식 깊이→RGB 정렬 예제](https://github.com/realsenseai/librealsense/blob/master/wrappers/python/examples/align-depth2color.py)와
[투영·내부 파라미터 문서](https://dev.realsenseai.com/docs/projection-in-realsense-sdk-2-0/)를 참고했다.

## D435 연결 점검 기록

2026-10-30 D435는 인식됐지만 USB2.1 연결에서 30/15fps 및424×240 6fps RGB 단독 수신까지 실패했다. SDK 로그는 영상 크기의32–48%만 수신한 불완전 프레임을 기록했다. 모델 호출 이전의 영상 전송 단계 문제이며 실물 인식 검증은 아직 미완료다. USB3 데이터 케이블/포트로 직접 연결 후 재확인한다. `--require-usb3 --wait-seconds 600`으로 USB3 연결 대기를 실행할 수 있다.

D455F USB3.2 연결에서는 RGB/깊이 수신 및 YOLO→CenterPose 연속 실행을 확인했다. SDK 왜곡 보정 왕복 검사192점의 최대오차는0.000074px였다. 최초 실물 화면에서는 상자 일부를 red_cylinder로 오인식했고 CP 깊이약245mm와 표면 깊이약887mm가 크게 달랐다. 실물 퍽 식별 정확도 확인은 미완료다. `DEPTH CONFLICT`는 두 깊이가100mm 넘게 차이 나는 후보의 거친 일관성 경고이며 정량 중심오차나 검증 성공 지표가 아니다.

최신 실물 점검 결과: `runs/perception/realsense-live/verification.json`. D455F의 영상 수신과 두 모델 연결 실행은 통과했지만, 실제 빨간 원기둥/초록 직육면체가 보인 장면의 미검출 및 상자 오인식을 관찰하여 실물 인식 성능은 통과로 판단하지 않았다. 원본 프레임의640/800/1280/1600 입력해상도 비교는 퍽이 없는 장면이므로 퍽 재현율 평가가 아니다. GUI는 계속 실행한다.
