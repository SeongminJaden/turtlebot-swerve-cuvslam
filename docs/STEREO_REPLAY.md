# 정적 스테레오 캡처로 ROS 연결 확인

이 검사는 저장 PNG를 반복 발행합니다. 실제 이동, 시차 정확도, SLAM 드리프트를 검증하지 않습니다.

터미널 A:

```bash
source /opt/ros/lyrical/setup.bash
export ROS_DOMAIN_ID=87
ros2 launch ./launch/cuvslam_sim.launch.py
```

터미널 B (같은 저장소 루트):

```bash
source /opt/ros/lyrical/setup.bash
export ROS_DOMAIN_ID=87
python3 scripts/replay_stereo.py --images evidence/2026-10-26 --seconds 12
```

발행 항목: 좌우 RGB8 Image, CameraInfo, /clock, base_link에서 두 optical frame으로 가는 /tf_static. 한 쌍의 Image와 CameraInfo는 같은 시각을 씁니다. 별도의 clock/TF/카메라 발행기가 없는 독립 도메인에서 실행합니다.

2026-10-26 검사에서 360쌍을 발행했고, `cuvslam-replay.log`에 cuVSLAM 15.0.0의 GPU warmup 및 tracker 초기화 성공이 기록됐습니다. 중간 프레임 간격 경고도 발생했습니다. 따라서 이 결과는 입력 연결/초기화 확인이며 30Hz 처리 성능 합격이 아닙니다. 실행 종료는 20초 timeout의 SIGINT로 수행했습니다.

실시간 연결에서는 토픽 hz, 실제 수신량, 처리 시간, drop 수, 좌우 stamp 차이를 추가 측정해야 합니다.
