# 가상 카메라와 ROS 연결 규약

## 좌표와 내부 파라미터

`base_link`: ROS +X 전방, +Y 좌측, +Z 상방.
광학 프레임: +X 영상 우측, +Y 영상 아래, +Z 전방.
URDF fixed joint RPY는 (-π/2, 0, -π/2).

| 프레임 | base_link 기준 위치(m) |
|---|---|
| sim_left_optical | (0.25, +0.025, 0.25) |
| sim_right_optical | (0.25, -0.025, 0.25) |

기준선 0.05 m. 640×480, fx=fy=400, cx=320, cy=240, D=0, R=identity.
K=[400,0,320; 0,400,240; 0,0,1].
왼쪽 P=[400,0,320,0; 0,400,240,0; 0,0,1,0].
오른쪽 P=[400,0,320,-20; 0,400,240,0; 0,0,1,0].
실제 렌더러가 주점/픽셀 중심을 다르게 보고하면 CameraInfo와 렌더러를 같은 값으로 맞춘 뒤 재검사한다.
이 값은 D435 실측 보정값이 아니다. 실물 이전 때 RealSense CameraInfo와 외부보정을 사용한다.

## 필요한 토픽

| 토픽 | 타입 | frame_id |
|---|---|---|
| /front_stereo_camera/left/image_rect_color | sensor_msgs/Image | sim_left_optical |
| /front_stereo_camera/right/image_rect_color | sensor_msgs/Image | sim_right_optical |
| /front_stereo_camera/left/camera_info | sensor_msgs/CameraInfo | sim_left_optical |
| /front_stereo_camera/right/camera_info | sensor_msgs/CameraInfo | sim_right_optical |
| /clock | rosgraph_msgs/Clock | 해당 없음 |
| /joint_states | sensor_msgs/JointState | 해당 없음 |
| /tf, /tf_static | tf2_msgs/TFMessage | 연결 트리 |

RGB8 또는 지원되는 mono8 영상을 발행한다. PNG 렌더링 파일만 저장하는 현재 스크립트는 ROS 발행기가 아니다.
좌우 Image와 CameraInfo는 같은 시뮬레이션 스텝의 timestamp를 사용한다. 우선 30 Hz, QoS 호환, use_sim_time=true로 시험한다.

## Sim 6.1에서 구현할 그래프

ROS 2 확장을 활성화하고 6.1 버전의 ROS sensor/graph 문서를 따른다. 구형 5.1 노드 이름을 그대로 붙이지 않는다.
각 USD Camera에 RenderProduct 생성 → 이미지 및 CameraInfo 발행 → /clock 발행 → 로봇 관절 상태 발행 순으로 연결한다.
현재 저장소의 render_stereo.py는 렌더링 단독 점검용이며, 그래프 저장본은 아직 포함하지 않았다.

`robot_state_publisher`에 URDF와 joint_states를 제공해 base_link 아래 TF를 만든다.
Sim ground truth는 `sim_world` 또는 별도 토픽으로 기록한다. cuVSLAM의 map→odom→base_link와 같은 TF를 중복 발행하지 않는다.
static TF를 다른 노드로 발행했다면 robot_state_publisher의 같은 변환은 중복 발행하지 않는다.

## 수신 점검

```bash
ros2 topic hz /front_stereo_camera/left/image_rect_color
ros2 topic hz /front_stereo_camera/right/image_rect_color
ros2 topic echo /front_stereo_camera/right/camera_info --once
ros2 topic echo /clock --once
ros2 run tf2_ros tf2_echo base_link sim_left_optical
ros2 node info /visual_slam_node
```

cuVSLAM은 ground truth를 입력으로 쓰지 않는다. 추정 결과와 ground truth를 별도 저장해 정확도를 비교한다.
