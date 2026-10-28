"""Native editable appendix tables; measured results are populated after validation."""
CASES=[
 dict(key='sensor_clock',title='시뮬 센서: 같은 시각으로 연결',rows=[
 ['경로','입력과 출력','주기 / 프레임','설정 이유'],
 ['두 OpenCR IMU','PhysX → ADC 양자화 → ROS','200 Hz / 각 보드 frame','레버암 가속도까지 포함'],
 ['엔코더 13축','unwrap → count → 차분 속도','200 Hz / 30 ms 차분창','래핑 오류·차분 진동 제거'],
 ['팔 스테레오','RGB·깊이·CameraInfo·FK','영상 촬영 시각 기준','최신 팔 각도와 섞지 않기'],
 ['비동기 실행','CPU 물리 / GPU 영상 / ROS','/clock 발행자는 하나','늦은 영상은 새 값이 아님']]),
 dict(key='sensor_fusion',title='EKF 설정: 스워브와 움직이는 카메라',rows=[
 ['측정','융합할 값','사용하지 않을 값','이유'],
 ['바퀴 odometry','차체 vx, vy','적분 위치 중복 융합','횡방향 주행이 가능한 로봇'],
 ['이중 IMU 평균','보정된 yaw rate','독립 AHRS yaw의 평균','공통 좌표·시각에서 비교'],
 ['cuVSLAM + 팔 FK','body 위치 차분 → 속도','VO yaw를 차체 yaw로 강제','팔 움직임과 차체 이동 분리'],
 ['평면 좌표계','첫 body pose 기준 정규화','pitch만 숫자 0으로 변경','기울어진 축은 회전 후 투영']]),
 dict(key='sensor_amcl',title='Nav2 위치 보정: 지도·팔 깊이·EKF',rows=[
 ['설정','현재 값 / 구성','변경 사례','확인할 결과'],
 ['운동 모델','nav2_amcl::OmniMotionModel','차동 모델 사용하지 않기','옆 이동 후 위치 편향'],
 ['레이저 입력','/arm_depth/scan · 1° bin','깊이 범위 0.23–5 m','미관측 방향은 infinity'],
 ['입자·갱신','500–1500 / 이동 0.01 m','입자 수 ↑ / 갱신 거리 ↑','위치 오차와 CPU 부하'],
 ['좌표계 연결','map→odom→base→camera','TF 발행자 중복 제거','영상 시각 TF 조회 성공']]),
 dict(key='sensor_dynamics',title='동역학: 속도만 맞추면 끝나지 않는다',rows=[
 ['항목','발견한 문제','현재 처리','실물 확인'],
 ['바퀴 관성','COM이 회전축 밖에 있었음','동심 실린더 overlay','실측 질량·관성으로 대체'],
 ['기어·속도','기어 전후 단위 혼동','차체 최대 0.95 m/s','출력축 RPM·바퀴 반경'],
 ['급제동','속도 서보 정지 시 차체 기울기','바퀴 명령 변화율 제한','정지거리·전류·미끄럼'],
 ['코너 속도','직선 속도로 조향하면 이탈','곡률·조향속도·제동 제한','완주시간과 차체 접촉']]),
 dict(key='sensor_limits',title='완료 판정: 성공 로그와 실제 검증 구분',rows=[
 ['항목','판정 기준','측정 방법','주의'],
 ['도착','실제 오차 < 0.10 m','평가용 실제 pose와 비교','추정 pose 도착만으론 부족'],
 ['속도','실제 ≤ 0.951 m/s','200 Hz 물리 스텝 peak','명령 상한과 구분'],
 ['접촉','비바퀴 힘 ≤ 0.5 N','native ContactSensor','벽·지면 접촉 모두 검사'],
 ['적용 범위','두 닫힌 미로 / 합성 텍스처','실제 영상 cuVSLAM 폐루프','실물 무오류 보장은 아님']]),
]
NOTES='Isaac Sim 6.1 / Isaac Lab 27.0.2. 이 페이지는 현재 로컬 구현을 설명하며 완주 검증 결과는 별도의 결과 표를 따른다. OpenCR 공식 MCU driver ±2g/±2000dps. 엔코더 4096 count/rev, 질량/토크와 nominal IMU 장착 축은 실물 계측 전 가정이다. cuVSLAM은 VO 모드이고 팔 카메라에는 body IMU 고정 extrinsic VIO를 켜지 않는다. AMCL은 공식 Nav2 1.5.0 OmniMotionModel, robot_localization 3.10.0을 사용한다. 관련 소스 config/ekf_swerve.yaml, config/amcl_swerve.yaml, rl/robot_sensors.py, scripts/active_camera_bridge.py. 원본 CAD는 변경하지 않고 wheel inertia overlay를 사용한다. https://emanual.robotis.com/docs/en/parts/controller/opencr10/ https://docs.nav2.org/rolling/configuration_and_development/configuration_guide/others/configuring_amcl/ https://github.com/cra-ros-pkg/robot_localization/blob/rolling-devel/doc/configuring_robot_localization.rst'

CASES_EXTRA=[
 dict(key='encoder_wrap',title='엔코더 사례: 바퀴가 갑자기 역회전?',rows=[
 ['항목','잘못된 처리','현재 구현','튜닝할 값'],
 ['관절 각도 래핑','원시 각도를 곧바로 차분','200 Hz에서 연속각 복원','최대 속도 × 주기 < π'],
 ['속도 추정','5 ms 양자화 차분만 사용','약 30 ms 차분창','10 / 30 / 50 ms 비교'],
 ['카운트·기어','바퀴 count를 모터로 오인','모터축 = 바퀴각 / 4','실물 CPR·기어비로 교체'],
 ['검증','명령 속도만 확인','native 속도·카운트 비교','래핑 때 spike가 없어야 함']]),
 dict(key='steering_guard',title='스워브 사례: 정지할 때 조향이 흔들림',rows=[
 ['증상','원인','현재 처리','판단 기준'],
 ['정지 중 모듈 회전','0 속도에서 방향 재계산','조향각 유지 + 바퀴 감속','불필요한 재정렬 감소'],
 ['한쪽 바퀴만 가속','모듈별 정렬 시간이 다름','큰 전환은 감속→정렬','차체 yaw·경로 이탈'],
 ['직진 중 180° 전환','임시 ±90° 끝단을 왕복','차체 시작 yaw −45° 비교','실물 범위는 별도 실측'],
 ['팔까지 매번 정지','주행·시야 제어를 묶음','평소 팔과 차체 동시 제어','추적 재획득 때만 함께 유지']]),
 dict(key='transfer',title='팀별 로봇: 그대로 복사하지 않을 값',rows=[
 ['설정','M38 시뮬 예시','팀별 변경','검증'],
 ['기구학','반경 0.030687 m / 기어 4','URDF 축·반경·모듈 위치','직진·횡이동·회전 시험'],
 ['동역학','질량·토크·관성은 가정 포함','실측 질량·COM·모터 한계','전류·정지거리·슬립'],
 ['카메라·IMU','가상 stereo / CAD 장착값','intrinsic·TF·시각·noise','정지·회전·가속 기록'],
 ['정책 배포','24개 관측 / 2개 행동','정규화·단위·제약 일치','실제 영상 폐루프 평가']]),
]
