"""Measured yaw policy results and live integration. No Git instruction content."""
CASES=[]
def add(key,title,rows,notes):
 CASES.append(dict(key=key,title=title,rows=[['항목 / 상황','실제 설정 / 결과','조정·구현 방법','해석 / 주의점']]+rows,notes=notes))
add('range','팔 회전 범위: 근거와 적용 범위',[
 ['기존 ±0.35 rad','약 ±20° 시험 창','기계 한계로 해석 금지','CAD 검증용 임시값'],
 ['팔 기저 yaw','공식 OMX: ±180°','시연 자산 joint1 적용','배선 한계는 별도'],
 ['나머지 팔 4축','이번 학습에서는 0 고정','사용자 정의 wrist 포함','순정 범위 복사 금지'],
 ['CAD yaw 검사','361자세 / 검출 간섭 0','베이스↔팔 link2~5','배선·동적 충돌 제외']
], '공식 ROBOTIS OpenMANIPULATOR-X URDF commit ee9a720bcc45acda9c0df8fe689b1182844307db. https://raw.githubusercontent.com/ROBOTIS-GIT/open_manipulator/main/open_manipulator_description/urdf/open_manipulator_x/open_manipulator_x.urdf ; 로컬 evidence/2026-10-26/arm-range-audit/range_provenance.json 및 yaw_mesh_screen.json. M38은 사용자 정의 wrist가 추가된 모델이며 다른 축의 부호·길이가 순정과 다르다. yaw 샘플 검사 최소거리37.9mm, 인접 base/link1 및 배선 제외. 실제 하드웨어 ±180도 운용을 인증한 결과가 아니다.')
add('setup','카메라 정책: 실제 학습 구성',[
 ['관측 6개','목표 q·현재 q·q̇','오차·이전 행동·남은 시간','이미지 입력 없음'],
 ['행동 1개','팔 yaw 목표 증분','0.5 × a × dt / 제한','차체·다른 팔 축 고정'],
 ['환경 / 시간','16환경 / 15 Hz 정책','물리 60 Hz / 18초 종료','유한 horizon 관측'],
 ['학습량','160회 / 327,680 step','seed 7 / 약 354.5초','단일 seed 결과']
], '실행 rl/train_camera.py --iterations160 --num-envs16 --output runs/camera_ppo_v1. 관측은 목표/π, q/π, qdot/0.5, 목표오차/π, 이전action, 남은시간비율. 시뮬 actuator effort2Nm, velocity0.5rad/s,Kp150,Kd8; 하드웨어 연속정격값이 아니다. stochastic Normal policy의 mean을 평가에 사용한다. 한정된 yaw 정렬 문제이며 visual feature quality와 전체 팔 자세를 학습하지 않았다.')
add('ppo_actual','PPO: 실제 사용값과 조정 기준',[
 ['학습률 / clip','3e−4 / 0.2','진동·KL 증가면 lr↓','다른 항은 고정 비교'],
 ['gamma / lambda','0.99 / 0.95','긴 대기면 .995 비교','이번 결과는 .99'],
 ['rollout / batch','128×16 = 2,048','minibatch256 / epoch4','업데이트당 32 batch'],
 ['손실 / 신경망','value0.5 / entropy0.005','64–64 Tanh / grad clip1','std는 학습됨']
], '근거 runs/camera_ppo_v1/train_camera.py.snapshot, camera_policy.py.snapshot, metadata.json. entropy coefficient0.005, initial logstd−0.7, Adam3e−4. 이번 학습값과 다음 실험 제안을 구분한다. 시간제한은 남은 시간이 관측되는 유한 horizon task의 terminal로 처리했다. PPO loss 수식은 본문 기존 렌더링 수식을 참조한다. 여러 seed와 ablation을 실행한 결과가 아니다.')
add('reward_tune','학습 보상: 증상별 조정 사례',[
 ['정렬 속도가 느림','오차감소 계수 3','3→5 단독 실험','과도응답·시간 비교'],
 ['정렬 후 잔진동','Δaction 비용 0.005','0.005→0.02 비교','정착 시간·오차'],
 ['성공 직전 멈춤','성공 보상 +5','판정 dwell·속도 점검','보상만 키우지 않기'],
 ['큰 각도 timeout','episode 18초','거리/0.5 + 여유 계산','가동범위·도달성 먼저']
], '실제로 학습한 계수는 표시된 기본값뿐이다. 화살표 뒤 값은 후속 비교 실험 제안이며 효과 검증 결과가 아니다. 실제 reward:3*(|e_prev|-|e|)+0.2exp(-10e²)dt−0.01a²dt−0.005(a−a_prev)²−0.02dt+5success. 정렬 유지 보상은 이 유한 시간 정렬 전용 환경의 shaping이며 주행 환경에 그대로 지속 지급하면 정지 악용 가능성이 있다. 통합 주행에서는 이 정책의 가중치를 동결하고 gate가 차체를 통제한다.')
add('eval','별도 평가: 128개 시작·목표 조건',[
 ['전방','12 / 12 성공','초기 각도 −2.8~2.8 rad','학습 후 결정론 평가'],
 ['좌·우','각각 12 / 12 성공','목표 ±π/2','차체 고정 조건'],
 ['후방 양쪽 접근','24 / 24 성공','목표 ±3.05 rad','경계 두 방향 분리'],
 ['무작위','68 / 68 성공','평가 seed 12345','전체 평균 정착6.15초']
], 'evidence/2026-10-26/camera-eval-v1/report.json. 성공:18초 내 |yaw오차|<0.12rad, |yaw속도|<0.07rad/s를0.5초 연속 유지. 총128/128, 최대 최종오차2.221도. 미사용 랜덤 샘플의 단일 평가이며 일반적 안전성/모든 로봇/실제SLAM 성공률을 뜻하지 않는다. 평가 상태전이는 실제 Isaac Lab PhysX로 계산했다.')
add('wiring','실제 연동: 영상·추적·차체 변환',[
 ['Isaac Lab','PhysX 관절·몸체 자세','학습된 yaw 정책 실행','차체는 비교 제어기'],
 ['Isaac Sim GUI','같은 시각 좌우 렌더','640×480 / 50mm 기준선','fx=fy=400 px'],
 ['cuVSLAM','camera_rig 기준 추정','image_qos=DEFAULT','영상 stamp별 성공 확인'],
 ['ROS 어댑터','동시각 FK로 차체 환산','odom→base 단독 발행','twist·cov 복사 안 함']
], '실행 scripts/run_active_camera_demo.sh. scripts/active_camera_renderer.py, active_camera_bridge.py, launch/cuvslam_arm_camera.launch.py. 동기식 co-simulation이며 실시간속도를 보장하지 않는다. Kit은 바로 전달된 PhysX 자세를 렌더하고 cuVSLAM 결과를 받은 후 다음 물리스텝으로 진행한다. 녹화된 과거 자세를 입력한 재생이 아니다. 리그 장착값은 CAD 근사이며 실물 캘리브레이션을 대신하지 않는다. 차체 이동 명령은 simulator pose 기반 비례제어기이고 Nav2 폐루프 실행이나 차체PPO 성공 결과가 아니다.')
add('debug','연동에서 발견한 문제와 수정',[
 ['영상 프레임 누락','Best Effort에서 관측','Reliable QoS로 변경','stamp별 수신 검사'],
 ['초기 추적 없음','LOOK 진입도 차단됨','정지 후 팔 탐색은 허용','주행은 계속 금지'],
 ['후방 각도 경계','+π↔−π 목표 반복','회전 가지 연속성 유지','미세 yaw 변화 회귀검사'],
 ['팔 이동 중 DRIVE','흔들림에 추적 위험','팔 속도 초과면 차단','정착 후 다시 dwell']
], '실제 실행 로그 active-integration, active-integration-v2, 최종v3와 구분. v2는 추적1200/1200이지만 후방각도경계진동으로4방향완료실패. 이를 최종 성공 결과에서 제외한다. tests/test_active_view.py10개, test_pose_math.py2개, test_camera_heading.py3개. Reliable은 이 로컬 동기식 영상 파이프라인의 누락을 해결한 설정이며 모든 실시간 네트워크에서 지연을 개선한다는 뜻이 아니다.')
add('reproduce','실행 순서와 평가 파일',[
 ['1. 정책 학습','rl/train_camera.py','160회 / 16환경','policy.pt·training.json'],
 ['2. 분리 평가','rl/evaluate_camera.py','128조건 / 18초씩','report.json·poses.npz'],
 ['3. 실제 영상 연동','run_active_camera_demo.sh','Sim·ROS·Lab 자동 시작','steps.jsonl·report.json'],
 ['4. 결과 해석','추적률·주행 차단·오차','구간별·실패별로 비교','성공률만 보고 끝내지 않기']
], '명령과 파일 역할은 README의 카메라정책실행 절 참조. ROS_DOMAIN_ID를 다른 ROS세션과 분리한다. 단일 환경 영상만 하나의 cuVSLAM에 연결한다. 16개 병렬학습의 영상을 하나의tracker로 섞지 않는다. 실행환경 IsaacSim6.1, IsaacLab27.0.2, ROSLyrical, cuVSLAM15.0 라이브러리. 카메라정책학습과자료반영의재현을위한내용이며하드웨어배포승인아님.')
