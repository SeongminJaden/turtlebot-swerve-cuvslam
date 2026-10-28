import json
from pathlib import Path
p=json.load(open('build/lecture/native-54-black.json'))['structuredContent'];by={s['objectId']:s for s in p['slides']};req=[];keep=[];plan=[]
def rep(o,t,cell=None):
 a={'objectId':o,'textRange':{'type':'ALL'}};b={'objectId':o,'text':t,'insertionIndex':0}
 if cell:a['cellLocation']=cell;b['cellLocation']=cell
 req.extend([{'deleteText':a},{'insertText':b}])
def use(sid,title):
 keep.append(sid);plan.append({'slide':sid,'exemplar':sid,'method':'reuse copied native slide','role':title,'media':'none except explicitly retained TF equation or official replacement'})
 rep(sid+'_i3',title)
 notes=by[sid]['slideProperties']['notesPage']['notesProperties']['speakerNotesObjectId']
 rep(notes,'후속 교안: nvblox·Nav2 통합. 이전 교안과 별도 파일.\n공식 자료: https://nvidia-isaac-ros.github.io/repositories_and_packages/isaac_ros_nvblox/index.html\n구현: config/nvblox_arm.yaml, config/nav2_nvblox.yaml, scripts/nav2_policy_path.py, turtlebot_swerve/live_path.py\n설정 예시는 시뮬용이며 팀별 센서·기구 값에 맞게 변경. 검증 결과는 마지막 페이지에 별도 기록.')
def two(sid,title,lh,left,rh,right):
 use(sid,title)
 for k,t in [('lh',lh),('left',left),('rh',rh),('right',right)]:rep('compact_'+sid+'_'+k,t)
def table(sid,title,rows,caption):
 use(sid,title);o=next(e['objectId'] for e in by[sid]['pageElements'] if 'table'in e)
 assert len(rows)==next(e['table']['rows'] for e in by[sid]['pageElements'] if 'table'in e)
 for r,row in enumerate(rows):
  for c,t in enumerate(row):rep(o,t,{'rowIndex':r,'columnIndex':c})
 for e in by[sid]['pageElements']:
  if e['objectId'].endswith('_i10'):rep(e['objectId'],caption)
use('p1','팀별 로봇 자율주행 2');rep('p1_i4','nvblox · Nav2 · 예측 카메라 정책');rep('p1_i5','팀별 URDF·USD 기반 / 시연: M38 / 후속 실습')
two('p2','실습 범위와 시스템 구성','추가하는 기능','팔 깊이 영상으로 주변 3D 지도 생성\nESDF 거리 지도를 Nav2 비용지도에 반영\n목표까지 경로를 주기적으로 재계획\n\nNav2 경로를 기존 PPO 관측에 공급\n가감속·팔 yaw를 주행 중 함께 제어\n차체 최고 속도 0.95 m/s 유지','기존 구성과 연결','cuVSLAM + 엔코더 + 이중 IMU\nEKF odom + AMCL map 위치 유지\n\nCAD 지도: 전역 경로의 기반\nnvblox: 관측 장애물 추가\nPPO: 가감속·카메라 yaw\n\n미지 공간 탐험은 별도 확장 단계')
two('p3','설치 환경과 실행 패키지','공식 패키지','ROS 2 Lyrical / Isaac ROS 5.0\nnvblox_ros / nvblox_nav2\nnav2_planner / nav2_navfn_planner\nnav2_lifecycle_manager\n\nSim: RTX 깊이 영상 생성\nLab: 200 Hz PhysX·정책 실행\nGPU 메모리와 실행 지연 함께 확인','프로젝트 전용 설치','python3 scripts/setup_nvblox_runtime.py\n\nsource /opt/ros/lyrical/setup.bash\nsource scripts/robot_localization_env.sh\nsource scripts/nvblox_env.sh\n\n공식 deb를 build/nvblox-runtime에 추출\n시스템 ROS 환경과 분리하여 실행')
# Original official figure at original aspect ratio, editable adjacent explanation.
use('cuvslam_official_intro','nvblox: 깊이 영상에서 장애물 거리 지도까지')
rep('cuvslam_official_intro_head','TSDF·ESDF·Mesh 역할')
rep('cuvslam_official_intro_body','TSDF: 표면 주변의 부호 있는 거리\n영상별 측정을 가중 누적\n\nMesh: 복원한 표면 시각화\nESDF: 가까운 장애물까지 거리\n\n2D slice를 Nav2 비용으로 변환\nRGB 색상 입력은 선택 사항\n현재 실습은 깊이 입력 사용')
req.append({'deleteObject':{'objectId':'cuvslam_official_intro_figure'}})
url='https://media.githubusercontent.com/media/NVIDIA-ISAAC-ROS/.github/release-5.0/resources/isaac_ros_docs/repositories_and_packages/isaac_ros_nvblox/system_diagram_technical_details.jpg'
req.append({'createImage':{'objectId':'nvblox_official_figure','url':url,'elementProperties':{'pageObjectId':'cuvslam_official_intro','size':{'width':{'magnitude':365,'unit':'PT'},'height':{'magnitude':365*458/633,'unit':'PT'}},'transform':{'scaleX':1,'scaleY':1,'translateX':45,'translateY':105,'unit':'PT'}}}})
rep('cuvslam_official_intro_source','NVIDIA 공식 구조도 · nvblox Technical Details')
req.append({'updateTextStyle':{'objectId':'cuvslam_official_intro_source','textRange':{'type':'ALL'},'style':{'link':{'url':'https://nvidia-isaac-ros.github.io/concepts/scene_reconstruction/nvblox/technical_details.html'}},'fields':'link'}})
two('p62','팔 카메라 입력과 촬영 시각 TF','토픽 연결','/arm_stereo/left/depth\n16UC1 깊이 영상, mm 단위\n\n/arm_stereo/left/camera_info\n깊이 영상과 같은 intrinsic·시각\n\nframe_id: arm_left_optical\nTF: odom → base_link → camera_rig','움직이는 카메라 처리','use_tf_transforms: true\nglobal_frame: odom\n\n영상 시각의 관절 엔코더로 FK 계산\n팔 회전을 차체 이동과 분리\n\n촬영 시각에 TF가 없으면 적분 대기\n최신 관절각으로 대체하지 않음')
use('integration_camera_ekf_frames','카메라·차체 좌표와 전역 지도 연결')
table('tune_cuvslam_tune','nvblox 해상도·주기 조정',[
['항목','현재 시작값','조정 방향','확인 기준'],['voxel_size','0.025 m','작게: 세밀·메모리 증가','얇은 벽·GPU 메모리'],['깊이 적분','15 Hz','센서 속도 이하','누락·입력 지연'],['ESDF 갱신','10 Hz','느리면 반응 지연','지도 나이·정지거리'],['mesh 갱신','2 Hz','시각화 필요량만','렌더링과 GPU 경합']], '설정 파일: config/nvblox_arm.yaml · 한 항씩 바꾸고 동일 경로에서 비교')
table('predictive_joint_preview','높이 범위·장애물 거리 설정',[
['항목','설정','조정 기준','실패 증상'],['slice 높이 범위','0.05–0.65 m','차체·팔 충돌 높이','바닥이 장애물로 누적'],['표시 높이','0.15 m','RViz 표시 위치','충돌 범위와 혼동'],['적분 최대 거리','5.0 m','센서 유효 거리','먼 거리 노이즈 누적'],['지도 유지 반경','6.0 m','미로 크기·메모리','먼 장애물 기록 삭제']], '높이 범위는 실제 로봇 외곽과 센서 오차를 기준으로 재설정')
two('p69','Nav2 경로 계획과 비용지도','계층 구성','StaticLayer: CAD 점유 지도\nNvbloxCostmapLayer: 실측 장애물\nInflationLayer: 로봇 여유 거리\n\nNavFn Dijkstra: 목표까지 경로 계산\nallow_unknown: false\n요청 주기: 1 s\n주행 제어는 기존 PPO에 연결','좌표계와 미관측 공간','nvblox 적분 좌표: odom\nNav2 비용지도 좌표: map\n플러그인이 두 좌표의 TF 사용\n\nCAD의 빈 공간은 관측 증거가 아님\n실제 전진은 신선한 깊이 관측 필요\n미관측 방향은 감속·관측 후 진행')
table('tune_nav2_tune','Nav2 비용지도 튜닝',[
['항목','현재 시작값','조정 방향','평가 기준'],['robot_radius','0.315 m','실제 외곽 반영','코너 접촉·통로 폭'],['inflation_radius','0.42 m','통로 여유와 절충','경로 생성·벽 여유'],['cost_scaling_factor','5.0','거리별 비용 변화','중앙 선호·우회'],['계획·지도 갱신','1 Hz / 5 Hz','CPU·반응 시간 비교','경로 지연·실시간 배율']], '원형 반경은 시뮬 시작값 · 비원형 외곽과 팔 돌출은 팀별 별도 확인')
two('p76','Nav2 경로를 PPO 관측으로 변환','경로 입력 규칙','ComputePathToPose action 사용\n/ policy/planned_path 에 결과 표시\n\nmap 좌표·유한수·목표 일치 검사\n중복점 제거 후 0.05 m 간격 재표본화\n새 경로에서 가까운 진행점 재선정\n\n경로 preview 방향으로 팔을 선행 회전','기존 정책 적용 범위','24차원 관측 순서와 단위 유지\n행동: 가감속·팔 yaw 속도\n\n재계획 경로는 학습 경로와 다를 수 있음\n굴곡·남은 거리·진행률 분포 확인\n\n완주율 저하 시 재계획 경로로 재학습\n정책 보상과 최종 완주 검증 구분')
table('integration_faults','입력 지연·경로 실패 시 처리',[
['상황','판정','현재 처리','검증 기준'],['경로 지연','2.5 s 초과','주행 차단','오래된 경로 추종 없음'],['nvblox 지도 지연','1.0 s 초과','주행 차단','입력 중단 후 정지'],['계획 실패','action 실패·거절','유효 경로 해제','막힌 통로 진입 없음'],['영상·IMU 오류','기존 freshness 검사','기존 감속·정지 유지','복구 후 재계획']], '센서와 경로의 시각은 /clock 기준 · 단일 속도 명령 소유자 유지')
two('p39','통합 실행과 결과 확인','실행 명령','GUI=1 USE_NVBLOX=1 DURATION=180\n  bash scripts/run_predictive_async.sh\n  evidence/nvblox-run\n\n줄을 이어 하나의 명령으로 실행\n\n기본 목표: 첫 미로 경로의 끝점\n시작·목표 이외 경로는 Nav2가 생성','결과 파일','nvblox-nav2.log: 지도·계획 노드\nnav2-path-events.jsonl: 재계획 이력\nlatest-nvblox-slice.json: 거리 지도\nlatest-costmap.json: Nav2 비용지도\n\nreport.json: 속도·접촉·오차·완주\nposes.npz: 시뮬 자세 기록\n성공 여부는 실제 실행 결과로 판정')
table('integration_results','통합 검증 결과와 남은 확인',[
['검증 항목','실행 결과','근거','범위'],['단위·회귀 검사','58개 통과','pytest tests','경로·센서·기구학'],['nvblox GPU 시작','노드 초기화 확인','smoke.log','지도 적분은 별도'],['Nav2 플러그인','정상 로드 확인','nvblox-nav2.log','계획 실행은 별도'],['폐루프 주행','검증 진행 중','report.json 확인','완주 결과로 갱신']], '검증 진행 중 · 이전 교안의 완주 결과를 새 통합 결과로 사용하지 않음')
two('p50','팀별 실습 과제와 비교 실험','튜닝 실험','voxel 0.025 / 0.05 m 비교\nESDF 갱신 5 / 10 Hz 비교\n팔 preview 거리·속도 비교\n\n동일 시작·목표·seed에서 반복\n완주 시간·최소 거리·접촉 기록\n프레임 지연과 GPU 메모리 기록','확장 실험','새 장애물로 경로 재계획 확인\n깊이 입력 중단 후 정지·복구 확인\nTF 시각 지연에 따른 지도 왜곡 확인\n\n움직이는 사람은 별도 동적 설정 필요\n현재 static_tsdf 결과와 구분\n실물 센서 노이즈·지연으로 재검증')
for i,sid in enumerate(keep,1):rep(sid+'_i7',str(i))
# Fix inadvertent whitespace in topic label.
req.append({'replaceAllText':{'containsText':{'text':'/ policy/planned_path','matchCase':True},'replaceText':'/policy/planned_path'}})
Path('build/lecture/nvblox/plan.json').write_text(json.dumps({'keep':keep,'slides':plan},ensure_ascii=False,indent=2))
for i in range(0,len(req),100):Path(f'build/lecture/nvblox/create-{i//100}.json').write_text(json.dumps(req[i:i+100],ensure_ascii=False))
print(len(keep),len(req),(len(req)+99)//100)
