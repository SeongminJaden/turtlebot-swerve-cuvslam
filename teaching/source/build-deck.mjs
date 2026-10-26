import fs from 'node:fs/promises';
import path from 'node:path';
import {FileBlob,PresentationFile} from '@oai/artifact-tool';
const root=process.cwd();
const skill='/home/seongmin/.codex/plugins/cache/openai-primary-runtime/presentations/26.1013.11914/skills/presentations';
const template='/media/seongmin/One Touch/physicalAI/Lv2/모듈4_10_강의자료/모듈6/37_2D SLAM 원리_KANT.pptx';
const p=await PresentationFile.importPptx(await FileBlob.load(template));
p.slides.keep([1,2,3,4,7,8,17,23,24,26,29,32,34,36,37,38,42].map(x=>x-1));
const lessons=[
 ['M38 스워브 로봇 개발','모듈 6 프로젝트 보충 실습 01','Ubuntu 24.04 · ROS 2 Lyrical · Isaac Sim 6.1'],
 ['학습 목표와 실습 범위','URDF의 관절 축을 읽고 스워브 역기구학을 구현한다.','Git 변경 이력과 테스트 로그로 구현 결과를 재현한다.','이번 실습: 제어 노드와 렌더링. 물리 주행과 SLAM은 후속 검증이다.'],
 ['개발 PC와 설치 확인','Ubuntu 24.04.5 / ROS 2 Lyrical / NVIDIA 595.91.07','RTX 5060 Laptop 8GB. nvidia-smi의 CUDA 표시는 드라이버 지원 수준이다.','cat /etc/os-release\nnvidia-smi\ndf -h .'],
 ['환경 설치와 버전 고정','Isaac Sim 6.1.0.0을 별도 Python 3.12 venv에 설치한다.','python3 -m venv ~/isaacsim6_venv\nsource ~/isaacsim6_venv/bin/activate','정확한 pip 명령은 LAB_GUIDE와 설치 로그를 따른다. cuVSLAM 패키지는 별도 설치가 필요하다.'],
 ['Git 실습: 변경을 작은 단위로 기록','git status --short\ngit switch -c practice/swerve-control','git diff\ngit add turtlebot_swerve tests\ngit diff --cached','git commit -m "feat: add swerve control"\ngit log --oneline -5'],
 ['Git 실습: 비교와 되돌리기','git show <commit>\ngit diff HEAD~1 HEAD','공유 이력의 변경 취소: git revert <commit>\n실패 원인과 수정 이유도 커밋 설명에 남긴다.','영상 원본과 대용량 bag은 별도 보관한다. 작은 캡처와 재현 로그를 코드 버전과 연결한다.'],
 ['프로젝트 구조','assets: URDF / USD / 메시 / 가정 물성\nturtlebot_swerve: 역기구학과 ROS 제어','launch: 실행 구성\ntests: 수학적 동작 검증\nscripts: 통신 검사와 캡처','evidence: 실제 실행 로그와 렌더 이미지\nteaching: 강의 슬라이드와 실습 가이드'],
 ['스워브 역기구학','몸체 속도: vx, vy [m/s], wz [rad/s]\n모듈 위치: xi, yi [m]','ux_i = vx − wz × yi\nuy_i = vy + wz × xi','진행각 = atan2(uy_i, ux_i)\n바퀴 각속도 = 진행속도 / 반지름'],
 ['모델별 회전축과 부호','양의 진행 방향은 바퀴 축과 +Z의 외적이다.\n앞바퀴 +X축: 기본 진행 방향 −Y','뒤 왼쪽 +Y축: 기본 진행 방향 +X\n뒤 오른쪽 −Y축: 기본 진행 방향 −X','기준 진행각을 빼서 조향각을 구한다.\n±90°를 넘으면 조향각을 π만큼 바꾸고 회전 부호를 뒤집는다.'],
 ['코드 작성과 단위 테스트','load_modules(): URDF에서 위치·축·반지름 읽기\nsolve(): 몸체 속도를 관절 목표로 변환','속도 제한: 모든 바퀴에 공통 비율 적용\n정지 명령: 조향 유지, 회전 속도 0','python3 -m unittest discover -s tests -v\n직진·횡이동·회전·포화·비정상 입력 검증'],
 ['ROS 메시지와 정지 조건','입력 /cmd_vel: 몸체 속도\n입력 /joint_states: 측정 관절 상태','출력 /swerve/joint_commands\n조향 position과 바퀴 velocity를 별도 메시지로 전송','명령 또는 피드백이 0.5초 끊기면 정지한다.\n모든 조향 오차가 0.05 rad 미만일 때 바퀴 회전을 허용한다.'],
 ['빌드와 실행','source /opt/ros/lyrical/setup.bash\ncolcon build --base-paths . --packages-select turtlebot_swerve','source install/setup.bash\nros2 launch turtlebot_swerve swerve_control.launch.py','측정 피드백이 없으면 바퀴 명령 0이 정상이다.\n물리 시뮬레이터 어댑터는 후속 구현 대상이다.'],
 ['실제 USD 모델 렌더','', '', 'Isaac Sim 6.1 렌더 캡처. 로봇 외형 확인이며 물리 주행 검증은 아니다.'],
 ['좌우 스테레오 렌더','', '', '각 640×480, 가상 기준선 0.05 m. 좌우 비교 GIF를 함께 제공한다.'],
 ['스테레오 깊이와 cuVSLAM','이상적인 깊이: Z = fx × B / disparity\nfx=400 px, B=0.05 m, 시차 20 px라면 깊이 1 m','Image / CameraInfo / clock / TF의 시간과 좌표를 일치시킨다.\n추정 TF와 ground truth TF를 중복 발행하지 않는다.','cuVSLAM은 영상 기반 추정이다. 모듈6의 LaserScan·점유격자·AMCL 실습과 입력이 다르다.'],
 ['검증 결과와 남은 개발','완료: 역기구학 테스트 5개, 패키지 빌드\nROS 합성 피드백 검사 3개, 좌우 영상 렌더','미완료: 물리 구동, 센서 ROS 발행, cuVSLAM 추적\nNav2 경로 이동과 장애물 회피','다음 캡처: 정지·직진·횡이동·회전, TF와 토픽\n다음 지표: ATE/RPE, 추적 손실, 주행 성공률'],
 ['제출물과 재현 확인','코드 변경 + Git 커밋 + 실행 명령 + 종료 코드','캡처마다 버전과 실행 조건을 기록한다.\n합성 테스트, 렌더링, 물리 주행 결과를 구분한다.','질문: 뒤 오른쪽 바퀴 회전 부호가 다른 이유는?\n질문: 피드백 없이 바퀴를 회전시키면 어떤 문제가 생기는가?']
];
for(let i=0;i<lessons.length;i++){
 const s=p.slides.items[i]; const shapes=s.shapes.items;
 const [title,...body]=lessons[i];
 for(const x of shapes){const t=x.text.toString(); if(t.startsWith('Lv.2')) x.text.replace(t,'Lv.2 프로젝트'); else if(/^\d\d$/.test(t)) x.text.replace(t,String(i+1).padStart(2,'0'));}
 shapes[2].text.replace(shapes[2].text.toString(),title);
 if(i===0){ for(let j=0;j<2;j++)shapes[j+3].text.replace(shapes[j+3].text.toString(),body[j]); }
 else{
   for(let j=0;j<3;j++){const x=shapes[j+4]; x.text.replace(x.text.toString(),body[j]);}
 }
 s.speakerNotes.text=`실습 ${i+1}. ${title}\n관련 코드와 명령: teaching/LAB_GUIDE.md. 실행 증거: evidence/2026-10-26.\n원본 양식: ${template}\n${body.join('\n')}\n${i===14?'가상 카메라 파라미터 출처: docs/CAMERA_AND_TF.md. 깊이 예제는 이상식 계산이며 측정 결과가 아니다.':''}`;
 if(i===12||i===13){
   shapes[6].position={left:65,top:451,width:830,height:48};
   shapes[6].text.style={fontFamily:'맑은 고딕',fontSize:18,color:'#333333'};
   let images=i===12?['model-overview.png']:['sim61_left.png','sim61_right.png'];
   for(let j=0;j<images.length;j++){
     const b=await fs.readFile(`evidence/2026-10-26/${images[j]}`);
     s.images.add({blob:b.buffer.slice(b.byteOffset,b.byteOffset+b.byteLength),contentType:'image/png',alt:images[j],fit:'contain',position:i===12?{left:240,top:140,width:480,height:300}:{left:65+j*425,top:155,width:405,height:280}});
   }
 }
}
await fs.mkdir('teaching/output',{recursive:true});
for(let i=0;i<p.slides.items.length;i++)await fs.writeFile(`build/teaching/final-${i+1}.png`,new Uint8Array(await(await p.export({slide:p.slides.items[i],format:'png',scale:1})).arrayBuffer()));
await fs.writeFile('build/teaching/final-montage.webp',new Uint8Array(await(await p.export({format:'webp',montage:true})).arrayBuffer()));
const candidate=path.join(root,'build/teaching/candidate.pptx');
await(await PresentationFile.exportPptx(p)).save(candidate);
const {finalizePresentation}=await import(`${skill}/container_tools/artifact_tool_utils.mjs`);
console.log(await finalizePresentation({workspaceDir:root,candidatePath:candidate,finalPath:path.join(root,'teaching/output/M38_개발실습01.pptx'),pythonExecutable:'/home/seongmin/.cache/codex-runtimes/codex-primary-runtime/dependencies/python/bin/python3',integrityValidatorPath:`${skill}/container_tools/inspect_presentation_package_integrity.py`,layoutValidatorPath:`${skill}/container_tools/inspect_presentation_layout_geometry.py`,layoutArgs:['--expected-slide-size-emu','9144000,5143500','--validate-heading-fit'],requiredNativeTableOwnerSlides:[],verifyArtifactToolImport:true,receiptPath:path.join(root,'build/teaching/validation.json')}));
