import fs from 'node:fs/promises';
import path from 'node:path';
import {FileBlob,PresentationFile} from '@oai/artifact-tool';
const root=process.cwd();
const skill='/home/seongmin/.codex/plugins/cache/openai-primary-runtime/presentations/26.1013.11914/skills/presentations';
const template='/media/seongmin/One Touch/physicalAI/Lv2/모듈4_10_강의자료/모듈6/37_2D SLAM 원리_KANT.pptx';
const p=await PresentationFile.importPptx(await FileBlob.load(template));
p.slides.keep([1,2,3,4,7,8,17,23,24,26,29,32,34,36,37,38,42].map(x=>x-1));
const lessons=[
  [
    "팀별 로봇 시뮬레이션 개발",
    "모듈 6 프로젝트 공통 실습 01",
    "각 팀의 URDF·USD로 실습 / 강사 시연: M38"
  ],
  [
    "학습 목표와 팀별 적용",
    "각 팀은 자기 로봇의 URDF와 USD를 준비한다.\n관절·좌표·물성 정보를 설명하고 차이를 검증한다.",
    "로봇 구조에 맞는 제어기와 센서 구성을 선택한다.\nGit 이력, 로그, 캡처로 결과를 재현한다.",
    "M38은 강사 시연 사례다. 관절명·치수·구동 방식을\n다른 팀 로봇에 그대로 적용하지 않는다."
  ],
  [
    "공통 개발 환경과 설치 확인",
    "Ubuntu 24.04 / ROS 2 Lyrical / Isaac Sim 6.1.0.0\n실습 버전을 고정하고 각 팀 PC 상태를 기록한다.",
    "cat /etc/os-release\nnvidia-smi\ndf -h .",
    "GPU·메모리·설치 경로를 팀별로 확인한다.\nLyrical의 Sim native ROS 연결은 별도 검증이 필요하다."
  ],
  [
    "설치 환경 분리",
    "python3 -m venv ~/isaacsim6_venv\nsource ~/isaacsim6_venv/bin/activate",
    "Sim 설치는 버전을 고정한 setup_sim.sh를 사용한다.\nROS 패키지와 Python venv의 역할을 구분한다.",
    "설치 로그와 package version을 저장한다.\n세부 명령과 실행 확인: TEAM_LAB.md"
  ],
  [
    "Git 실습: 팀 모델의 출발점",
    "git status --short\ngit switch -c practice/team-model",
    "URDF·USD·메시를 함께 등록한다.\n외부 참조 경로와 모델 제공 범위를 확인한다.",
    "git add assets docs\ngit diff --cached\ngit commit -m \"feat: add team robot model\""
  ],
  [
    "Git 실습: 구현과 검증 이력",
    "git diff\ngit show <commit>\ngit log --oneline -5",
    "모델 수정, 제어 구현, 센서 연결을 작은 커밋으로 나눈다.\n실패 원인과 재현 명령을 변경 설명에 남긴다.",
    "공유 이력 취소: git revert <commit>\n캡처와 테스트 로그에 실행한 커밋을 기록한다."
  ],
  [
    "팀별 프로젝트 구조",
    "assets/<team_robot>/: URDF / USD / 메시\nconfig/: 관절 매핑·단위·제한값·센서 설정",
    "제어 코드 / ROS 어댑터 / launch / tests\n로봇 고유 수치와 공통 로직을 분리한다.",
    "evidence/: 버전·로그·PNG·GIF\ndocs/: 구조·원리·설치·실행 방법"
  ],
  [
    "URDF와 USD 모델 검증",
    "루트 링크, parent-child 트리, 관절 종류를 확인한다.\n메시와 USD 외부 참조가 모두 열리는지 검사한다.",
    "좌표계, 길이 단위, 관절 축, 영점, 제한값을 비교한다.\n질량·관성·충돌 설정의 가정값을 표시한다.",
    "정적 모델 로드 성공과 물리 구동 성공을 구분한다.\n모델 수정 전후 화면과 diff를 함께 남긴다."
  ],
  [
    "로봇 구조에 맞는 제어기",
    "차동구동: 좌우 바퀴 속도\n스워브: 각 모듈 조향각과 바퀴 속도",
    "팔 로봇: 관절 위치·속도 또는 궤적\n그 밖의 구조: 해당 기구학과 구동 인터페이스 정의",
    "강사 M38 예제는 스워브 구동이다.\n앞뒤 바퀴 축 방향과 회전 부호가 서로 다르다."
  ],
  [
    "코드 작성과 검증 순서",
    "모델에서 관절명·축·치수·제한값을 읽는다.\n입력 명령을 관절 목표로 바꾸는 함수를 작성한다.",
    "단위·부호·포화·정지 조건을 테스트한다.\n그다음 ROS 입력과 시뮬레이터 출력을 연결한다.",
    "M38 예: ux_i = vx − wz × yi\n이 수식은 평면 스워브의 시연 사례다."
  ],
  [
    "ROS 연결과 시간·좌표",
    "입력 명령과 측정 상태를 서로 다른 토픽으로 정의한다.\n관절명으로 명령과 articulation을 연결한다.",
    "시뮬레이션 clock과 센서 timestamp를 일치시킨다.\nTF의 parent-child 구조와 발행 주체를 정한다.",
    "통신 만료 시 정지하는 처리를 수신부에도 둔다.\nM38 예제는 cmd_vel·joint_states·관절 명령을 사용한다."
  ],
  [
    "빌드와 실행 검증",
    "source /opt/ros/lyrical/setup.bash\ncolcon build --packages-select <팀_패키지>",
    "source install/setup.bash\nros2 launch <팀_패키지> <팀_launch.py>",
    "노드 로드, 명령 전달, 측정 상태를 차례로 확인한다.\nM38 실행 예제는 강사용 LAB_GUIDE.md에 분리했다."
  ],
  [
    "강사 시연: M38 모델 렌더",
    "",
    "",
    "강사 예시 캡처. 각 팀은 자기 USD 모델의 같은 단계 화면을 제출한다."
  ],
  [
    "강사 시연: 스테레오 센서",
    "",
    "",
    "M38 가상 카메라 사례. 각 팀은 자기 센서의 해상도·외부보정을 기록한다."
  ],
  [
    "센서와 알고리즘 선택",
    "스테레오 영상: CameraInfo·좌우 동기화·optical TF\nLiDAR: LaserScan·거리/각도 규약·scan TF",
    "cuVSLAM과 slam_toolbox·AMCL은 입력이 다르다.\n팀 목표에 맞는 파이프라인을 구성한다.",
    "깊이 예제: Z = fx × B / disparity\nfx와 기준선 B는 각 팀 카메라 값으로 바꾼다."
  ],
  [
    "단계별 완료 기준",
    "모델 로드: 참조·축·단위 검증\n구동: 정지·방향·제한·충돌 검증",
    "센서: 주기·timestamp·TF 검증\n추정·주행: 목표에 맞는 정확도와 성공률 측정",
    "M38 현재: 제어 통신·렌더·저장 영상 초기화 확인\n물리 주행과 실시간 SLAM은 후속 검증이다."
  ],
  [
    "팀별 제출물과 발표",
    "설치법 / 파일 구조 / 제어·센서 원리 / 코드 설명",
    "Git 커밋 / 재현 명령 / 테스트 로그\n자기 로봇의 실행 PNG와 짧은 GIF",
    "가정값과 실패 조건, 미완료 항목을 명시한다.\n다른 팀이 자료만 보고 같은 결과를 재현하는지 확인한다."
  ]
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
 s.speakerNotes.text=`실습 ${i+1}. ${title}\n공통 실습: teaching/TEAM_LAB.md. 강사 M38 코드 사례: teaching/LAB_GUIDE.md. 실행 증거: evidence/2026-10-26.\n원본 양식: ${template}\n${body.join('\n')}\n${i===14?'가상 카메라 파라미터 출처: docs/CAMERA_AND_TF.md. 깊이 예제는 이상식 계산이며 측정 결과가 아니다.':''}`;
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
console.log(await finalizePresentation({workspaceDir:root,candidatePath:candidate,finalPath:path.join(root,'teaching/output/팀별로봇_개발실습01.pptx'),pythonExecutable:'/home/seongmin/.cache/codex-runtimes/codex-primary-runtime/dependencies/python/bin/python3',integrityValidatorPath:`${skill}/container_tools/inspect_presentation_package_integrity.py`,layoutValidatorPath:`${skill}/container_tools/inspect_presentation_layout_geometry.py`,layoutArgs:['--expected-slide-size-emu','9144000,5143500','--validate-heading-fit'],requiredNativeTableOwnerSlides:[],verifyArtifactToolImport:true,receiptPath:path.join(root,'build/teaching/team-validation.json')}));
