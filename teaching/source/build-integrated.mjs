import fs from 'node:fs/promises';
import path from 'node:path';
import {FileBlob,PresentationFile} from '@oai/artifact-tool';
const root=process.cwd();
const p=await PresentationFile.importPptx(await FileBlob.load('teaching/output/팀별로봇_개발실습01.pptx'));
const lessons=JSON.parse(await fs.readFile('teaching/source/rl-slides.json','utf8'));
const template=p.slides.items[1];
for(let i=0;i<lessons.length;i++){const s=template.duplicate();s.moveTo(p.slides.items.length-1);}
const FONT='맑은 고딕';
function text(s,content,x,y,w,h,size=24,color='#252525',bold=false){const a=s.shapes.add({geometry:'textbox',position:{left:x,top:y,width:w,height:h},fill:'none',line:{fill:'none',width:0}});a.text=content;a.text.style={typeface:FONT,fontSize:size,color,bold};return a;}
function clear(s){for(const a of s.shapes.items.slice(4))a.delete();for(const a of [...s.images.items])a.delete();}
function title(s,t,n){s.shapes.items[2].text.replace(s.shapes.items[2].text.toString(),t);s.shapes.items[3].text.replace(s.shapes.items[3].text.toString(),String(n).padStart(2,'0'));}
function box(s,label,x,y,w=184,h=142){let b=s.shapes.add({geometry:'rect',position:{left:x,top:y,width:w,height:h},fill:'#F0F3F5',line:{fill:'#617889',width:1.4}});b.text=label;b.text.style={typeface:FONT,fontSize:22,color:'#1E303D',alignment:'center',verticalAlignment:'middle'};return b;}
function connect(s,a,b,from='right',to='left'){s.shapes.connect(a,b,{kind:'straight',fromSide:from,toSide:to,line:{fill:'#617889',width:2},tail:{type:'triangle',width:'med',length:'med'}});}
function flow(s,labels,caption,cycle=false){let bs;if(cycle){bs=[box(s,labels[0],110,170,290,88),box(s,labels[1],550,170,290,88),box(s,labels[2],550,330,290,88),box(s,labels[3],110,330,290,88)];connect(s,bs[0],bs[1]);connect(s,bs[1],bs[2],'bottom','top');connect(s,bs[2],bs[3],'left','right');connect(s,bs[3],bs[0],'top','bottom');}else{bs=labels.map((v,i)=>box(s,v,65+i*212,190));for(let i=0;i<3;i++)connect(s,bs[i],bs[i+1]);}if(caption)text(s,caption,65,452,830,48,20,'#4B4B4B');}
async function image(s,file,x,y,w,h){const b=await fs.readFile(file);s.images.add({blob:b.buffer.slice(b.byteOffset,b.byteOffset+b.byteLength),contentType:file.endsWith('.gif')?'image/gif':'image/png',alt:path.basename(file),fit:'contain',position:{left:x,top:y,width:w,height:h}});}
// Existing lesson retains its original master and fonts, with terminology corrected everywhere.
for(const s of p.slides.items){for(const a of s.shapes.items){const t=a.text.toString();if(t.includes('강사'))a.text.replace(t,t.replaceAll('강사용','시연용').replaceAll('강사 시연','시연').replaceAll('강사 예시','시연 예시').replaceAll('강사','시연'));}if(s.speakerNotes?.text)s.speakerNotes.text=s.speakerNotes.text.replaceAll('강사용','시연용').replaceAll('강사','시연');}
p.slides.items[0].shapes.items[2].text.replace('팀별 로봇 시뮬레이션 개발','팀별 로봇 미로 주행 실습');
p.slides.items[0].shapes.items[3].text.replace(p.slides.items[0].shapes.items[3].text.toString(),'Isaac Sim · Isaac Lab 강화학습');
p.slides.items[0].shapes.items[4].text.replace(p.slides.items[0].shapes.items[4].text.toString(),'팀별 URDF·USD로 실습 / 시연: M38');
title(p.slides.items[16],'기초 실습 체크포인트',17);
const originals={
  4:['자율주행 실습의 네 단계',['모델 검증\nURDF·USD','위치추정\ncuVSLAM','주행 정책\nIsaac Lab','통합 평가\nIsaac Sim'],'센서·위치·행동·물리 결과를 단계별로 확인한다.'],
  5:['위치추정과 자율주행의 차이',['내 위치는?\ncuVSLAM','어디로 갈까?\n경로계획','어떻게 움직일까?\n정책·구동기','실제로 도착했나?\n평가'],'위치추정만 실행해도 자율주행이 완성되는 것은 아니다.'],
  6:['팀별 프로젝트 구조',['assets/\nURDF·USD·미로','rl/\n환경·정책·학습','scripts/\n변환·실행·촬영','evidence/\n로그·포즈·영상'],'공통 학습 흐름과 로봇 고유 설정을 분리한다.'],
  7:['URDF와 USD 모델 검증',['트리·참조\n모델 로드','단위·축\n관절 비교','물성·충돌\n정지 상태','구동 검증\n방향·제한'],'겉모습이 정상이어도 articulation root와 충돌 설정은 별도 확인한다.'],
  9:['코드 작성과 검증 순서',['모델 정보\n이름·축·치수','입력 변환\n로봇별 운동학','단위 테스트\n부호·포화','물리 실행\n변위·접촉'],'시연의 스워브 운동학은 다른 구조의 로봇에 맞게 바꾼다.'],
  10:['ROS 연결과 시간·좌표',['명령 노드\n목표값 발행','ROS bridge\n시간·토픽','Isaac Sim\n관절 적용','측정 상태\n실제 관절값'],'timestamp, TF, 관절명을 맞추고 수신 만료 시 정지 동작을 검증한다.'],
  15:['단계별 완료 기준',['모델 로드\n참조·축·단위','구동 검증\nPhysX 이동','정책 평가\n완주·정체','센서 폐루프\n별도 검증'],'현재 물리 이동과 PPO 학습·평가를 실행했다. 상세 결과는 뒤의 평가 자료에 표시한다.'],
};
for(const [idx,v] of Object.entries(originals)){const s=p.slides.items[Number(idx)];clear(s);title(s,v[0],Number(idx)+1);flow(s,v[1],v[2],Number(idx)===10);}
// New material.
for(let i=0;i<lessons.length;i++){
 const d=lessons[i],s=p.slides.items[17+i];clear(s);title(s,d.title,18+i);
 s.speakerNotes.text=d.notes+'\n\n구현 출처: rl/maze_env.py, rl/policy.py, rl/train_ppo.py, rl/evaluate.py. 시연 데이터: evidence/2026-10-26.\nIsaac Lab 소스: https://github.com/isaac-sim/IsaacLab/tree/78b12aed1d2a56439493b8ea6d3538e6e379e2e5\nPPO 원 논문: https://arxiv.org/abs/1707.06347';
 if(d.kind==='flow')flow(s,d.body,d.caption,['환경 클래스의 실행 순서','PPO 학습 루프','학습과 실행의 차이','정체 현상의 진단 흐름','학습이 안 될 때 점검 순서'].includes(d.title));
 if(d.kind==='gif'){await image(s,'evidence/2026-10-26/maze-eval-100/render/isaacsim-recorded-motion.gif',180,140,600,340);text(s,'실제 PhysX 기록 재생 / 0~40초 구간 / 약 7배속 / 초기 정책의 정체 사례',65,477,830,32,18);}
 if(d.kind==='sensorchoice'){
 const l=box(s,'2D LiDAR\nLaserScan',65,170,260,80),dep=box(s,'깊이 카메라\nPointCloud2',65,345,260,80),cm=box(s,'costmap 장애물층\nmarking·clearing',400,245,245,100),inf=box(s,'InflationLayer\n주변 여유 거리',715,245,180,100);connect(s,l,cm);connect(s,dep,cm);connect(s,cm,inf);text(s,'센서 대안 중 하나를 선택하거나 함께 사용한다.',65,460,830,35,20);
 }
 if(d.kind==='code'){text(s,d.body.join('\n\n'),65,160,830,310,22);}
 if(d.kind==='image'){await image(s,d.image,65,155,500,315);text(s,d.body.join('\n\n'),590,165,300,300,23);}
 if(d.kind==='chart'){
   const hist=JSON.parse(await fs.readFile('runs/maze_ppo/training.json','utf8'));
   const chart=s.charts.add('line',{position:{left:65,top:155,width:610,height:315},title:'Rollout 평균 보상',titleTextStyle:{fontSize:20},categories:hist.map(r=>String(r.iteration)),series:[{name:'평균 보상',values:hist.map(r=>r.mean_reward),line:{fill:'#326680',width:2}}],hasLegend:false,xAxis:{title:'PPO iteration',textStyle:{fontSize:12}},yAxis:{title:'Mean reward / step',textStyle:{fontSize:12},numberFormatCode:'0.00'},chartFill:'#FFFFFF',plotAreaFill:'#FFFFFF'});
   text(s,'51,200 transition\n\n8개 병렬 환경\n\n완주 성공률은\n별도 평가',700,180,190,250,23);
 }
 if(d.kind==='evaluation'){
   const r=JSON.parse(await fs.readFile('evidence/2026-10-26/maze-eval-100/report.json','utf8'));
   const rows=(await fs.readFile('evidence/2026-10-26/maze-eval-100/trajectory.csv','utf8')).trim().split('\n').slice(1).filter((_,j)=>j%10===0).map(x=>x.split(',').map(Number));const route=JSON.parse(await fs.readFile('assets/maze/maze.json','utf8')).route;
   s.charts.add('scatter',{position:{left:65,top:150,width:560,height:330},title:'첫 미로: 계획 경로와 실제 궤적',titleTextStyle:{fontSize:19},series:[{name:'계획 경로',xValues:route.map(r=>r[0]),values:route.map(r=>r[1]),line:{fill:'#9A9A9A',width:2},marker:{symbol:'none'}},{name:'100회 정책',xValues:rows.map(r=>r[1]),values:rows.map(r=>r[2]),line:{fill:'#B35A3E',width:3},marker:{symbol:'none'}}],scatterOptions:{style:'line'},hasLegend:true,legend:{position:'bottom',textStyle:{fontSize:14}},xAxis:{title:'X (m)',min:0,max:4.5},yAxis:{title:'Y (m)',min:0,max:3.3},chartFill:'#FFFFFF'});
   text(s,`완주 실패\n\n${r.simulated_s.toFixed(0)}초 평가\n\n최종 목표 오차\n${r.final_goal_error_m.toFixed(2)} m\n\n중간 구간 정체`,650,170,240,290,23);
 }
 if(d.kind==='comparison'){
   const paths=['maze-eval-100','maze-eval-bidirectional-preview','maze-baseline-hysteresis','maze-eval-final'];
   const values=[['실행','완주','모의 시간 (s)','목표 오차 (m)']];
   for(const dir of paths){try{const r=JSON.parse(await fs.readFile(`evidence/2026-10-26/${dir}/report.json`,'utf8'));values.push([r.policy==='proportional_debug_baseline'?'비교 제어기':`PPO ${r.checkpoint_iteration}회`,r.success?'성공':'실패',r.simulated_s.toFixed(1),r.final_goal_error_m.toFixed(3)]);}catch{values.push(['최종 정책','평가 중','—','—']);}}
   const t=s.tables.add({rows:5,columns:4,left:65,top:175,width:820,height:260,columnWidths:[250,150,210,210],values});
   for(let r=0;r<5;r++)for(let c=0;c<4;c++){const a=t.getCell(r,c);a.fill=r===0?'#E7EDF1':'#FFFFFF';a.text.style={typeface:FONT,fontSize:22,bold:r===0,color:'#222222'};}t.borders.assign({style:'solid',fill:'#9BA9B3',width:1});text(s,'각 1회 평가 / 알려진 경로·Sim 위치 사용 / 비교 제어기는 강화학습 정책과 구분',65,460,830,45,19);
 }
 if(d.kind==='capture'){
   const dir='evidence/2026-10-26/maze-eval-100/render';
   try{await image(s,dir+'/robot-00.png',65,150,400,300);await image(s,dir+'/robot-35.png',495,150,400,300);text(s,'실제 PhysX 포즈를 Isaac Sim으로 재생 렌더링한 화면 / 100회 정책',65,460,830,38,19);}catch{ text(s,'실제 렌더 캡처 생성 중',65,180,800,100,28); }
 }
}
// Explicit data aggregation and decision diagrams.
function redrawAt(titleText,draw){const j=lessons.findIndex(x=>x.title===titleText);const s=p.slides.items[17+j];clear(s);draw(s);}
function aggregate(s,labels,result,caption){const inputs=labels.map((v,i)=>box(s,v,65+i*212,165,184,112));const output=box(s,result,310,350,340,70);for(const b of inputs)connect(s,b,output,'bottom','top');text(s,caption,65,455,830,44,20);}
redrawAt('관측 벡터 12차원',s=>aggregate(s,lessons.find(x=>x.title==='관측 벡터 12차원').body,'torch.cat / [num_envs, 12]','네 종류의 정보를 같은 순서로 연결해 정책에 입력한다.'));
redrawAt('보상 함수의 구성',s=>aggregate(s,lessons.find(x=>x.title==='보상 함수의 구성').body,'각 항의 합 = reward','항별 크기와 실제 주행 결과를 함께 비교한다.'));
redrawAt('Actor와 Critic 네트워크',s=>{const obs=box(s,'관측 12',65,265,165,85);const actor=box(s,'Actor MLP\n64 Tanh + 64 Tanh',300,165,290,100);const critic=box(s,'Critic MLP\n64 Tanh + 64 Tanh',300,335,290,100);const a=box(s,'평균 행동 2\n+ 학습 logstd',675,165,210,100);const v=box(s,'가치 추정 1\nV(s)',675,335,210,100);connect(s,obs,actor);connect(s,obs,critic);connect(s,actor,a);connect(s,critic,v);text(s,'Actor와 Critic은 각각 별도의 가중치를 학습한다.',65,460,830,35,20);});
redrawAt('미로 주행 프로젝트 전체 구조',s=>{const model=box(s,'팀별 URDF·USD\n미로 USD',65,165,220,90),sim=box(s,'Isaac Sim PhysX\n로봇·접촉 물리',365,165,220,90),env=box(s,'Isaac Lab 환경\n관측·보상·종료',665,165,220,90),ppo=box(s,'PPO 업데이트\n가중치 학습',665,340,220,90),policy=box(s,'저장 정책\n이동 명령',365,340,220,90),motor=box(s,'로봇별 운동학\n관절 목표',65,340,220,90);connect(s,model,sim);connect(s,sim,env);connect(s,env,ppo,'bottom','top');connect(s,ppo,policy,'left','right');connect(s,policy,motor,'left','right');connect(s,motor,sim,'top','left');text(s,'실선: 현재 물리·학습 경로 / 센서 위치추정 연결은 별도 확장 단계',65,460,830,35,19);});
redrawAt('정체 현상의 진단 흐름',s=>{const start=box(s,'목표 거리 정체',65,165,210,70);const q=box(s,'명령과 실제 속도가\n모두 0에 가까운가?',355,155,250,100);q.geometry='diamond';q.text='';const qt=text(s,'명령·속도\n모두 ≈ 0?',415,174,130,65,20);qt.text.style={typeface:FONT,fontSize:20,alignment:'center'};const learning=box(s,'관측·보상·정책\n업데이트 검사',675,165,210,90);const physics=box(s,'관절·벽 접촉\n조향 지연 검사',355,345,250,90);connect(s,start,q);connect(s,q,learning);connect(s,q,physics,'bottom','top');text(s,'예',620,175,45,30,19);text(s,'아니오',470,285,100,30,19);text(s,'한 원인씩 수정한 뒤 같은 체크포인트 평가 조건으로 비교한다.',65,460,830,35,20);});

redrawAt('학습과 실행의 차이',s=>{text(s,'학습',65,145,120,35,22,'#222222',true);const a=box(s,'행동 샘플링',65,195,220,75),b=box(s,'PhysX 데이터 수집',365,195,220,75),c=box(s,'가중치 업데이트',665,195,220,75);connect(s,a,b);connect(s,b,c);text(s,'평가',65,310,120,35,22,'#222222',true);const d=box(s,'저장된 가중치',65,360,220,75),e=box(s,'평균 행동 선택',365,360,220,75),f=box(s,'PhysX 주행 실행',665,360,220,75);connect(s,d,e);connect(s,e,f);text(s,'평가 중에는 가중치를 변경하지 않는다.',65,465,830,35,20);});
{const s=p.slides.items[6];clear(s);const r=box(s,'팀 프로젝트 저장소',315,155,330,70);const labels=originals[6][1];for(let i=0;i<4;i++){const b=box(s,labels[i],65+i*212,315,184,100);connect(s,r,b,'bottom','top');}text(s,'모델·학습 코드·도구·증거를 서로 다른 폴더에 둔다.',65,460,830,35,20);}
try{const s=p.slides.items[13];clear(s);title(s,'시연: 미로의 좌우 카메라 영상',14);await image(s,'evidence/2026-10-26/stereo-maze-render/stereo-left.png',65,160,400,290);await image(s,'evidence/2026-10-26/stereo-maze-render/stereo-right.png',495,160,400,290);text(s,'640 × 480, 가상 기준선 0.05 m / 정적 렌더이며 이동 SLAM 검증과 구분한다.',65,465,830,35,19);}catch{}

redrawAt('환경 클래스의 실행 순서',s=>{const r=box(s,'reset\n초기 상태',65,165,190,90),o=box(s,'관측 계산',365,165,220,90),a=box(s,'step(action)\n물리 4회',665,165,220,90),v=box(s,'보상·종료 계산',665,340,220,85),q=box(s,'종료?',380,340,190,85);q.geometry='diamond';connect(s,r,o);connect(s,o,a);connect(s,a,v,'bottom','top');connect(s,v,q,'left','right');connect(s,q,o,'top','bottom');connect(s,q,r,'left','bottom');text(s,'계속',480,280,80,30,18);text(s,'종료',230,335,80,30,18);text(s,'physics dt = 1/60 s, decimation = 4, policy step = 1/15 s',65,465,830,35,20);});
redrawAt('보상 설계 실수와 수정',s=>{const values=[['관찰한 현상','우선 확인할 내용'],['정지 상태를 유지','진행률 보상과 시간비용 크기'],['벽에 계속 밀착','접촉·벽 거리 관측과 목표 위치'],['목표 주변에서 진동','도달 기준과 감속 처리'],['리셋 순간 보상 급증','새 목표의 이전 거리 초기화']];const t=s.tables.add({rows:5,columns:2,left:65,top:165,width:820,height:265,columnWidths:[280,540],values});for(let r=0;r<5;r++)for(let c=0;c<2;c++){const cell=t.getCell(r,c);cell.fill=r===0?'#E7EDF1':'#FFFFFF';cell.text.style={typeface:FONT,fontSize:23,color:'#222222',bold:r===0};}t.borders.assign({style:'solid',fill:'#9BA9B3',width:1});text(s,'원인 가설을 하나씩 바꾸고 같은 조건에서 재평가한다.',65,465,830,35,20);});

{const j=lessons.findIndex(x=>x.title==='체크포인트 저장과 이어 학습');text(p.slides.items[17+j],'최신 파일보다 고정 조건 평가 결과를 기준으로 정책을 선택한다.',65,465,830,35,20);}
// Replace old static captures with current maze evidence when available.
try{const s=p.slides.items[12];clear(s);title(s,'시연: 제공 미로와 로봇',13);await image(s,'evidence/2026-10-26/maze-eval-100/render/maze-overview.png',150,140,660,340);}catch{}
for(const sl of p.slides.items){
 for(const sh of sl.shapes.items){const old=sh.text.toString();const val=old.replaceAll('Git 이력, 로그, 캡처','설정, 로그, 캡처').replaceAll('Git 커밋 / 재현 명령 / 테스트 로그','설정 파일 / 재현 명령 / 테스트 로그');if(val!==old)sh.text.replace(old,val);}
 if(sl.speakerNotes?.text)sl.speakerNotes.text=sl.speakerNotes.text.split('\n').filter(v=>!(/\bgit\b|커밋|브랜치/i.test(v))).join('\n');
}
const {applyPresentationChartFont}=await import('/home/seongmin/.codex/plugins/cache/openai-primary-runtime/presentations/26.1025.11240/skills/presentations/container_tools/artifact_tool_utils.mjs');
for(const sl of p.slides.items)for(const ch of sl.charts.items)applyPresentationChartFont(ch,{fontFamily:FONT});
await fs.writeFile('build/lecture/outline.json',JSON.stringify(p.slides.items.map((s,i)=>({slide:i+1,text:s.shapes.items.map(x=>x.text.toString())})),null,2));
for(let i=0;i<p.slides.items.length;i++){await fs.writeFile(`build/lecture/slide-${String(i+1).padStart(2,'0')}.png`,new Uint8Array(await(await p.export({slide:p.slides.items[i],format:'png',scale:1})).arrayBuffer()));console.log('RENDERED',i+1);}
await(await PresentationFile.exportPptx(p)).save('build/lecture/candidate.pptx');
console.log('BUILD_COMPLETE',p.slides.items.length);
