from pathlib import Path
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import FancyBboxPatch,FancyArrowPatch
from matplotlib.font_manager import FontProperties
font=FontProperties(fname='/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc')
fig,ax=plt.subplots(figsize=(12,4.4),dpi=200,facecolor='white');ax.set(xlim=(0,12),ylim=(0,4.4));ax.axis('off')
def box(x,y,w,h,text,color='#EAF2FA',edge='#54779F'):
 ax.add_patch(FancyBboxPatch((x,y),w,h,boxstyle='round,pad=0.05,rounding_size=.08',facecolor=color,edgecolor=edge,linewidth=1.2));ax.text(x+w/2,y+h/2,text,ha='center',va='center',fontproperties=font,fontsize=16,color='#22344D',linespacing=1.25)
def arrow(a,b,color='#7B91A8'):
 ax.add_patch(FancyArrowPatch(a,b,arrowstyle='-|>',mutation_scale=14,color=color,linewidth=1.4,connectionstyle='arc3'))
box(.15,3.3,2.3,.65,'OpenCR IMU × 2\n축·시간·위치 보정')
box(.15,2.2,2.3,.65,'엔코더 13축\nunwrap · 속도 · 팔 FK')
box(.15,.95,2.3,.8,'팔 RGBD · 스테레오\n영상과 같은 시각의 FK')
box(3.2,3.3,2.2,.65,'평균 IMU · wheel odom\n200 Hz 센서 경로')
box(3.2,1.15,2.2,.65,'cuVSLAM → body\n추적 검사 · 차분 측정')
box(6.2,2.65,2.2,.8,'robot_localization\n연속 odom · 차체 속도','#E6F4EF','#36816B')
box(6.2,1.15,2.2,.8,'Nav2 AMCL\nCAD 지도 + 팔 깊이','#E6F4EF','#36816B')
box(9.2,2.65,2.6,.8,'경로 preview + PPO\n가감속 · 팔 yaw','#F8F0DF','#B18B44')
box(9.2,1.15,2.6,.8,'조향·제동·추적 제약\n측정 관절에 명령 적용','#F8F0DF','#B18B44')
for a,b in [((2.5,3.65),(3.15,3.65)),((2.5,2.55),(3.15,3.45)),((2.5,1.4),(3.15,1.4)),((5.45,3.6),(6.15,3.1)),((5.45,1.55),(6.15,2.85)),((7.3,2.6),(7.3,2.0)),((8.45,3.1),(9.15,3.1)),((8.45,1.6),(9.15,2.9)),((10.5,2.6),(10.5,2.0))]:arrow(a,b)
arrow((2.5,1.),(6.15,1.3))
ax.text(6,.35,'측정 → 추정 → 정책 → 움직임 → 새 측정  |  실제 pose는 평가에만 사용',fontproperties=font,fontsize=12,ha='center',color='#536B84')
fig.subplots_adjust(left=.01,right=.99,bottom=.01,top=.99);fig.savefig(Path(__file__).resolve().parents[1]/'output/equations/sensor_full_flow.png');plt.close(fig)
