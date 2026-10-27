"""Plot measured run evidence; never substitutes estimated arrival for truth."""
import argparse,json
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Circle
p=argparse.ArgumentParser();p.add_argument('directory',type=Path);a=p.parse_args()
r=json.loads((a.directory/'report.json').read_text());rows=r['rows'];m=json.loads(Path('assets/maze/maze.json').read_text());route=np.array(json.loads(Path(r.get('route_file','assets/maze/route_footprint.json')).read_text())['route'])
xy=np.array([x['base_xy'] for x in rows]);nav=np.array([x['nav_xy'] or [np.nan,np.nan] for x in rows]);t=np.array([x['sim_s'] for x in rows]);speed=np.array([x['speed_m_s'] for x in rows])
fig=plt.figure(figsize=(12,6),dpi=180,facecolor='white');gs=fig.add_gridspec(2,2,width_ratios=[1.3,1],hspace=.45,wspace=.35);ax=fig.add_subplot(gs[:,0])
for w in m['panels']:
 c=np.array(w['center']);u=np.array([np.cos(w['yaw']),np.sin(w['yaw'])])*w['length']/2
 ax.plot([c[0]-u[0],c[0]+u[0]],[c[1]-u[1],c[1]+u[1]],color='#34465c',lw=2)
for c in m['posts']:ax.add_patch(Circle(c,.03,color='#34465c'))
ax.plot(route[:,0],route[:,1],'--',color='#a4b0c0',label='Planned route',lw=1.5);ax.plot(xy[:,0],xy[:,1],color='#198a76',label='Actual PhysX',lw=2);ax.plot(nav[:,0],nav[:,1],color='#d29437',label='Estimated',lw=1,alpha=.9);ax.scatter(*route[0],s=45,color='#254f8d',zorder=5);ax.scatter(*route[-1],s=80,marker='*',color='#8c5fb5',zorder=5);ax.set(xlim=(-.1,4.5),ylim=(-.1,3.3),aspect='equal',xlabel='x (m)',ylabel='y (m)',title='Maze trajectory');ax.legend(fontsize=8,loc='upper right')
b=fig.add_subplot(gs[0,1]);b.plot(t,speed,color='#198a76',label='Actual speed');b.plot(t,[x['command_m_s'] for x in rows],color='#a6b6c7',lw=.8,label='Policy + limits');b.axhline(.95,color='#bd5461',ls='--',label='0.95 m/s maximum');b.set(xlabel='Simulation time (s)',ylabel='Speed (m/s)',ylim=(0,1.05),title='Speed at controller sample times');b.legend(fontsize=8)
c=fig.add_subplot(gs[1,1]);c.plot(t,np.linalg.norm(xy-nav,axis=1)*100,color='#d29437');c.set(xlabel='Simulation time (s)',ylabel='XY estimation error (cm)',title='Evaluation-only truth comparison')
for aa in [ax,b,c]:aa.spines[['top','right']].set_visible(False);aa.grid(alpha=.12)
fig.suptitle(f"{'PASS' if r['validated_success'] else 'FAIL'} | {r['sim_seconds']:.2f} s | goal error {r['actual_goal_error_m']*100:.1f} cm | non-wheel contact {r['peak_nonwheel_contact_force_N']:.1f} N",fontsize=13,color='#243c61')
fig.savefig(a.directory/'validation-chart.png',bbox_inches='tight');plt.close(fig)
