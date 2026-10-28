import json
from pathlib import Path
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
root=Path(__file__).resolve().parents[2];out=root/'teaching/output/equations';out.mkdir(exist_ok=True)
fig=plt.figure(figsize=(12,3.8),dpi=200,facecolor='white')
lines=[(r'$r_t=3(|e_{t-1}|-|e_t|)+0.2\,e^{-10e_t^2}\Delta t$',.78),(r'$\quad -0.01a_t^2\Delta t-0.005(a_t-a_{t-1})^2-0.02\Delta t+5I_{\mathrm{success}}$',.48),(r'$q_t^*=\operatorname{clip}(q_t+0.5a_t\Delta t,\,-\pi,\,\pi),\quad\Delta t=1/15\ \mathrm{s}$',.18)]
for text,y in lines:fig.text(.035,y,text,fontsize=24,color='#243C61',va='center')
fig.savefig(out/'camera_actual_reward.png');plt.close(fig)
r=json.loads((root/'runs/camera_ppo_v1/training.json').read_text());fig,ax=plt.subplots(1,2,figsize=(12,4),dpi=180,facecolor='white')
ax[0].plot([x['transitions'] for x in r],[x['mean_reward'] for x in r],color='#3267B1',lw=2);ax[0].set_ylabel('Mean reward / rollout step');ax[1].plot([x['transitions'] for x in r],[x['mean_action_std'] for x in r],color='#16816F',lw=2);ax[1].set_ylabel('Policy action standard deviation')
for a in ax:a.set_xlabel('Training transitions');a.grid(alpha=.2);a.spines[['top','right']].set_visible(False)
fig.tight_layout();fig.savefig(out/'camera_learning_curve.png');plt.close(fig)
