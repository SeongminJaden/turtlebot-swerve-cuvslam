import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from pathlib import Path
out=Path('teaching/output/equations')
items={
 'active_tf':[(r'$ {}^{o}T_{b}(t) = {}^{o}T_{r}(t)\,\left({}^{b}T_{r}(q(t))\right)^{-1}$',.72),(r'$ {}^{b}T_{r}(q(t)) = \mathrm{FK}(q(t))\,{}^{e}T_{r}$',.33)],
 'active_reward':[(r'$r_t=w_p g_t\Delta s_t + w_a\frac{|\theta_{t-1}|-|\theta_t|}{\pi}+w_r I_{\mathrm{ready}}$',.79),(r'$\quad -w_b(1-g_t)\Vert v_t\Vert\Delta t-w_q\Vert\dot q_t\Vert^2\Delta t$',.49),(r'$\quad -w_t\Delta t-w_c I_{\mathrm{collision}}+w_g I_{\mathrm{success}}$',.19)]}
for name,lines in items.items():
 fig=plt.figure(figsize=(12,3.8),dpi=200,facecolor='white')
 for t,y in lines:fig.text(.04,y,t,fontsize=25,color='#243C61',va='center')
 fig.savefig(out/(name+'.png'),dpi=200);plt.close(fig)
