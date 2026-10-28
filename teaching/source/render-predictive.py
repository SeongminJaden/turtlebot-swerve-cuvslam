from pathlib import Path
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
out=Path(__file__).resolve().parents[1]/'output/equations/predictive_actual.png'
fig=plt.figure(figsize=(12,4.4),dpi=200,facecolor='white')
lines=[
 r'$v_t^*=\mathrm{clip}(v_{t-1}^*+0.6a_t\Delta t,0,0.95),\quad q_t^*=\mathrm{clip}(q_t+0.5b_t\Delta t,-\pi,\pi)$',
 r'$r_t=8\Delta s+0.25(|e_{t-1}|-|e_t|)-0.08\Delta t-0.005(a_t^2+b_t^2)\Delta t$',
 r'$\quad -0.01(a_t-a_{t-1})^2+5I_{\mathrm{success}}-5I_{\mathrm{failure}}$',
 r'$\quad -2\max(v_t^2/1.2+0.18-d_{\mathrm{seen}},0)\Delta t-2(q_x^2+q_y^2)\Delta t/0.09$',
 r'$\quad -0.3|e_t|\Delta t-0.15d_{\mathrm{cross}}\Delta t$',
 r'$v_{\mathrm{safe}}=\sqrt{2(0.6)\max(d_{\mathrm{seen}}-0.18,0)},\quad v_{\mathrm{cmd}}=\min(v_t^*,v_{\mathrm{safe}})$',
 r'$a,b\in[-1,1],\quad\Delta t=1/15\ \mathrm{s},\quad e=q_{\mathrm{preview}}-q,\quad q_x,q_y:\ \mathrm{body\ quaternion}$'
]
for i,line in enumerate(lines):fig.text(.025,.89-i*.125,line,fontsize=16.5,color='#243C61',va='center')
fig.savefig(out);plt.close(fig)
