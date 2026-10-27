from pathlib import Path
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
fig=plt.figure(figsize=(12,4.4),dpi=180,facecolor='white')
lines=[r'$\Delta\theta_t=\mathrm{atan2}(\sin(\theta_t-\theta_{t-1}),\cos(\theta_t-\theta_{t-1}))$',r'$\widetilde\theta_t=\widetilde\theta_{t-1}+\Delta\theta_t,\quad |\omega_{\max}|\Delta t<\pi$',r'$n_{\mathrm{motor}}=\mathrm{round}\left(\frac{N\widetilde\theta}{2\pi g}\right),\quad N=4096,\ g=4$',r'$\widehat\omega_{\mathrm{wheel}}=\frac{2\pi g}{N}\frac{n_t-n_{t-k}}{t_t-t_{t-k}},\quad t_t-t_{t-k}\approx 0.03\ \mathrm{s}$',r'$(2\pi-0.05)\rightarrow(-2\pi+0.05):\quad \Delta\theta=+0.10\ \mathrm{rad}$']
for i,line in enumerate(lines):fig.text(.03,.88-i*.175,line,fontsize=20,color='#243C61',va='center')
fig.savefig(Path(__file__).resolve().parents[1]/'output/equations/encoder_unwrap.png');plt.close(fig)
