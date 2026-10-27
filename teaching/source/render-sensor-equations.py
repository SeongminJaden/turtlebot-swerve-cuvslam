from pathlib import Path
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
out=Path(__file__).resolve().parents[1]/'output/equations'
sets={
 'dual_imu_origin':[
 r'$\boldsymbol{\omega}_i^B=R_{BS_i}\boldsymbol{\omega}_i^{S_i},\qquad \mathbf{f}_i^B=R_{BS_i}\mathbf{f}_i^{S_i}$',
 r'$\mathbf{f}_{O,i}=\mathbf{f}_i^B-\boldsymbol{\alpha}\times\mathbf{r}_i-\boldsymbol{\omega}\times(\boldsymbol{\omega}\times\mathbf{r}_i)$',
 r'$\|\boldsymbol{\omega}_1^B-\boldsymbol{\omega}_2^B\|\leq 0.35,\quad \|\mathbf{f}_{O,1}-\mathbf{f}_{O,2}\|\leq 2.0$',
 r'$\overline{\boldsymbol{\omega}}=(\boldsymbol{\omega}_1^B+\boldsymbol{\omega}_2^B)/2,\qquad \overline{\mathbf{f}}_O=(\mathbf{f}_{O,1}+\mathbf{f}_{O,2})/2$',
 r'$\mathbf{f}:\ \mathrm{specific\ force\ (gravity\ retained)},\quad \mathbf{r}_i:\ O\rightarrow S_i$',
 ],
 'camera_ekf_frames':[
 r'$T_{VB}(t)=T_{VC}(t)\,T_{BC}(q(t))^{-1}$',
 r'$T_{OB}(t)=T_{VB}(t_0)^{-1}\,T_{VB}(t)$',
 r'$T_{MB}(t)=T_{MO}^{\mathrm{AMCL}}(t)\,T_{OB}^{\mathrm{EKF}}(t)$',
 r'$\mathbf{z}_{\mathrm{wheel}}=[v_x,v_y],\quad \mathbf{z}_{\mathrm{IMU}}=[\omega_z],\quad \mathbf{z}_{\mathrm{VO}}=[\Delta x,\Delta y]/\Delta t$',
 r'$V:\ \mathrm{VO\ origin},\quad O:\ \mathrm{initial\ base},\quad M:\ \mathrm{CAD\ map}$',
 ],
 'policy_deployment_limits':[
 r'$v_t^*=\mathrm{clip}(v_{t-1}^{\mathrm{cmd}}+0.6a_t\Delta t,0,0.95)$',
 r'$v_i^{\mathrm{curve}}=\min(0.92,0.6/\max(|\kappa_i|,10^{-6}))$',
 r'$v_i^{\mathrm{path}}=\min(v_i^{\mathrm{curve}},\sqrt{(v_{i+1}^{\mathrm{path}})^2+2(0.4)\Delta s_i})$',
 r'$v_t^{\mathrm{cmd}}=\min(v_t^*,v^{\mathrm{path}},\sqrt{1.2\max(d_{\mathrm{seen}}-0.32,0)})$',
 r'$|\Delta\omega_{\mathrm{wheel}}|\leq (0.6/R)\Delta t,\quad R=0.0306872\ \mathrm{m}$',
 ]}
for name,lines in sets.items():
 fig=plt.figure(figsize=(12,4.4),dpi=180,facecolor='white')
 for i,line in enumerate(lines):fig.text(.03,.88-i*.175,line,fontsize=20,color='#243C61',va='center')
 fig.savefig(out/(name+'.png'));plt.close(fig)
