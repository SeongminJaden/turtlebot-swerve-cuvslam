"""Extract official apt binaries into a user-local prefix without changing ROS."""
from pathlib import Path
import json,subprocess,hashlib
root=Path(__file__).resolve().parents[1]/'build/nvblox-runtime'
packages=['ros-lyrical-nvblox-ros','ros-lyrical-nvblox-nav2','ros-lyrical-nav2-planner','ros-lyrical-nav2-navfn-planner','ros-lyrical-nav2-lifecycle-manager']
plan=subprocess.check_output(['apt-get','-s','install',*packages],text=True)
names=list(dict.fromkeys(packages+[line.split()[1] for line in plan.splitlines() if line.startswith('Inst ')]))
for d in ['archives','prefix']:(root/d).mkdir(parents=True,exist_ok=True)
subprocess.run(['apt-get','download',*names],cwd=root/'archives',check=True)
manifest=[]
for f in (root/'archives').glob('*.deb'):
    subprocess.run(['dpkg-deb','-x',str(f),str(root/'prefix')],check=True)
    manifest.append({'file':f.name,'sha256':hashlib.sha256(f.read_bytes()).hexdigest()})
(root/'package-manifest.json').write_text(json.dumps(manifest,indent=2))
print('Source scripts/nvblox_env.sh after ROS and robot_localization_env.sh')
