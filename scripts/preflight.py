"""Read-only environment report. Missing optional stages stay explicit."""
import argparse
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys

p = argparse.ArgumentParser()
p.add_argument('--sim-python', default=str(Path.home()/'isaacsim6_venv/bin/python'))
p.add_argument('--output', type=Path, default=Path('checks/preflight.json'))
a = p.parse_args()
checks = []


def check(name, command):
    try:
        result = subprocess.run(command, capture_output=True, text=True, timeout=30)
        checks.append({'name': name, 'command': command, 'passed': result.returncode == 0,
                       'exit_code': result.returncode, 'output': result.stdout + result.stderr})
    except (OSError, subprocess.TimeoutExpired) as e:
        checks.append({'name': name, 'command': command, 'passed': False, 'output': str(e)})


check('GPU driver', ['nvidia-smi', '--query-gpu=name,driver_version,memory.total', '--format=csv'])
check('Pinned Isaac Sim package', [a.sim_python, '-c',
      'from importlib.metadata import version; v=version("isaacsim"); print(v); assert v=="6.1.0.0"'])
check('ROS environment', ['ros2', 'pkg', 'prefix', 'rclpy'])
check('cuVSLAM package', ['ros2', 'pkg', 'prefix', 'isaac_ros_cuvslam'])
report = {'scope': 'Package availability only, not runtime or tracking success',
          'ros_distro': os.environ.get('ROS_DISTRO'), 'free_bytes': shutil.disk_usage('.').free,
          'checks': checks}
a.output.parent.mkdir(parents=True, exist_ok=True)
a.output.write_text(json.dumps(report, ensure_ascii=False, indent=2))
print(json.dumps(report, ensure_ascii=False, indent=2))
sys.exit(0 if all(x['passed'] for x in checks) else 1)
