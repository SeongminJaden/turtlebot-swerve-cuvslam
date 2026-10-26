"""Run a script with Lab dependencies and an optional separate Sim installation."""
import os
from pathlib import Path
import runpy
import site
import sys
sim_site=Path(os.environ.get('ISAAC_SIM_SITE', str(Path.home()/'isaacsim6_venv/lib/python3.12/site-packages')))
if sim_site.is_dir():
    site.addsitedir(str(sim_site))
if len(sys.argv)<2:
    raise SystemExit('Usage: <IsaacLab Python> scripts/lab_python.py <script.py> [arguments]')
script=sys.argv[1]
sys.argv=sys.argv[1:]
sys.path.insert(0,os.path.dirname(os.path.abspath(script)))
runpy.run_path(script,run_name='__main__')
