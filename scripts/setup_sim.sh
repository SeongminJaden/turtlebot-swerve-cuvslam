#!/usr/bin/env bash
# Rebuild the pinned Python runtime. Run from any working directory.
set -euo pipefail
SIM_ENV="${SIM_ENV:-$HOME/isaacsim6_venv}"
python3 -m venv "$SIM_ENV"
"$SIM_ENV/bin/python" -m pip install --no-cache-dir \
  'isaacsim[all,extscache]==6.1.0.0' --extra-index-url https://pypi.nvidia.com
"$SIM_ENV/bin/python" -c 'from importlib.metadata import version; print(version("isaacsim"))'
