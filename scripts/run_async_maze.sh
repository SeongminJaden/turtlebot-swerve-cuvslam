#!/usr/bin/env bash
set -eo pipefail
cd "$(dirname "$0")/.."
source /opt/ros/lyrical/setup.bash
set -u
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-91}"
export OMNI_KIT_ACCEPT_EULA=YES
out="${1:-evidence/2026-10-26/async-maze-baseline}"
mkdir -p "$out"
children=()
cleanup(){ for child in "${children[@]}"; do kill -TERM "$child" 2>/dev/null || true; done; }
trap cleanup EXIT
ros2 launch ./launch/cuvslam_arm_camera.launch.py > "$out/cuvslam.log" 2>&1 & children+=("$!")
/usr/bin/python3 scripts/active_camera_bridge.py --directory "$out" --pose-quality-gate --width 320 > "$out/bridge.log" 2>&1 & children+=("$!")
/home/seongmin/isaacsim6_venv/bin/python scripts/active_camera_renderer.py --directory "$out" --async-mode --textured-floor --width 320 --headless > "$out/renderer.log" 2>&1 & children+=("$!")
for ((i=0;i<120;i++)); do
 if rg -q RENDERER_READY "$out/renderer.log"; then break; fi
 if ! kill -0 "${children[2]}" 2>/dev/null; then exit 1; fi
 sleep 1
done
rg -q RENDERER_READY "$out/renderer.log"
/home/seongmin/IsaacLab/.venv/bin/python scripts/lab_python.py rl/async_maze_camera.py --output "$out" --seconds "${DURATION:-240}" --max-speed .95 --visualizer none > "$out/evaluation.log" 2>&1
cat "$out/report.json"
