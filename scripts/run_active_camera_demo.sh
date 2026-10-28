#!/usr/bin/env bash
# Live lockstep simulation; only launch one instance (localhost ports 18766/18767).
set -eo pipefail
cd "$(dirname "$0")/.."
source /opt/ros/lyrical/setup.bash
set -u
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-88}"
export OMNI_KIT_ACCEPT_EULA=YES
out="${1:-evidence/2026-10-26/active-integration-v3}"
mkdir -p "$out"
children=()
cleanup() { for child in "${children[@]}"; do kill -TERM "$child" 2>/dev/null || true; done; }
trap cleanup EXIT
ros2 launch ./launch/cuvslam_arm_camera.launch.py > "$out/cuvslam.log" 2>&1 & children+=("$!")
/usr/bin/python3 scripts/active_camera_bridge.py --directory "$out" --pose-quality-gate > "$out/bridge.log" 2>&1 & children+=("$!")
render_args=()
if [[ "${TEXTURED_FLOOR:-0}" == 1 ]]; then render_args+=(--textured-floor); fi
/home/seongmin/isaacsim6_venv/bin/python scripts/active_camera_renderer.py --directory "$out" "${render_args[@]}" > "$out/renderer.log" 2>&1 & children+=("$!")
for ((i=0;i<120;i++)); do
 if rg -q RENDERER_READY "$out/renderer.log"; then break; fi
 if ! kill -0 "${children[2]}" 2>/dev/null; then cat "$out/renderer.log"; exit 1; fi
 sleep 1
done
rg -q RENDERER_READY "$out/renderer.log"
/home/seongmin/IsaacLab/.venv/bin/python scripts/lab_python.py rl/evaluate_active_camera.py --output "$out" --steps 2400 --drop-step 100 --visualizer none > "$out/evaluation.log" 2>&1
cat "$out/report.json"
