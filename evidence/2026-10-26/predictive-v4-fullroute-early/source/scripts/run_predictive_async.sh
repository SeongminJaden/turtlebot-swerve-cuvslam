#!/usr/bin/env bash
set -eo pipefail
cd "$(dirname "$0")/.."
exec 9>build/predictive-runtime.lock
flock -n 9 || { echo "Another predictive runtime owns the camera bridge" >&2; exit 1; }
source /opt/ros/lyrical/setup.bash
set -u
export OMP_NUM_THREADS="${OMP_NUM_THREADS:-1}"
export OPENBLAS_NUM_THREADS="${OPENBLAS_NUM_THREADS:-1}"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-$((100 + $$ % 100))}"
export OMNI_KIT_ACCEPT_EULA=YES
out="${1:-evidence/2026-10-26/predictive-async}"
mkdir -p "$out"
children=()
cleanup(){ for child in "${children[@]}"; do kill -INT -- "-$child" 2>/dev/null || true; done; }
trap cleanup EXIT
setsid ros2 launch ./launch/cuvslam_arm_camera.launch.py > "$out/cuvslam.log" 2>&1 & children+=("$!")
setsid /usr/bin/python3 scripts/active_camera_bridge.py --directory "$out" --external-clock --pose-quality-gate --width "${CAMERA_WIDTH:-640}" > "$out/bridge.log" 2>&1 & children+=("$!")
setsid /home/seongmin/isaacsim6_venv/bin/python scripts/active_camera_renderer.py --directory "$out" --async-mode --textured-floor --width "${CAMERA_WIDTH:-640}" --headless > "$out/renderer.log" 2>&1 & children+=("$!")
for ((i=0;i<120;i++)); do
 if rg -q RENDERER_READY "$out/renderer.log"; then break; fi
 if ! kill -0 "${children[1]}" 2>/dev/null; then exit 1; fi
 if ! kill -0 "${children[2]}" 2>/dev/null; then exit 1; fi
 sleep 1
done
rg -q RENDERER_READY "$out/renderer.log"
setsid /usr/bin/python3 scripts/isaac_sensor_bridge.py --udp-port 18768 --directory "$out" > "$out/sensors.log" 2>&1 & children+=("$!")
setsid /usr/bin/python3 -c 'from turtlebot_swerve.dual_imu_node import main; main()' --ros-args --params-file config/dual_imu_sim.yaml > "$out/fusion.log" 2>&1 & children+=("$!")
setsid /usr/bin/python3 scripts/robot_observation_adapter.py --directory "$out" > "$out/observations.log" 2>&1 & children+=("$!")
/home/seongmin/IsaacLab/.venv/bin/python scripts/lab_python.py rl/run_predictive_async.py --output "$out" --seconds "${DURATION:-240}" --checkpoint "${CHECKPOINT:-runs/predictive_ppo_v3/policy.pt}" --visualizer none > "$out/evaluation.log" 2>&1
cat "$out/report.json"
