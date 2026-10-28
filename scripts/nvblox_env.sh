# Source after ROS and robot_localization_env.sh. Official user-local binaries.
nvblox_prefix="$(cd "$(dirname "${BASH_SOURCE[0]}")/../build/nvblox-runtime/prefix" && pwd)"
export AMENT_PREFIX_PATH="$nvblox_prefix/opt/ros/lyrical:${AMENT_PREFIX_PATH:-}"
export LD_LIBRARY_PATH="$nvblox_prefix/opt/ros/lyrical/lib:$nvblox_prefix/usr/lib/x86_64-linux-gnu:${LD_LIBRARY_PATH:-}"
export PYTHONPATH="$nvblox_prefix/opt/ros/lyrical/lib/python3.12/site-packages:${PYTHONPATH:-}"
