# Source after /opt/ros/lyrical/setup.bash. User-local official binary packages.
robot_localization_prefix="$(cd "$(dirname "${BASH_SOURCE[0]}")/../build/robot-localization-runtime/prefix" && pwd)"
export AMENT_PREFIX_PATH="$robot_localization_prefix/opt/ros/lyrical:${AMENT_PREFIX_PATH:-}"
export LD_LIBRARY_PATH="$robot_localization_prefix/opt/ros/lyrical/lib:$robot_localization_prefix/usr/lib/x86_64-linux-gnu:$robot_localization_prefix/usr/lib:${LD_LIBRARY_PATH:-}"
export PYTHONPATH="$robot_localization_prefix/opt/ros/lyrical/lib/python3.12/site-packages:${PYTHONPATH:-}"
