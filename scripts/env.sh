# Environment of the experiments (usage: source scripts/env.sh [px4]). Tested on Ubuntu 24.04 + ROS 2 Rolling.
# Removes micromamba/conda from PATH (ROS uses the system Python) and sources ROS, px4_msgs and the workspace.
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
export PATH=$(echo "$PATH" | tr ':' '\n' | grep -viE "micromamba|conda" | paste -sd:)
unset CONDA_PREFIX PYTHONPATH
source /opt/ros/${ROS_DISTRO:-rolling}/setup.bash
[ "$1" = "px4" ] && source "${PX4_ROS_WS:-$HOME/px4_ros_ws}/install/setup.bash"
source "$ROOT/ros2_ws/install/setup.bash"
