#!/usr/bin/env bash
# Installs ROS 2 (Rolling, tested), ros_gz, Gazebo Harmonic (E5) and builds the PAL2v-ROS workspace.
# Usage (native Ubuntu or WSL):  bash install_ubuntu.sh
set -euo pipefail
. /etc/os-release
# Tested: Ubuntu 24.04 + ROS 2 Rolling (see REPRODUCIBILITY.md). Another distro: ROS_DISTRO=jazzy bash install_ubuntu.sh
[ "$VERSION_ID" = "24.04" ] || echo "WARNING: tested only on Ubuntu 24.04 (found $VERSION_ID)"
ROS_DISTRO=${ROS_DISTRO:-rolling}
echo ">> Ubuntu $VERSION_ID -> ROS 2 $ROS_DISTRO"
sudo apt update && sudo apt install -y software-properties-common curl git python3-pip python3-venv \
  python3-numpy python3-scipy python3-matplotlib
sudo add-apt-repository -y universe
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $UBUNTU_CODENAME main" \
  | sudo tee /etc/apt/sources.list.d/ros2.list >/dev/null
sudo apt update
sudo apt install -y ros-$ROS_DISTRO-desktop ros-dev-tools python3-colcon-common-extensions \
  ros-$ROS_DISTRO-ros-gz python3-pil
# E5: Gazebo Harmonic (the PX4 of the experiment was built against it)
sudo curl -sSL https://packages.osrfoundation.org/gazebo.gpg -o /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $UBUNTU_CODENAME main" \
  | sudo tee /etc/apt/sources.list.d/gazebo-stable.list >/dev/null
sudo apt update && sudo apt install -y gz-harmonic
grep -q "/opt/ros/$ROS_DISTRO/setup.bash" ~/.bashrc || echo "source /opt/ros/$ROS_DISTRO/setup.bash" >> ~/.bashrc
HERE="$(cd "$(dirname "$0")" && pwd)"
# the lpa2v core is a workspace package: colcon builds it along (no pip needed)
# workspace ROS 2
source /opt/ros/$ROS_DISTRO/setup.bash
cd "$HERE/ros2_ws" && colcon build --symlink-install
# core validation (should give 26 tests OK)
(cd "$HERE/lpa2v_core" && python3 -m unittest discover -s tests)
echo ">> Done. In a new terminal:  source $HERE/ros2_ws/install/setup.bash"
echo ">> Teste: ros2 launch lpa2v_ros altitude_demo.launch.py   |   Experiments: REPRODUCIBILITY.md"
