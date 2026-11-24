#!/usr/bin/env bash
# Prepares PX4 SITL exactly as in E5: PX4-Autopilot at the commit used, with the DDS topics patch,
# Micro-XRCE-DDS agent v2.4.3 and px4_msgs at the commit used. Usage: bash scripts/setup_px4.sh
# Prerequisite: Gazebo Harmonic (gz-harmonic) installed; see REPRODUCIBILITY.md.
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
PX4_DIR="${PX4_DIR:-$HOME/PX4-Autopilot}"
PX4_COMMIT=9ac03f03ebd888fd06093e7f0c2e8cacbb2ee83d      # v1.16.0-alpha2-441-g9ac03f03eb
PX4_MSGS_COMMIT=7a079b5dc1987ad68912c4d53282d6679215b2ca
XRCE_TAG=v2.4.3
# clean PATH: the micromamba/conda protobuf breaks the PX4 build
export PATH=$(echo "$PATH" | tr ':' '\n' | grep -viE "micromamba|conda" | paste -sd:)
unset CONDA_PREFIX PYTHONPATH

# 1. PX4-Autopilot
[ -d "$PX4_DIR" ] || git clone https://github.com/PX4/PX4-Autopilot.git "$PX4_DIR"
cd "$PX4_DIR"
git fetch --tags origin
git checkout "$PX4_COMMIT"
git submodule update --init --recursive
git apply --check "$ROOT/patches/px4_dds_topics_vehicle_air_data.patch" 2>/dev/null \
  && git apply "$ROOT/patches/px4_dds_topics_vehicle_air_data.patch" || echo ">> patch already applied"
bash Tools/setup/ubuntu.sh --no-nuttx
python3 -m pip install --user --break-system-packages --no-deps kconfiglib pyros-genmsg cerberus 2>/dev/null || true
make px4_sitl

# 2. Micro-XRCE-DDS agent (installed in ~/.local)
SRC="$HOME/Micro-XRCE-DDS-Agent"
[ -d "$SRC" ] || git clone -b "$XRCE_TAG" https://github.com/eProsima/Micro-XRCE-DDS-Agent.git "$SRC"
cmake -S "$SRC" -B "$SRC/build" -DCMAKE_INSTALL_PREFIX="$HOME/.local"
cmake --build "$SRC/build" -j"$(nproc)" && cmake --install "$SRC/build"

# 3. px4_msgs (separate workspace)
WS="${PX4_ROS_WS:-$HOME/px4_ros_ws}"
mkdir -p "$WS/src"
[ -d "$WS/src/px4_msgs" ] || git clone https://github.com/PX4/px4_msgs.git "$WS/src/px4_msgs"
git -C "$WS/src/px4_msgs" checkout "$PX4_MSGS_COMMIT"
source /opt/ros/${ROS_DISTRO:-rolling}/setup.bash
(cd "$WS" && colcon build)
echo ">> PX4 ready. Test: source scripts/env.sh px4 && ros2 launch lpa2v_px4 e5.launch.py method:=pal2v scenario:=S0_nominal"
