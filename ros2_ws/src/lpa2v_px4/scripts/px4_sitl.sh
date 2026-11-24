#!/usr/bin/env bash
# Starts Gazebo Harmonic (gz 8) + PX4 SITL (x500 with downward LiDAR) + uXRCE-DDS agent, in a CLEAN
# environment (without ROS/Jetty and without micromamba), because PX4 was built against the system Harmonic.
# Usage: px4_sitl.sh [gui 0|1] [gz partition]
# On exit (end of PX4, SIGINT/SIGTERM), it stops Gazebo and the agent: no orphan servers.

if [ -z "$PX4_SITL_CLEAN" ]; then
  GUI=${1:-0}
  PART=${2:-px4e5}
  # With Harmonic (8) and Ionic (9) installed, the command-line `gz` used by the PX4 scripts would pick
  # Ionic and not see the Harmonic server. Restricts the gz tools to the Harmonic configurations.
  GZCFG="$HOME/.gz_harmonic_config"
  mkdir -p "$GZCFG"
  for f in fuel9 gui8 launch7 model8 msgs10 plugin2 sdformat14 sim8 transport13 transportlog13 transportparam13; do
    ln -sf "/usr/share/gz/$f.yaml" "$GZCFG/"
  done
  exec env -i PX4_SITL_CLEAN=1 HOME="$HOME" USER="$USER" DISPLAY="${DISPLAY:-}" XAUTHORITY="${XAUTHORITY:-}" \
    TERM=dumb PATH=/usr/local/bin:/usr/bin:/bin LD_LIBRARY_PATH="$HOME/.local/lib" \
    GZ_PARTITION="$PART" GZ_CONFIG_PATH="$GZCFG" GUI="$GUI" \
    PX4_DIR="${PX4_DIR:-$HOME/PX4-Autopilot}" AGENT="${XRCE_AGENT:-$HOME/.local/bin/MicroXRCEAgent}" \
    bash "$0"
fi

B="$PX4_DIR/build/px4_sitl_default"
cleanup() {
  trap - EXIT INT TERM
  pkill -TERM -P $$ 2>/dev/null
  sleep 1
  pkill -KILL -P $$ 2>/dev/null
  # `gz sim` (ruby) creates the server as a grandchild: kill it by world + partition
  for p in $(pgrep -f "gz sim .*${PX4_GZ_WORLDS}/default.sdf") $(pgrep -fx "gz sim -g"); do
    [ "$(tr '\0' '\n' < /proc/$p/environ 2>/dev/null | grep -c "^GZ_PARTITION=$GZ_PARTITION$")" = 1 ] && kill -9 "$p"
  done
}
trap cleanup EXIT INT TERM

# PX4 itself starts Gazebo (normal mode; the PX4_GZ_STANDALONE mode of v1.16-alpha leaves `gz_command`
# empty and hangs waiting for the world). With GZ_CONFIG_PATH above, `gz sim` is Harmonic.
PX4_GZ_WORLDS="$PX4_DIR/Tools/simulation/gz/worlds"
[ "$GUI" = "1" ] || export HEADLESS=1
"$AGENT" udp4 -p 8888 >/dev/null 2>&1 &
cd "$B/rootfs" || exit 1
PX4_SYS_AUTOSTART=4016 PX4_SIM_MODEL=gz_x500_lidar_down PX4_GZ_WORLD=default \
  "$B/bin/px4" -d "$B/etc" &
PX4_PID=$!
# Autonomous flight via ROS 2, with no ground station or radio: disables the corresponding requirements.
# Applied only after rcS (the uXRCE-DDS client is the last to start), otherwise rcS overwrites it; verifies.
for i in $(seq 120); do
  "$B/bin/px4-uxrce_dds_client" status >/dev/null 2>&1 && break
  sleep 0.5
done
for i in $(seq 10); do
  "$B/bin/px4-param" set NAV_DLL_ACT 0 >/dev/null 2>&1
  "$B/bin/px4-param" set NAV_RCL_ACT 0 >/dev/null 2>&1
  "$B/bin/px4-param" set COM_RCL_EXCEPT 4 >/dev/null 2>&1
  "$B/bin/px4-param" show NAV_DLL_ACT 2>/dev/null | grep -q "NAV_DLL_ACT.*: 0" && { echo "LPA2V: parametros aplicados"; break; }
  sleep 1
done
wait $PX4_PID
