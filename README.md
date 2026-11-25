# PAL2v-ROS: Paraconsistent Annotated Logic for ROS 2

[![DOI](https://zenodo.org/badge/DOI/10.5281/zenodo.24544260.svg)](https://doi.org/10.5281/zenodo.24544260)

A library of the **Paraconsistent Annotated Logic with two-valued annotation (PAL2v, "LPA2v" in Portuguese)**
for perception and control in mobile robots and drones. The core is pure Python with no dependencies, and
ready-to-use ROS 2 nodes are included. The repository also holds every experiment of the paper
*"Paraconsistent fusion of redundant sensors with PAL2v logic in ROS 2"* (E1–E5), with the scripts that
generate each table and figure.

## Components
| Package (`ros2_ws/src/`) | Contents |
|---|---|
| `lpa2v_core` (Python package `lpa2v`) | Paraconsistent Analysis Node (PAN, "NAP") with the indefiniteness rule (φE ≤ 0.25 or d > 1), 12-state Para-Analyzer, PANCd decision, ParaExtrctr, PAL2v learning cell/filter, **PAL2vFusion** (PAN + ParaExtrctr network; `mode="graded"` attenuates contradictory sources by their tolerance interval instead of excluding them, `mode="step"` is the v1.0.0 limit case) and the **PAL2v-PI** controller. Pure Python, 32 tests. |
| `lpa2v_msgs` | messages `Evidence`, `PANState`, `FusionState` |
| `lpa2v_ros` | nodes `para_analyzer`, `pal2v_fusion` (state on `/diagnostics`), `pal2v_filter`, `pal2v_controller`, `altitude_sim` |
| `lpa2v_sim` | E4: TurtleBot3 in Gazebo, arena, sensors with injected faults, baseline methods, logger |
| `lpa2v_px4` | E5: PX4 SITL (x500 + downward LiDAR), altitude sensors, offboard control, logger |

The package names keep the `lpa2v` prefix used since the first version. `lpa2v_core/` at the root is a
shortcut to `ros2_ws/src/lpa2v_core/`.

## How the library is loaded in ROS 2
The `lpa2v` core is an `ament_python` package of the workspace itself. `colcon build` builds and installs it
together with the others, and `lpa2v_ros`, `lpa2v_sim` and `lpa2v_px4` declare it as `<exec_depend>`. The
nodes only do `from lpa2v import PAL2vFusion`; all the logic lives in the core, which is tested outside ROS.

```bash
cd ros2_ws && colcon build --symlink-install && source install/setup.bash
# E4 limits: PAL2vFusion.from_sigma([0.05, 0.02, 0.08], 0.0, 3.5, max_rate=1.0, dt=0.05)
ros2 run lpa2v_ros pal2v_fusion --ros-args -p input_topics:="[sonar, lidar, ir]" \
  -p vmin:=0.0 -p vmax:=3.5 -p ct_lim:="[0.1078, 0.0942, 0.1078]" -p rate_lim:="[0.0951, 0.0466, 0.1436]"
```
The experiment launch files compute `ct_lim` and `rate_lim` with `from_sigma` (see `lpa2v_sim/launch/e4.launch.py`).

Outside ROS, the same package installs with pip:
```bash
pip install ./lpa2v_core
```
```python
from lpa2v import PAL2vFusion
fus = PAL2vFusion.from_sigma([0.05, 0.02, 0.08], vmin=0.0, vmax=4.0, max_rate=0.6, dt=0.02, mode="graded")
fus.update([2.00, 2.00, 2.00])
out = fus.update([2.01, 3.99, 2.03])          # sensor 1 in dropout
print(out.value, out.rejected, out.labels)    # 2.02 [1] ['t', '⊤', 't']
```

## Reproducing the paper
Step by step, exact versions and the time of each stage: **[REPRODUCIBILITY.md](REPRODUCIBILITY.md)**.
Raw data (CSVs of every run and flight, PX4 logs, captures): **[DATA.md](DATA.md)** (Zenodo).

```bash
bash install_ubuntu.sh                  # ROS 2 + Gazebo + workspace (Ubuntu 24.04)
bash experiments/run_offline.sh         # E1–E3, no ROS (minutes)
bash experiments/run_e4.sh              # E4, TurtleBot3 in Gazebo (~6 h)
bash scripts/setup_px4.sh               # PX4 at the commit used + patch
bash experiments/run_e5.sh              # E5, PX4 SITL (~5 h)
```

## How to cite
Software: DOI [10.5281/zenodo.24544260](https://doi.org/10.5281/zenodo.24544260) (version 1.1.0, used in the paper; version 1.0.0: [10.5281/zenodo.24499334](https://doi.org/10.5281/zenodo.24499334));
[10.5281/zenodo.24499333](https://doi.org/10.5281/zenodo.24499333) always resolves to the latest version.
See [CITATION.cff](CITATION.cff) (GitHub: "Cite this repository").

## License
Apache-2.0 (see [LICENSE](LICENSE) and [NOTICE](NOTICE)). The TurtleBot3 model in
`ros2_ws/src/lpa2v_sim/models/turtlebot3_lpa` derives from ROBOTIS `turtlebot3_gazebo` 2.3.7 (Apache-2.0).

Author: Alessandro Ferreira Paz Lima (Universidade Santa Cecília – UNISANTA, Brazil).
