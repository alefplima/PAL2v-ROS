# Reproducibility of experiments E1–E5

This guide redoes, from scratch, every number, table and figure of the paper. There are two paths:

- **A. Redo the analyses from the published data** (minutes): download the Zenodo package
  ([DATA.md](DATA.md)) and run the analysis scripts. It reproduces the tables exactly.
- **B. Redo the simulations** (~11 machine hours): run the batches. Gazebo and PX4 SITL are not bit-for-bit
  deterministic (scheduling, real-time factor), so re-run batches give statistically equivalent, but not
  identical, results. Sensor noise and faults, however, are generated with fixed seeds.

## 1. Environment used

| Item | Version |
| --- | --- |
| Machine | Intel Core i7-8565U laptop (4 cores/8 threads), one run/flight at a time |
| OS | Ubuntu 24.04.3 LTS, kernel 6.14 |
| ROS 2 | Rolling (Jan/2026 packages: `rclpy` 10.0.4, `ros_gz_sim` 3.0.5) |
| Python | system 3.12.3; numpy 1.26.4, scipy 1.11.4, matplotlib 3.6.3, Pillow 10.2.0 |
| E4 | Gazebo Jetty (gz-sim 10.0.0, the one shipped with ROS Rolling); TurtleBot3 Burger from `turtlebot3_gazebo` 2.3.7 (adjusted copy in the repository) |
| E5 | Gazebo Harmonic (gz-sim 8.10.0, `gz-harmonic` 1.0.0); PX4-Autopilot `9ac03f03eb` (v1.16.0-alpha2-441) with `patches/px4_dds_topics_vehicle_air_data.patch`; airframe 4016 `gz_x500_lidar_down`; Micro-XRCE-DDS Agent v2.4.3; `px4_msgs` `7a079b5` |

micromamba/conda must **not** be in PATH: ROS uses the system Python, and the conda protobuf breaks the PX4
build. `scripts/env.sh` cleans PATH and sources the environment.

Ubuntu 24.04 with ROS 2 Jazzy (which uses Gazebo Harmonic) should work and would simplify E5, but it was
**not tested**. With another version, record it together with the results.

## 2. Installation

```bash
git clone https://github.com/alefplima/PAL2v-ROS.git && cd PAL2v-ROS
bash install_ubuntu.sh              # ROS 2, ros_gz, Gazebo; colcon build; 26 core tests
bash scripts/setup_px4.sh           # E5 only: PX4 at the commit used + patch, XRCE-DDS agent, px4_msgs
```

Check: `source scripts/env.sh && ros2 pkg list | grep lpa2v` lists `lpa2v`, `lpa2v_msgs`, `lpa2v_ros`,
`lpa2v_sim` and `lpa2v_px4`, and `colcon test --packages-select lpa2v` gives 26 tests without failure.

## 3. Experiments, seeds and outputs

| Experiment | Command | Seeds | Time | Outputs (`experiments/results/`) |
| --- | --- | --- | --- | --- |
| E1 cost | `run_offline.sh` | `random.Random(0)` | 1 min | `e1_benchmark.json` |
| E2 offline fusion | `run_offline.sh` | evaluation 0–29 | 2 min | `e2_fusion.json`, `e2_table.md`, `e2_fusion.png` |
| E3 1-DOF altitude | `run_offline.sh` | tuning 100–104, evaluation 0–29 | 5 min | `e3_altitude.json`, `e3_table.md`, `e3_altitude.png` |
| E4 TurtleBot3 | `run_e4.sh` | 0–9 | ~6 h | `e4_runs/<scenario>/<method>_s<n>.csv`, `e4_gazebo.json`, `e4_table.md`, `e4_collisions.png`, `e4_series.png` |
| E5 PX4 SITL | `run_e5.sh` | 0–4 | ~5 h | `e5_runs/...csv`, `e5_runs/logs/`, `e5_px4.json`, `e5_table.md`, `e5_series.png`, `e5_failsafe_audit.md` |
| Captures | `run_prints.sh` (GUI) | 0 | ~40 min | `evidencias/*.png`, `evidencias/panel_*.png` |

The batches resume where they stopped: a run whose CSV already exists is skipped. Run the batches on an idle
machine, with no other Gazebo open.

## 4. From results to the paper

| Paper | Source |
| --- | --- |
| Cost of 65 µs per cycle | `e1_benchmark.json` |
| Offline validation (Section 5.1) | `e2_table.md` |
| Table 2 (E4, % with collision), detection and open loop | `e4_table.md`, `e4_gazebo.json` |
| Table 3 (E5, RMSE and ground touch), detection, Wilcoxon | `e5_table.md`, `e5_px4.json` |
| Figure 1 (lattice) and Figure 2 (architecture) | `paper/figures/make_figures.py` |
| Figure 3 | `e4_collisions.png` |
| Figures 4, 5, 7 and 8 (captures) | `run_prints.sh` → `evidence_panels.py` |
| Figure 6 | `e5_series.png` |
| E5 integrity (attempts, invalid flights) | `e5_runs/progresso.log`, `e5_failsafe_audit.md` |

The figures as used in the paper are in `paper/figures/final/`.

## 5. E5 integrity

- A flight whose PX4 log has a simulated-sensor timeout (`Accel/Gyro/Baro/Mag #0 fail`) is marked `INVALID`
  and redone, up to 3 attempts (`e5_px4.py`, `SIM_ARTIFACTS`). The published batch had 58 invalid flights in
  238 attempts, unrelated to the method, and no flight ran out of attempts.
- The invalid rate grows with CPU thermal throttling. With the CPU above ~85 °C and the clock stuck at
  ~1.2 GHz, almost every flight comes out invalid: let the machine cool down and keep it plugged in.
- After the batch, a second simulator warning showed up, `Preflight Fail: Battery unhealthy` →
  `Failsafe activated`. The batch criterion was **not** changed, so that the code remains the one that
  generated the data. `e5_audit_failsafe.py` audits those flights: the published batch had 9 flights with the
  warning; 7 had no mode change, landing or disarm, and in the remaining 2 (S6, IVW and ParaExtrctr) the
  warning came after the drone had already touched the ground.

## 6. Known environment issues

| Symptom | Cause | Fix in the repository |
| --- | --- | --- |
| PX4 does not see the Gazebo world | the command-line `gz` picks Ionic/Jetty instead of Harmonic | `px4_sitl.sh` sets `GZ_CONFIG_PATH` to the Harmonic .yaml files only |
| PX4 hangs in `PX4_GZ_STANDALONE` | v1.16-alpha bug (empty `gz_command`) | PX4 itself starts Gazebo |
| Drone does not take off in offboard | no ground station/radio | `px4_sitl.sh` sets `NAV_DLL_ACT`, `NAV_RCL_ACT`, `COM_RCL_EXCEPT` after rcS |
| x500 arms but does not leave the ground | `models/x500/model.sdf` edited locally | `git checkout` of the model (`setup_px4.sh` uses the clean commit) |
| apport warnings at every E4 run | `parameter_bridge` segfaults on SIGINT | `e4_gazebo.py` ends the group with SIGKILL |
| `ImportError: lpa2v` | workspace not sourced | `source scripts/env.sh` |

## 7. Glossary of identifiers kept in Portuguese

Scenario names, some file/directory names and the keys of the result JSON files keep the Portuguese
identifiers used when the data were produced, so that the code reads the published data unchanged.

| Identifier | Meaning |
| --- | --- |
| `S1_travado`, `S2_picos`, `S3_deriva`, `S4_dropout`, `S5_salto_gps`, `S6_duplo` | stuck sensor, impulsive spikes, drift, dropout, GPS jump, double (simultaneous) fault |
| `evidencias/`, `*_amplo.png`, `.anterior_<t>` | evidence (captures); wide view; previous (invalid) attempt |
| `progresso.log` | batch progress log. Lines written by v1.0 (published data) use `ok N amostras` (samples), `FALHOU` (failed), `INVALIDO (artefato do simulador: …)` (invalid, simulator artifact) |
| `*.falhou` | flight that did not take off |
| `"voo"`, `"armando"` (phase topic of `offboard_alt`) | flight, arming |
| JSON keys | `resumo` summary, `malha_aberta` open loop, `por_rodada`/`por_voo` per run/flight, `colisoes` collisions, `t_contato` time in contact, `percorrido` traveled distance, `rmse_falha` RMSE in the fault window, `deteccao` detection, `falso_alarme` false alarm, `toque_chao` ground touch, `desvio_max` maximum deviation, `media`/`mediana` mean/median |
