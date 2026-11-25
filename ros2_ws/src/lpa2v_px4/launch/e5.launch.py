"""E5: PX4 SITL (x500 + downward LiDAR) in Gazebo, altitude by fusion of baro + LiDAR + GPS under faults.

  ros2 launch lpa2v_px4 e5.launch.py method:=pal2v scenario:=S4_dropout gui:=true

Requires: ~/PX4-Autopilot built (make px4_sitl), MicroXRCEAgent in ~/.local/bin and px4_msgs sourced
(see scripts/setup_px4.sh).
method: pal2v (step, v1.0) | pal2v_g (graded, no exclusion) | mean | median | ivw | paraextrctr | kf_chi2
scenario: S0_nominal | S1_travado | S3_deriva | S4_dropout | S5_salto_gps | S6_duplo
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, EmitEvent, ExecuteProcess, OpaqueFunction,
                            RegisterEventHandler)
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration as LC
from launch_ros.actions import Node

from lpa2v import PAL2vFusion

VMAX = 10.0                     # altitude UD [0, 10] m (same as E3)
FS = 50.0                       # fusion rate [Hz]
KF_Q = 30.0                     # kf_chi2: tuned on the nominal flight only (same criterion as E4)


def _floats(s):
    return [float(v) for v in s.split(",")]


def _setup(context):
    a = {k: LC(k).perform(context) for k in
         ("method", "scenario", "seed", "duration", "out", "gui", "sigma", "max_rate", "auto_shutdown",
          "fault_lidar", "fault_gps", "partition")}
    share = get_package_share_directory("lpa2v_px4")
    sigma = _floats(a["sigma"])                 # baro, LiDAR, GPS [m] (estimated in the nominal flight)
    topics = ["/e5/baro", "/e5/lidar", "/e5/gps"]
    if a["method"] in ("pal2v", "pal2v_g"):
        mode = "graded" if a["method"] == "pal2v_g" else "step"
        f = PAL2vFusion.from_sigma(sigma, 0.0, VMAX, max_rate=float(a["max_rate"]), dt=1.0 / FS)
        fusion = Node(package="lpa2v_ros", executable="pal2v_fusion", name="e5_fusion",
                      parameters=[{"input_topics": topics, "input_type": "float64", "vmin": 0.0,
                                   "vmax": VMAX, "ct_lim": f.ct_lim, "rate_lim": f.rate_lim,
                                   "rate_hz": FS, "mode": mode}],
                      remappings=[("fused", "/e5/fused"), ("fusion_state", "/e5/fusion_state")])
    else:
        fusion = Node(package="lpa2v_sim", executable="baseline_fusion", name="e5_fusion",
                      parameters=[{"method": a["method"], "input_topics": topics, "sigma": sigma,
                                   "rate_hz": FS, "vmin": 0.0, "vmax": VMAX, "kf_q": KF_Q}],
                      remappings=[("fused", "/e5/fused"), ("fusion_state", "/e5/fusion_state")])
    logger = Node(package="lpa2v_px4", executable="e5_logger",
                  parameters=[{"out": a["out"], "duration": float(a["duration"]), "rate_hz": 20.0}])
    gui = "1" if a["gui"].lower() in ("1", "true", "yes") else "0"
    end = ([RegisterEventHandler(OnProcessExit(target_action=logger, on_exit=[EmitEvent(event=Shutdown())]))]
           if a["auto_shutdown"].lower() in ("1", "true", "yes") else [])
    return [
        ExecuteProcess(cmd=["bash", os.path.join(share, "scripts", "px4_sitl.sh"), gui, a["partition"]],
                       output="log"),
        Node(package="lpa2v_px4", executable="alt_sensors",
             parameters=[{"seed": int(a["seed"]), "scenario": a["scenario"],
                          "fault_lidar": _floats(a["fault_lidar"]), "fault_gps": _floats(a["fault_gps"])}]),
        fusion,
        Node(package="lpa2v_px4", executable="offboard_alt"),
        logger,
    ] + end


def generate_launch_description():
    args = [("method", "pal2v"), ("scenario", "S0_nominal"), ("seed", "0"), ("duration", "60.0"),
            ("out", "/tmp/e5_run.csv"), ("gui", "false"), ("sigma", "0.15,0.02,0.05"),
            ("max_rate", "4.0"), ("auto_shutdown", "true"), ("fault_lidar", "8.0,18.0"),
            ("fault_gps", "25.0,35.0"), ("partition", "px4e5")]
    return LaunchDescription([DeclareLaunchArgument(k, default_value=v) for k, v in args]
                             + [OpaqueFunction(function=_setup)])
