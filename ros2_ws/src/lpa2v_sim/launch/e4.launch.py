"""E4: TurtleBot3 (Gazebo) with reactive avoidance and fusion of sonar + LiDAR + IR under LiDAR faults.

  ros2 launch lpa2v_sim e4.launch.py method:=pal2v scenario:=S4_dropout seed:=0 gui:=true

method: pal2v (step, v1.0) | pal2v_g (graded, no exclusion) | mean | median | ivw | paraextrctr | kf_chi2     scenario: S0_nominal ... S4_dropout, S6_duplo
gui:=false runs headless (batch); rtf limits the real-time factor (gui:=true forces 1.0).
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, EmitEvent, ExecuteProcess, OpaqueFunction,
                            RegisterEventHandler, SetEnvironmentVariable)
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration as LC
from launch_ros.actions import Node

from lpa2v import PAL2vFusion

SIGMA = [0.05, 0.02, 0.08]          # sonar, LiDAR, IR [m]
RMAX, MAX_RATE, FS = 3.5, 1.0, 20.0  # UD [0, 3.5] m; max. rate 1 m/s (includes turning in place); 20 Hz
STEP = 0.002                         # physics step of the world (s)
KF_Q = 100.0                         # kf_chi2: tuned on S0 only (lowest RMSE, 5 % tie -> lowest false alarm)


def _setup(context):
    a = {k: LC(k).perform(context) for k in
         ("method", "scenario", "seed", "duration", "fault_start", "fault_end", "out", "gui", "rtf",
          "auto_shutdown")}
    share = get_package_share_directory("lpa2v_sim")
    gui = a["gui"].lower() in ("1", "true", "yes")
    rtf = 1.0 if gui else float(a["rtf"])
    world = os.path.join(share, "worlds", "e4_arena.sdf")
    gz = ["gz", "sim", "-r", "-z", str(int(round(rtf / STEP))), world]
    if not gui:
        gz[2:2] = ["-s", "--headless-rendering"]
    st = {"use_sim_time": True}
    topics = ["/e4/sonar", "/e4/lidar", "/e4/ir"]
    if a["method"] in ("pal2v", "pal2v_g"):
        mode = "graded" if a["method"] == "pal2v_g" else "step"
        f = PAL2vFusion.from_sigma(SIGMA, 0.0, RMAX, max_rate=MAX_RATE, dt=1.0 / FS)
        fusion = Node(package="lpa2v_ros", executable="pal2v_fusion", name="e4_fusion",
                      parameters=[st, {"input_topics": topics, "input_type": "float64",
                                       "vmin": 0.0, "vmax": RMAX, "ct_lim": f.ct_lim,
                                       "rate_lim": f.rate_lim, "rate_hz": FS,
                                   "mode": mode}],
                      remappings=[("fused", "/e4/fused"), ("fusion_state", "/e4/fusion_state")])
    else:
        fusion = Node(package="lpa2v_sim", executable="baseline_fusion", name="e4_fusion",
                      parameters=[st, {"method": a["method"], "input_topics": topics,
                                       "sigma": SIGMA, "rate_hz": FS, "kf_q": KF_Q}],
                      remappings=[("fused", "/e4/fused"), ("fusion_state", "/e4/fusion_state")])
    logger = Node(package="lpa2v_sim", executable="e4_logger",
                  parameters=[st, {"out": a["out"], "duration": float(a["duration"]), "rate_hz": FS}])
    # The ros_gz parameter_bridge may segfault on SIGINT (apport warning on Ubuntu). The batch runner
    # uses auto_shutdown:=false and ends the process group with SIGKILL when the CSV appears.
    end = ([RegisterEventHandler(OnProcessExit(target_action=logger, on_exit=[EmitEvent(event=Shutdown())]))]
           if a["auto_shutdown"].lower() in ("1", "true", "yes") else [])
    return [
        SetEnvironmentVariable("GZ_SIM_RESOURCE_PATH",
                               os.path.join(share, "models") + os.pathsep + os.environ.get("GZ_SIM_RESOURCE_PATH", "")),
        ExecuteProcess(cmd=gz, output="screen" if gui else "log"),
        Node(package="ros_gz_bridge", executable="parameter_bridge", output="log",
             parameters=[{"config_file": os.path.join(share, "config", "e4_bridge.yaml")}]),
        Node(package="lpa2v_sim", executable="range_sensors",
             parameters=[st, {"seed": int(a["seed"]), "scenario": a["scenario"],
                              "fault_start": float(a["fault_start"]), "fault_end": float(a["fault_end"]),
                              "sigma": SIGMA, "fast_rate_hz": FS}]),
        fusion,
        Node(package="lpa2v_sim", executable="avoid_controller", parameters=[st, {"rate_hz": FS}]),
        logger,
    ] + end


def generate_launch_description():
    args = [("method", "pal2v"), ("scenario", "S0_nominal"), ("seed", "0"), ("duration", "120.0"),
            ("fault_start", "40.0"), ("fault_end", "100.0"), ("out", "/tmp/e4_run.csv"),
            ("gui", "false"), ("rtf", "2.5"), ("auto_shutdown", "true")]
    return LaunchDescription([DeclareLaunchArgument(k, default_value=v) for k, v in args]
                             + [OpaqueFunction(function=_setup)])
