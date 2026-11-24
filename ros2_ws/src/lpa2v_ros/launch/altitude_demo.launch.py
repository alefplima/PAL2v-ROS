"""Demo: PAL2v fusion of 3 altimeters feeding the PAL2v-PI controller.

Usage: ros2 launch lpa2v_ros altitude_demo.launch.py [sim:=true|false] [fault:=true|false]
Expected topics (from the simulator/drone): /baro_alt, /lidar_alt, /gps_alt (std_msgs/Float64)
and /alt_setpoint. Output: /thrust_cmd. Parameters in config/altitude.yaml.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    cfg = os.path.join(get_package_share_directory("lpa2v_ros"), "config", "altitude.yaml")
    sim = LaunchConfiguration("sim")
    fault = LaunchConfiguration("fault")
    return LaunchDescription([
        DeclareLaunchArgument("sim", default_value="true",
                              description="starts the 1-DOF simulator (altitude_sim) instead of the drone"),
        DeclareLaunchArgument("fault", default_value="false",
                              description="injects the faults of scenario F of E3"),
        Node(package="lpa2v_ros", executable="altitude_sim", name="altitude_sim",
             parameters=[{"fault": ParameterValue(fault, value_type=bool)}], condition=IfCondition(sim)),
        Node(package="lpa2v_ros", executable="pal2v_fusion", name="alt_fusion",
             parameters=[cfg], remappings=[("fused", "/alt_fused")]),
        Node(package="lpa2v_ros", executable="pal2v_controller", name="alt_controller",
             parameters=[cfg], remappings=[("setpoint", "/alt_setpoint"),
                                           ("measurement", "/alt_fused"),
                                           ("command", "/thrust_cmd")]),
    ])
