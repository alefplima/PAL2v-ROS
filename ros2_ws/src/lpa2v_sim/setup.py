import os
from glob import glob

from setuptools import setup

package_name = "lpa2v_sim"


def tree(src, dst):
    """Recursive data_files (Gazebo models)."""
    out = []
    for root, _, files in os.walk(src):
        if files:
            out.append((os.path.join("share", package_name, dst, os.path.relpath(root, src)),
                        [os.path.join(root, f) for f in files]))
    return out


setup(
    name=package_name,
    version="1.0.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", glob("launch/*.py")),
        ("share/" + package_name + "/config", glob("config/*.yaml")),
        ("share/" + package_name + "/worlds", glob("worlds/*.sdf")),
    ] + tree("models", "models"),
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Alessandro Ferreira Paz Lima",
    description="Experiment E4 of the LPA2v library: TurtleBot3 in Gazebo with sensor fusion under faults",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "range_sensors = lpa2v_sim.e4_nodes:range_sensors_main",
            "baseline_fusion = lpa2v_sim.e4_nodes:baseline_fusion_main",
            "avoid_controller = lpa2v_sim.e4_nodes:avoid_controller_main",
            "e4_logger = lpa2v_sim.e4_nodes:logger_main",
        ],
    },
)
