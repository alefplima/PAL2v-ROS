from glob import glob

from setuptools import setup

package_name = "lpa2v_px4"

setup(
    name=package_name,
    version="1.0.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", glob("launch/*.py")),
        ("share/" + package_name + "/scripts", glob("scripts/*.sh")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Alessandro Ferreira Paz Lima",
    description="Experiment E5 of the LPA2v library: PX4 SITL (x500) with altitude by paraconsistent fusion",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "alt_sensors = lpa2v_px4.e5_nodes:alt_sensors_main",
            "offboard_alt = lpa2v_px4.e5_nodes:offboard_alt_main",
            "e5_logger = lpa2v_px4.e5_nodes:logger_main",
        ],
    },
)
