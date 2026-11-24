from glob import glob

from setuptools import setup

package_name = "lpa2v_ros"

setup(
    name=package_name,
    version="1.0.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", glob("launch/*.py")),
        ("share/" + package_name + "/config", glob("config/*.yaml")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Alessandro Ferreira Paz Lima",
    description="ROS 2 nodes for the Paraconsistent Annotated Logic with two-valued annotation (PAL2v)",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "para_analyzer = lpa2v_ros.nodes:para_analyzer_main",
            "pal2v_fusion = lpa2v_ros.nodes:fusion_main",
            "pal2v_filter = lpa2v_ros.nodes:filter_main",
            "pal2v_controller = lpa2v_ros.nodes:controller_main",
            "altitude_sim = lpa2v_ros.altitude_sim:main",
        ],
    },
)
