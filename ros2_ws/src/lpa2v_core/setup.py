"""lpa2v: PAL2v core in pure Python.

Serves both uses:
  pip install ./lpa2v_core          (outside ROS)
  colcon build                      (ament_python package "lpa2v" of the ros2_ws workspace)
"""
from setuptools import setup

setup(
    name="lpa2v",
    version="1.1.0",
    description="Paraconsistent Annotated Logic with two-valued annotation (PAL2v) for robotics and control",
    author="Alessandro Ferreira Paz Lima",
    maintainer="Alessandro Ferreira Paz Lima",
    license="Apache-2.0",
    python_requires=">=3.8",
    packages=["lpa2v"],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/lpa2v"]),
        ("share/lpa2v", ["package.xml"]),
    ],
    install_requires=[],
    zip_safe=True,
    tests_require=["pytest"],
)
