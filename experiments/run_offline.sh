#!/usr/bin/env bash
# E1 (cost), E2 (offline fusion, 30 seeds) and E3 (1-DOF altitude, 30 seeds): no ROS, minutes.
set -e
cd "$(dirname "$0")"
python3 e1_benchmark.py && python3 e2_fusion.py && python3 e3_altitude.py
