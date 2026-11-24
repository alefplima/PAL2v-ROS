#!/usr/bin/env bash
# Full E4: 6 methods × 6 scenarios × 10 seeds (360 runs of 90 s at rtf 2.5: ~1 min each, ~6 h) + analysis.
# Resumes where it stopped (skips runs whose CSV already exists). Extra arguments go to e4_gazebo.py.
set -e
cd "$(dirname "$0")"
source ../scripts/env.sh
/usr/bin/python3 e4_gazebo.py "$@"
/usr/bin/python3 e4_analysis.py
