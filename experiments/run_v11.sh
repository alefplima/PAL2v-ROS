#!/usr/bin/env bash
# v1.1: closed-loop runs of the graded (no-exclusion) PAL2vFusion + 6th E5 seed for every method.
# The v1.0.0 runs (method pal2v = step) are kept and reused; resumes where it stopped.
set -e
cd "$(dirname "$0")"
echo "[$(date +%T)] E4 pal2v_g"
( source ../scripts/env.sh; /usr/bin/python3 e4_gazebo.py --methods pal2v_g; /usr/bin/python3 e4_analysis.py > /dev/null )
echo "[$(date +%T)] E5 pal2v_g seeds 0-5 + seed 5 for all methods"
( source ../scripts/env.sh px4
  /usr/bin/python3 e5_px4.py --methods pal2v_g --seeds 0 1 2 3 4 5
  /usr/bin/python3 e5_px4.py --seeds 5
  /usr/bin/python3 e5_analysis.py > /dev/null
  /usr/bin/python3 e5_audit_failsafe.py > /dev/null )
echo "[$(date +%T)] done"
