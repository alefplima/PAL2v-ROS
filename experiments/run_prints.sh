#!/bin/bash
# Close-up captures (identification) of the robot (E4, Gazebo Jetty) and of the drone (E5, PX4 + Gazebo Harmonic), in sequence.
# Run only with the batches stopped (the windows open with a GUI and heat the CPU).
cd "$(dirname "$0")"
source ../scripts/env.sh
F="libprotobuf|DynamicFactory|^$"
for s in S4_dropout S6_duplo; do
  for m in mean pal2v kf_chi2; do
    echo "== E4 $s $m"; /usr/bin/python3 e4_screenshots.py --method $m --scenario $s 2>&1 | grep -vE "$F"; sleep 3
  done
done
source ../scripts/env.sh px4
for s in S4_dropout S6_duplo; do
  for m in mean pal2v kf_chi2; do
    echo "== E5 $s $m"; /usr/bin/python3 e5_screenshots.py --method $m --scenario $s 2>&1 | grep -vE "$F"; sleep 3
  done
done
echo "captures done"
