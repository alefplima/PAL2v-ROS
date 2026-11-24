#!/bin/bash
cd "$(dirname "$0")"
source ../scripts/env.sh px4
for s in S4_dropout S6_duplo; do for m in mean pal2v kf_chi2; do
  echo "== E5 $s $m"; /usr/bin/python3 e5_screenshots.py --method $m --scenario $s 2>&1 | grep -vE "libprotobuf|DynamicFactory|^$"; sleep 3
done; done
