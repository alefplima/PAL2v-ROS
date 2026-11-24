#!/bin/bash
# Redoes the E5 captures up to 3 times while there is a simulator artifact. Usage: ./run_prints_e5_retry.sh S6_duplo mean pal2v
cd "$(dirname "$0")"
source ../scripts/env.sh px4
s=$1; shift
for m in "$@"; do for k in 1 2 3; do
  echo "== E5 $s $m (attempt $k)"
  out=$(/usr/bin/python3 e5_screenshots.py --method $m --scenario $s 2>&1 | grep -vE "libprotobuf|DynamicFactory|^$"); echo "$out"
  echo "$out" | grep -q "simulator artifact: no" && break; sleep 20
done; done
