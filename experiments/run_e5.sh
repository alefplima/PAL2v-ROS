#!/usr/bin/env bash
# Full E5: 6 methods × 6 scenarios × 5 seeds (180 flights of 60 s: ~80 s each, ~5 h with the redone ones) + analysis and audit.
# One flight at a time. A flight with a simulated-sensor timeout in the PX4 log is INVALID and redone (up to 3 attempts).
# Recommended: a cool machine (thermal throttling increases the invalid flights; see REPRODUCIBILITY.md).
set -e
cd "$(dirname "$0")"
source ../scripts/env.sh px4
/usr/bin/python3 e5_px4.py "$@"
/usr/bin/python3 e5_analysis.py
/usr/bin/python3 e5_audit_failsafe.py
