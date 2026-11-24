# Experiment data

The raw data are not in git (1.2 GB). They are on Zenodo:

- **File:** `pal2v_ros_data_v1.0.tar.xz` (85 MB; 1.19 GB unpacked)
- **DOI:** *(filled in after the deposit)*
- **SHA-256:** `72052ee7dd9c57bb4ffaa096f1a727ba7a8cd1ef56e5eb2c912ca7bef8a209af`
- **Data license:** CC BY 4.0

To use it, extract it inside `experiments/` (it creates `experiments/results/...`) and run the analyses:
```bash
echo "72052ee7dd9c57bb4ffaa096f1a727ba7a8cd1ef56e5eb2c912ca7bef8a209af  pal2v_ros_data_v1.0.tar.xz" | sha256sum -c
tar -xJf pal2v_ros_data_v1.0.tar.xz -C experiments
cd experiments && python3 e4_analysis.py && python3 e5_analysis.py && python3 e5_audit_failsafe.py
```

## Contents
| Path | What it is |
| --- | --- |
| `results/e4_runs/<scenario>/<method>_s<seed>.csv` | 360 E4 runs (20 Hz): time, sensors, fused value, ground truth, rejected sensors, contact, true pose |
| `results/e4_runs/progresso.log` | E4 batch log (parameters, samples and duration of each run) |
| `results/e5_runs/<scenario>/<method>_s<seed>.csv` | 180 valid E5 flights: time, sensors, fused value, ground truth, reference, rejected sensors |
| `results/e5_runs/logs/<flight>/` | full PX4/launch log of every attempt; `*.anterior_<t>` = invalid attempts, kept for transparency |
| `results/e5_runs/progresso.log` | E5 batch log, with the `INVALIDO` (invalid) attempts and their cause |
| `results/e5_piloto/`, `results/v1_mediana/` | pilot flights and the discarded version of the fusion (median-based v1) |
| `results/evidencias/` | Gazebo captures, CSVs and logs of the capture runs, panels of Figures 4, 5, 7 and 8 |
| `results/*.json`, `results/*.md` | aggregated analysis outputs, as produced by v1.0 (Portuguese file names and labels) |

The logs and file names in the package were written by the v1.0 code, in Portuguese. The identifiers are
explained in the glossary of [REPRODUCIBILITY.md](REPRODUCIBILITY.md#7-glossary-of-identifiers-kept-in-portuguese);
the current code reads them unchanged and writes English labels.
