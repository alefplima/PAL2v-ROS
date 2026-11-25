# E5 — PX4 SITL (x500), altitude by baro + LiDAR + GPS fusion in closed loop

Mean ± std over seeds. p = Wilcoxon paired by seed vs PAL2vFusion graded (no exclusion) (RMSE in the fault; n ≥ 5).


**S0_nominal** (window 8–18 s)

| Method | n | Fusion RMSE in fault [m] | Max. trajectory deviation [m] | Min. alt. in fault [m] | % flights w/ ground touch | p |
|---|---|---|---|---|---|---|
| **PAL2vFusion (graded)** | 6 | 0.05 ± 0.01 | 0.06 ± 0.02 | 1.73 ± 0.02 | 0 % | — |
| PAL2vFusion step (v1.0) | 6 | 0.04 ± 0.00 | 0.13 ± 0.13 | 1.68 ± 0.10 | 0 % | 1.6e-01 |
| Mean | 6 | 0.05 ± 0.00 | 0.11 ± 0.07 | 1.65 ± 0.13 | 0 % | 2.2e-01 |
| Median | 6 | 0.02 ± 0.01 | 0.19 ± 0.28 | 1.65 ± 0.20 | 0 % | 3.1e-02 |
| IVW | 6 | 0.01 ± 0.00 | 0.14 ± 0.12 | 1.65 ± 0.16 | 0 % | 3.1e-02 |
| ParaExtrctr | 6 | 0.04 ± 0.01 | 0.21 ± 0.20 | 1.63 ± 0.25 | 0 % | 4.4e-01 |
| Kalman χ² | 6 | 0.01 ± 0.00 | 0.07 ± 0.01 | 1.74 ± 0.01 | 0 % | 3.1e-02 |

PAL2vFusion (graded): false alarm outside the fault 1.8 %.

PAL2vFusion step (v1.0): false alarm outside the fault 0.0 %.

Kalman χ²: false alarm outside the fault 1.1 %.

**S1_travado** (window 8–18 s)

| Method | n | Fusion RMSE in fault [m] | Max. trajectory deviation [m] | Min. alt. in fault [m] | % flights w/ ground touch | p |
|---|---|---|---|---|---|---|
| **PAL2vFusion (graded)** | 6 | 0.09 ± 0.02 | 0.10 ± 0.03 | 1.74 ± 0.02 | 0 % | — |
| PAL2vFusion step (v1.0) | 6 | 0.11 ± 0.02 | 0.16 ± 0.07 | 1.66 ± 0.17 | 0 % | 6.2e-02 |
| Mean | 6 | 0.13 ± 0.04 | 0.27 ± 0.23 | 1.67 ± 0.13 | 0 % | 3.1e-02 |
| Median | 6 | 0.09 ± 0.03 | 0.21 ± 0.22 | 1.76 ± 0.22 | 0 % | 1.0e+00 |
| IVW | 6 | 0.70 ± 0.10 | 1.09 ± 0.21 | 1.67 ± 0.10 | 17 % | 3.1e-02 |
| ParaExtrctr | 6 | 0.12 ± 0.03 | 0.18 ± 0.12 | 1.69 ± 0.10 | 0 % | 9.4e-02 |
| Kalman χ² | 6 | 1.09 ± 0.09 | 1.76 ± 0.17 | 1.78 ± 0.11 | 0 % | 3.1e-02 |

PAL2vFusion (graded): faulty sensor(s) out of tolerance (rejected in step; attenuated in graded) in 0 % of the fault; false alarm outside the fault 0.0 %.

PAL2vFusion step (v1.0): faulty sensor(s) out of tolerance (rejected in step; attenuated in graded) in 13 % of the fault; false alarm outside the fault 0.1 %.

Kalman χ²: faulty sensor(s) out of tolerance (rejected in step; attenuated in graded) in 0 % of the fault; false alarm outside the fault 1.3 %.

**S3_deriva** (window 8–18 s)

| Method | n | Fusion RMSE in fault [m] | Max. trajectory deviation [m] | Min. alt. in fault [m] | % flights w/ ground touch | p |
|---|---|---|---|---|---|---|
| **PAL2vFusion (graded)** | 6 | 0.12 ± 0.01 | 0.79 ± 1.63 | 1.74 ± 0.02 | 0 % | — |
| PAL2vFusion step (v1.0) | 6 | 0.09 ± 0.01 | 0.21 ± 0.20 | 1.68 ± 0.15 | 0 % | 3.1e-02 |
| Mean | 6 | 0.48 ± 0.02 | 0.71 ± 0.05 | 1.26 ± 0.07 | 0 % | 3.1e-02 |
| Median | 6 | 0.11 ± 0.05 | 0.14 ± 0.06 | 1.66 ± 0.13 | 17 % | 6.9e-01 |
| IVW | 6 | 1.22 ± 0.00 | 1.86 ± 0.02 | 0.16 ± 0.02 | 83 % | 3.1e-02 |
| ParaExtrctr | 6 | 0.49 ± 0.01 | 0.75 ± 0.03 | 1.25 ± 0.03 | 0 % | 3.1e-02 |
| Kalman χ² | 6 | 1.44 ± 0.00 | 1.99 ± 0.00 | -0.00 ± 0.00 | 100 % | 3.1e-02 |

PAL2vFusion (graded): faulty sensor(s) out of tolerance (rejected in step; attenuated in graded) in 75 % of the fault; false alarm outside the fault 12.7 %.

PAL2vFusion step (v1.0): faulty sensor(s) out of tolerance (rejected in step; attenuated in graded) in 76 % of the fault; false alarm outside the fault 0.1 %.

Kalman χ²: faulty sensor(s) out of tolerance (rejected in step; attenuated in graded) in 0 % of the fault; false alarm outside the fault 1.2 %.

**S4_dropout** (window 8–18 s)

| Method | n | Fusion RMSE in fault [m] | Max. trajectory deviation [m] | Min. alt. in fault [m] | % flights w/ ground touch | p |
|---|---|---|---|---|---|---|
| **PAL2vFusion (graded)** | 6 | 0.08 ± 0.00 | 0.08 ± 0.02 | 1.74 ± 0.02 | 0 % | — |
| PAL2vFusion step (v1.0) | 6 | 0.08 ± 0.01 | 0.19 ± 0.18 | 1.62 ± 0.22 | 0 % | 8.4e-01 |
| Mean | 6 | 2.62 ± 0.01 | 3.28 ± 0.47 | -0.00 ± 0.00 | 100 % | 3.1e-02 |
| Median | 6 | 0.13 ± 0.04 | 0.15 ± 0.10 | 1.72 ± 0.04 | 0 % | 6.2e-02 |
| IVW | 6 | 6.64 ± 0.02 | 3.04 ± 0.08 | -0.00 ± 0.00 | 100 % | 3.1e-02 |
| ParaExtrctr | 6 | 3.39 ± 0.00 | 3.07 ± 0.02 | -0.00 ± 0.00 | 100 % | 3.1e-02 |
| Kalman χ² | 6 | 0.04 ± 0.01 | 0.09 ± 0.03 | 1.72 ± 0.05 | 0 % | 3.1e-02 |

PAL2vFusion (graded): faulty sensor(s) out of tolerance (rejected in step; attenuated in graded) in 100 % of the fault; false alarm outside the fault 0.1 %.

PAL2vFusion step (v1.0): faulty sensor(s) out of tolerance (rejected in step; attenuated in graded) in 100 % of the fault; false alarm outside the fault 0.1 %.

Kalman χ²: faulty sensor(s) out of tolerance (rejected in step; attenuated in graded) in 100 % of the fault; false alarm outside the fault 2.1 %.

**S5_salto_gps** (window 25–35 s)

| Method | n | Fusion RMSE in fault [m] | Max. trajectory deviation [m] | Min. alt. in fault [m] | % flights w/ ground touch | p |
|---|---|---|---|---|---|---|
| **PAL2vFusion (graded)** | 6 | 0.08 ± 0.01 | 0.84 ± 1.91 | 4.95 ± 0.02 | 0 % | — |
| PAL2vFusion step (v1.0) | 6 | 0.08 ± 0.01 | 0.07 ± 0.06 | 4.92 ± 0.03 | 0 % | 8.4e-01 |
| Mean | 6 | 1.01 ± 0.02 | 1.03 ± 0.05 | 3.97 ± 0.05 | 0 % | 3.1e-02 |
| Median | 6 | 0.11 ± 0.05 | 0.13 ± 0.11 | 4.88 ± 0.07 | 0 % | 9.4e-02 |
| IVW | 6 | 0.41 ± 0.00 | 0.46 ± 0.10 | 4.53 ± 0.10 | 0 % | 3.1e-02 |
| ParaExtrctr | 6 | 0.63 ± 0.02 | 0.63 ± 0.04 | 4.36 ± 0.04 | 0 % | 3.1e-02 |
| Kalman χ² | 6 | 0.07 ± 0.15 | 0.83 ± 1.97 | 4.78 ± 0.43 | 0 % | 4.4e-01 |

PAL2vFusion (graded): faulty sensor(s) out of tolerance (rejected in step; attenuated in graded) in 100 % of the fault; false alarm outside the fault 7.4 %.

PAL2vFusion step (v1.0): faulty sensor(s) out of tolerance (rejected in step; attenuated in graded) in 100 % of the fault; false alarm outside the fault 0.1 %.

Kalman χ²: faulty sensor(s) out of tolerance (rejected in step; attenuated in graded) in 100 % of the fault; false alarm outside the fault 9.4 %.

**S6_duplo** (window 8–18 s)

| Method | n | Fusion RMSE in fault [m] | Max. trajectory deviation [m] | Min. alt. in fault [m] | % flights w/ ground touch | p |
|---|---|---|---|---|---|---|
| **PAL2vFusion (graded)** | 6 | 1.45 ± 1.42 | 1.90 ± 1.30 | 1.49 ± 0.64 | 0 % | — |
| PAL2vFusion step (v1.0) | 6 | 1.79 ± 1.16 | 1.83 ± 0.87 | 1.58 ± 0.21 | 0 % | 5.6e-01 |
| Mean | 6 | 3.59 ± 0.02 | 3.08 ± 0.01 | -0.00 ± 0.00 | 100 % | 6.2e-02 |
| Median | 6 | 3.01 ± 0.03 | 3.07 ± 0.01 | -0.00 ± 0.00 | 100 % | 6.2e-02 |
| IVW | 6 | 6.98 ± 0.14 | 3.21 ± 0.43 | 0.00 ± 0.01 | 100 % | 3.1e-02 |
| ParaExtrctr | 6 | 4.02 ± 0.01 | 3.04 ± 0.05 | -0.00 ± 0.00 | 100 % | 6.2e-02 |
| Kalman χ² | 6 | 1.74 ± 1.25 | 1.90 ± 1.39 | 0.70 ± 0.84 | 50 % | 5.6e-01 |

PAL2vFusion (graded): faulty sensor(s) out of tolerance (rejected in step; attenuated in graded) in 0 % of the fault; false alarm outside the fault 14.4 %.

PAL2vFusion step (v1.0): faulty sensor(s) out of tolerance (rejected in step; attenuated in graded) in 0 % of the fault; false alarm outside the fault 0.1 %.

Kalman χ²: faulty sensor(s) out of tolerance (rejected in step; attenuated in graded) in 52 % of the fault; false alarm outside the fault 1.0 %.

## Open loop (same sensor data, PAL2v flights): RMSE in the fault window [m]

| Scenario | PAL2vFusion (graded) | PAL2vFusion step (v1.0) | Mean | Median | IVW | ParaExtrctr | Kalman χ² |
|---|---|---|---|---|---|---|---|
| S0_nominal | 0.04 ± 0.00 | 0.04 ± 0.00 | 0.05 ± 0.00 | 0.02 ± 0.01 | 0.01 ± 0.00 | 0.04 ± 0.00 | 0.01 ± 0.00 |
| S1_travado | 0.10 ± 0.03 | 0.11 ± 0.02 | 0.12 ± 0.03 | 0.09 ± 0.03 | 0.30 ± 0.11 | 0.11 ± 0.03 | 0.35 ± 0.13 |
| S3_deriva | 0.11 ± 0.01 | 0.10 ± 0.01 | 0.49 ± 0.02 | 0.11 ± 0.02 | 1.22 ± 0.01 | 0.48 ± 0.02 | 1.44 ± 0.00 |
| S4_dropout | 0.08 ± 0.01 | 0.08 ± 0.01 | 2.01 ± 0.02 | 0.09 ± 0.01 | 5.15 ± 0.03 | 1.87 ± 0.03 | 0.04 ± 0.01 |
| S5_salto_gps | 0.07 ± 0.01 | 0.07 ± 0.01 | 0.99 ± 0.02 | 0.08 ± 0.03 | 0.41 ± 0.00 | 0.56 ± 0.02 | 0.00 ± 0.00 |
| S6_duplo | 1.74 ± 1.13 | 1.68 ± 1.22 | 2.81 ± 0.23 | 3.00 ± 0.02 | 5.02 ± 0.56 | 2.75 ± 0.32 | 0.10 ± 0.02 |
