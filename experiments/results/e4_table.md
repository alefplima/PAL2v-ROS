# E4 — TurtleBot3 in Gazebo, reactive avoidance with sonar + LiDAR + IR fusion (LiDAR fault at 30–75 s)

Closed loop: the fusion method decides when to brake/turn. Mean ± std over seeds; **% runs w/ collision**; p = Wilcoxon paired by seed vs PAL2vFusion (time in contact).


**S0_nominal**

| Method | n | % runs w/ collision | Collisions | Time in contact [s] | Min. true dist. [m] | Traveled [m] | RMSE in fault [m] | p |
|---|---|---|---|---|---|---|---|---|
| **PAL2vFusion** | 10 | 0 % | 0.00 ± 0.00 | 0.00 ± 0.00 | 0.35 ± 0.04 | 12.89 ± 0.29 | 0.19 ± 0.02 | — |
| Mean | 10 | 0 % | 0.00 ± 0.00 | 0.00 ± 0.00 | 0.35 ± 0.05 | 13.10 ± 0.26 | 0.15 ± 0.03 | 1.0e+00 |
| Median | 10 | 0 % | 0.00 ± 0.00 | 0.00 ± 0.00 | 0.36 ± 0.03 | 13.11 ± 0.26 | 0.17 ± 0.02 | 1.0e+00 |
| IVW | 10 | 0 % | 0.00 ± 0.00 | 0.00 ± 0.00 | 0.38 ± 0.01 | 13.21 ± 0.15 | 0.23 ± 0.02 | 1.0e+00 |
| ParaExtrctr | 10 | 0 % | 0.00 ± 0.00 | 0.00 ± 0.00 | 0.38 ± 0.03 | 13.36 ± 0.64 | 0.16 ± 0.02 | 1.0e+00 |
| Kalman χ² | 10 | 0 % | 0.00 ± 0.00 | 0.00 ± 0.00 | 0.36 ± 0.03 | 12.41 ± 0.23 | 0.24 ± 0.03 | 1.0e+00 |

PAL2vFusion: LiDAR rejected in 2 % of the fault samples; false alarm outside the fault 4.1 %.

Kalman χ²: LiDAR rejected in 3 % of the fault samples; false alarm outside the fault 5.9 %.

**S1_travado**

| Method | n | % runs w/ collision | Collisions | Time in contact [s] | Min. true dist. [m] | Traveled [m] | RMSE in fault [m] | p |
|---|---|---|---|---|---|---|---|---|
| **PAL2vFusion** | 10 | 0 % | 0.00 ± 0.00 | 0.00 ± 0.00 | 0.36 ± 0.05 | 13.46 ± 0.94 | 0.16 ± 0.04 | — |
| Mean | 10 | 100 % | 1.90 ± 0.32 | 37.70 ± 0.20 | 0.00 ± 0.00 | 8.64 ± 0.17 | 0.45 ± 0.03 | 2.0e-03 |
| Median | 10 | 0 % | 0.00 ± 0.00 | 0.00 ± 0.00 | 0.35 ± 0.05 | 13.94 ± 0.86 | 0.14 ± 0.04 | 1.0e+00 |
| IVW | 10 | 100 % | 2.00 ± 0.00 | 37.95 ± 0.87 | 0.00 ± 0.00 | 8.96 ± 0.31 | 1.17 ± 0.12 | 2.0e-03 |
| ParaExtrctr | 10 | 100 % | 2.00 ± 0.00 | 38.45 ± 0.18 | 0.00 ± 0.00 | 8.81 ± 0.27 | 0.64 ± 0.02 | 2.0e-03 |
| Kalman χ² | 10 | 100 % | 2.00 ± 0.00 | 38.36 ± 0.22 | 0.00 ± 0.00 | 8.73 ± 0.22 | 1.38 ± 0.03 | 2.0e-03 |

PAL2vFusion: LiDAR rejected in 73 % of the fault samples; false alarm outside the fault 3.8 %.

Kalman χ²: LiDAR rejected in 0 % of the fault samples; false alarm outside the fault 5.8 %.

**S2_picos**

| Method | n | % runs w/ collision | Collisions | Time in contact [s] | Min. true dist. [m] | Traveled [m] | RMSE in fault [m] | p |
|---|---|---|---|---|---|---|---|---|
| **PAL2vFusion** | 10 | 0 % | 0.00 ± 0.00 | 0.00 ± 0.00 | 0.35 ± 0.04 | 12.79 ± 0.32 | 0.19 ± 0.02 | — |
| Mean | 10 | 0 % | 0.00 ± 0.00 | 0.00 ± 0.00 | 0.38 ± 0.05 | 13.51 ± 0.56 | 0.20 ± 0.03 | 1.0e+00 |
| Median | 10 | 0 % | 0.00 ± 0.00 | 0.00 ± 0.00 | 0.36 ± 0.04 | 13.21 ± 0.24 | 0.15 ± 0.04 | 1.0e+00 |
| IVW | 10 | 0 % | 0.00 ± 0.00 | 0.00 ± 0.00 | 0.38 ± 0.03 | 14.65 ± 0.72 | 0.34 ± 0.04 | 1.0e+00 |
| ParaExtrctr | 10 | 0 % | 0.00 ± 0.00 | 0.00 ± 0.00 | 0.35 ± 0.05 | 13.41 ± 0.74 | 0.18 ± 0.03 | 1.0e+00 |
| Kalman χ² | 10 | 0 % | 0.00 ± 0.00 | 0.00 ± 0.00 | 0.60 ± 0.81 | 11.27 ± 3.97 | 0.21 ± 0.08 | 1.0e+00 |

PAL2vFusion: LiDAR rejected in 19 % of the fault samples; false alarm outside the fault 3.9 %.

Kalman χ²: LiDAR rejected in 16 % of the fault samples; false alarm outside the fault 5.1 %.

**S3_deriva**

| Method | n | % runs w/ collision | Collisions | Time in contact [s] | Min. true dist. [m] | Traveled [m] | RMSE in fault [m] | p |
|---|---|---|---|---|---|---|---|---|
| **PAL2vFusion** | 10 | 0 % | 0.00 ± 0.00 | 0.00 ± 0.00 | 0.38 ± 0.05 | 13.60 ± 0.71 | 0.17 ± 0.05 | — |
| Mean | 10 | 100 % | 2.00 ± 0.00 | 38.42 ± 0.37 | 0.00 ± 0.00 | 8.88 ± 0.18 | 1.05 ± 0.00 | 2.0e-03 |
| Median | 10 | 0 % | 0.00 ± 0.00 | 0.00 ± 0.00 | 0.34 ± 0.05 | 13.97 ± 0.96 | 0.13 ± 0.04 | 1.0e+00 |
| IVW | 10 | 100 % | 1.90 ± 0.32 | 38.46 ± 0.23 | 0.00 ± 0.00 | 8.95 ± 0.23 | 2.55 ± 0.00 | 2.0e-03 |
| ParaExtrctr | 10 | 100 % | 2.00 ± 0.00 | 38.67 ± 0.29 | 0.00 ± 0.00 | 8.96 ± 0.22 | 1.13 ± 0.00 | 2.0e-03 |
| Kalman χ² | 10 | 100 % | 2.00 ± 0.00 | 38.63 ± 0.35 | 0.00 ± 0.00 | 8.83 ± 0.32 | 3.11 ± 0.00 | 2.0e-03 |

PAL2vFusion: LiDAR rejected in 96 % of the fault samples; false alarm outside the fault 4.1 %.

Kalman χ²: LiDAR rejected in 0 % of the fault samples; false alarm outside the fault 6.0 %.

**S4_dropout**

| Method | n | % runs w/ collision | Collisions | Time in contact [s] | Min. true dist. [m] | Traveled [m] | RMSE in fault [m] | p |
|---|---|---|---|---|---|---|---|---|
| **PAL2vFusion** | 10 | 0 % | 0.00 ± 0.00 | 0.00 ± 0.00 | 0.35 ± 0.08 | 13.44 ± 0.67 | 0.19 ± 0.03 | — |
| Mean | 10 | 100 % | 2.00 ± 0.00 | 38.80 ± 0.32 | 0.00 ± 0.00 | 8.87 ± 0.17 | 1.15 ± 0.00 | 2.0e-03 |
| Median | 10 | 0 % | 0.00 ± 0.00 | 0.00 ± 0.00 | 0.35 ± 0.04 | 13.84 ± 0.92 | 0.14 ± 0.03 | 1.0e+00 |
| IVW | 10 | 100 % | 1.90 ± 0.32 | 38.22 ± 0.38 | 0.00 ± 0.00 | 8.85 ± 0.20 | 2.77 ± 0.01 | 2.0e-03 |
| ParaExtrctr | 10 | 100 % | 1.90 ± 0.32 | 38.58 ± 0.31 | 0.00 ± 0.00 | 8.98 ± 0.20 | 1.16 ± 0.01 | 2.0e-03 |
| Kalman χ² | 10 | 40 % | 1.00 ± 1.49 | 2.00 ± 3.12 | 0.20 ± 0.18 | 12.95 ± 0.78 | 0.96 ± 0.98 | 6.8e-02 |

PAL2vFusion: LiDAR rejected in 99 % of the fault samples; false alarm outside the fault 4.5 %.

Kalman χ²: LiDAR rejected in 75 % of the fault samples; false alarm outside the fault 5.5 %.

**S6_duplo**

| Method | n | % runs w/ collision | Collisions | Time in contact [s] | Min. true dist. [m] | Traveled [m] | RMSE in fault [m] | p |
|---|---|---|---|---|---|---|---|---|
| **PAL2vFusion** | 10 | 100 % | 1.90 ± 0.32 | 38.76 ± 0.43 | 0.00 ± 0.00 | 8.87 ± 0.26 | 1.14 ± 0.10 | — |
| Mean | 10 | 100 % | 2.00 ± 0.00 | 38.74 ± 0.31 | 0.00 ± 0.00 | 8.88 ± 0.22 | 1.56 ± 0.04 | 1.0e+00 |
| Median | 10 | 100 % | 2.00 ± 0.00 | 38.48 ± 0.78 | 0.00 ± 0.00 | 8.87 ± 0.28 | 1.32 ± 0.18 | 3.1e-01 |
| IVW | 10 | 100 % | 2.00 ± 0.00 | 38.09 ± 0.53 | 0.00 ± 0.00 | 8.93 ± 0.32 | 2.83 ± 0.01 | 2.8e-02 |
| ParaExtrctr | 10 | 100 % | 2.00 ± 0.00 | 38.61 ± 0.55 | 0.00 ± 0.00 | 8.95 ± 0.17 | 1.50 ± 0.06 | 5.1e-01 |
| Kalman χ² | 10 | 50 % | 1.00 ± 1.05 | 9.86 ± 16.20 | 0.15 ± 0.15 | 12.71 ± 2.41 | 0.74 ± 0.57 | 9.8e-03 |

PAL2vFusion: LiDAR rejected in 100 % of the fault samples; false alarm outside the fault 4.9 %.

Kalman χ²: LiDAR rejected in 94 % of the fault samples; false alarm outside the fault 5.7 %.

## Open loop (same sensor data, PAL2v runs): RMSE in the fault window [m]

| Scenario | PAL2vFusion | Mean | Median | IVW | ParaExtrctr | Kalman χ² |
|---|---|---|---|---|---|---|
| S0_nominal | 0.15 ± 0.01 | 0.12 ± 0.01 | 0.12 ± 0.01 | 0.16 ± 0.05 | 0.12 ± 0.02 | 0.22 ± 0.04 |
| S1_travado | 0.12 ± 0.03 | 0.24 ± 0.02 | 0.11 ± 0.02 | 0.56 ± 0.06 | 0.26 ± 0.03 | 0.68 ± 0.07 |
| S2_picos | 0.15 ± 0.01 | 0.17 ± 0.01 | 0.11 ± 0.01 | 0.33 ± 0.02 | 0.15 ± 0.01 | 0.22 ± 0.03 |
| S3_deriva | 0.14 ± 0.05 | 0.79 ± 0.05 | 0.12 ± 0.03 | 1.94 ± 0.12 | 0.56 ± 0.06 | 1.22 ± 0.90 |
| S4_dropout | 0.15 ± 0.03 | 0.87 ± 0.05 | 0.12 ± 0.02 | 2.13 ± 0.13 | 0.61 ± 0.06 | 0.77 ± 0.92 |
| S6_duplo | 1.14 ± 0.10 | 1.56 ± 0.03 | 1.29 ± 0.11 | 2.84 ± 0.01 | 1.49 ± 0.04 | 0.55 ± 0.66 |
