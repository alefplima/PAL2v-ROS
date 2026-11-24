# E5 - failsafe audit of the valid flights

Valid flights with a warning: 9

| flight | warning after arming [s] | 1st warning | failsafe modes | landings | disarms |
| --- | --- | --- | --- | --- | --- |
| S0_nominal_mean_s3 | 5 | Preflight Fail: Battery unhealthy | — | 0 | 0 |
| S1_travado_ivw_s2 | 47 | Preflight Fail: Attitude failure (roll) | — | 0 | 0 |
| S3_deriva_median_s3 | 43 | Preflight Fail: Attitude failure (roll) | — | 0 | 0 |
| S3_deriva_pal2v_s1 | 7 | Preflight Fail: Battery unhealthy | — | 0 | 0 |
| S4_dropout_pal2v_s0 | 8 | Preflight Fail: Battery unhealthy | — | 0 | 0 |
| S5_salto_gps_kf_chi2_s1 | 37 | Preflight Fail: Attitude failure (roll) | — | 0 | 0 |
| S5_salto_gps_paraextrctr_s0 | 6 | Preflight Fail: Battery unhealthy | — | 0 | 0 |
| S6_duplo_ivw_s3 | 11 | Preflight Fail: Attitude failure (pitch) | blind land | 0 | 0 |
| S6_duplo_paraextrctr_s2 | 57 | Preflight Fail: Attitude failure (roll) | — | 1 | 1 |

No mode change, landing or disarm = the warning did not change the flight. With an effect, check in the height series (flight CSV) whether the ground touch came before the warning.
