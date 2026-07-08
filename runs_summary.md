# Runs summary

Per-axis error over each run after a warm-up of 200 steps, reported as `RMSE = sqrt(mean(e^2))`; the vertical force column is the peak (`max Fz`). Positions in mm, waist attitude in deg, force in N. A `diverged` run fell (MPC QP blew up) and its metrics are not meaningful.

## Configuration comparison (kf on)

Fresh headless runs on the current code/params. Each variant flips one feature off the **plain** baseline (ISMPC + capture-point + ZMP feedback).

Flag columns:

- **mpc** — intrinsically-stable MPC (off = the CP feedback controller)
- **cp fb** — capture-point feedback correction on the ZMP command
- **zmp fb** — `k_2*(p-p_ref)` ZMP-position feedback term (`use_zmp_fb`)
- **lag sys** — ZMP first-order-lag plant dynamics (`use_lag`)
- **openloop ref** — track open-loop references (always on for the CP controller, which retrieves them by construction)

| config | mpc | cp fb | zmp fb | lag sys | openloop ref | ZMP x [mm] | ZMP y [mm] | ZMP z [mm] | COM x [mm] | COM y [mm] | COM z [mm] | waist roll [deg] | waist pitch [deg] | max Fz [N] | diverged |
|---|:---:|:---:|:---:|:---:|:---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|:---:|
| ISMPC (no cp) | ✓ | · | · | · | · | 1.69 | 2.76 | 0.42 | 1.02 | 1.51 | 0.17 | 0.241 | 1.376 | 758.3 | no |
| plain | ✓ | ✓ | ✓ | · | · | 1.53 | 2.48 | 1.39 | 1.03 | 1.38 | 0.02 | 0.087 | 0.665 | 768.3 | no |
| no zmp fb | ✓ | ✓ | · | · | · | 1.62 | 2.66 | 1.23 | 1.05 | 1.48 | 0.03 | 0.141 | 1.043 | 767.1 | no |
| lag | ✓ | ✓ | ✓ | ✓ | · | 8.75 | 15.27 | 1.28 | 1.09 | 1.55 | 0.02 | 0.066 | 0.553 | 653.1 | no |
| openloop | ✓ | ✓ | ✓ | · | ✓ | 1.65 | 2.76 | 1.89 | 1.12 | 1.28 | 0.89 | 0.156 | 1.095 | 780.1 | no |
| CP controller | · | ✓ | ✓ | · | ✓ | 3.37 | 5.73 | 0.65 | 4.31 | 5.73 | 1.17 | 0.152 | 0.512 | 775.9 | no |
| CP controller, no zmp fb | · | ✓ | · | · | ✓ | 3.42 | 5.88 | 0.70 | 3.93 | 5.51 | 1.01 | 0.218 | 0.725 | 801.7 | no |
| CP controller, lag | · | ✓ | ✓ | ✓ | ✓ | 12.04 | 20.73 | 0.13 | 5.29 | 7.14 | 0.83 | 0.149 | 0.540 | 754.6 | no |

## Ablation studies

Each row is one run of a parameter sweep, recomputed from the study's cached raw series. Per-axis plots live in the matching `logs/` dir.

### Poles — alpha

| alpha | ZMP x [mm] | ZMP y [mm] | ZMP z [mm] | COM x [mm] | COM y [mm] | COM z [mm] | waist roll [deg] | waist pitch [deg] | max Fz [N] | diverged |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|:---:|
| -25 | 1.47 | 2.27 | 3.49 | 1.01 | 1.27 | 0.02 | 0.139 | 0.933 | 639.7 | no |
| -15 | 1.54 | 2.36 | 2.86 | 1.01 | 1.31 | 0.03 | 0.059 | 0.742 | 776.9 | no |
| -10 | 1.51 | 2.39 | 1.95 | 1.02 | 1.34 | 0.02 | 0.113 | 0.912 | 773.0 | no |
| -5 | 1.54 | 2.48 | 1.23 | 1.03 | 1.38 | 0.02 | 0.167 | 0.845 | 776.5 | no |
| -3 | 1.55 | 2.53 | 1.21 | 1.03 | 1.40 | 0.05 | 0.091 | 0.842 | 772.4 | no |
| -2 | 1.55 | 2.55 | 0.82 | 1.04 | 1.41 | 0.03 | 0.150 | 1.156 | 764.4 | no |
| -1 | 1.57 | 2.58 | 0.75 | 1.04 | 1.43 | 0.04 | 0.134 | 0.974 | 764.0 | no |

### Poles — beta

| beta | ZMP x [mm] | ZMP y [mm] | ZMP z [mm] | COM x [mm] | COM y [mm] | COM z [mm] | waist roll [deg] | waist pitch [deg] | max Fz [N] | diverged |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|:---:|
| -80 | — | — | — | — | — | — | — | — | — | yes |
| -40 | 1.46 | 2.27 | 2.76 | 1.01 | 1.27 | 0.02 | 0.082 | 0.905 | 788.5 | no |
| -25 | 1.49 | 2.36 | 1.55 | 1.02 | 1.32 | 0.02 | 0.141 | 1.052 | 606.0 | no |
| -16 | 1.54 | 2.48 | 1.23 | 1.03 | 1.38 | 0.02 | 0.167 | 0.845 | 776.5 | no |
| -8 | 1.60 | 2.69 | 0.80 | 1.05 | 1.50 | 0.04 | 0.102 | 1.146 | 764.5 | no |
| -4 | 1.68 | 2.91 | 0.50 | 1.08 | 1.62 | 0.05 | 0.175 | 1.268 | 761.5 | no |
| -2 | 1.74 | 3.08 | 0.51 | 1.11 | 1.74 | 0.03 | 0.197 | 0.765 | 762.3 | no |

### Poles — gamma

| gamma | ZMP x [mm] | ZMP y [mm] | ZMP z [mm] | COM x [mm] | COM y [mm] | COM z [mm] | waist roll [deg] | waist pitch [deg] | max Fz [N] | diverged |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|:---:|
| -15 | 1.49 | 2.32 | 2.51 | 1.02 | 1.30 | 0.02 | 0.158 | 1.100 | 781.6 | no |
| -10 | 1.50 | 2.37 | 2.03 | 1.02 | 1.32 | 0.02 | 0.144 | 0.843 | 775.9 | no |
| -5 | 1.54 | 2.45 | 1.54 | 1.02 | 1.36 | 0.03 | 0.123 | 0.809 | 604.6 | no |
| -3 | 1.54 | 2.48 | 1.23 | 1.03 | 1.38 | 0.02 | 0.167 | 0.845 | 776.5 | no |
| -2 | 1.54 | 2.50 | 1.07 | 1.03 | 1.39 | 0.04 | 0.102 | 0.949 | 765.2 | no |
| -1 | 1.53 | 2.52 | 0.96 | 1.04 | 1.40 | 0.03 | 0.128 | 0.986 | 767.2 | no |

### ZMP-lag gain g_p

| g_p | ZMP x [mm] | ZMP y [mm] | ZMP z [mm] | COM x [mm] | COM y [mm] | COM z [mm] | waist roll [deg] | waist pitch [deg] | max Fz [N] | diverged |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|:---:|
| 5 | — | — | — | — | — | — | — | — | — | yes |
| 10 | 1.45 | 2.26 | 2.58 | 1.01 | 1.26 | 0.02 | 0.116 | 0.853 | 791.4 | no |
| 20 | 1.54 | 2.48 | 1.23 | 1.03 | 1.38 | 0.02 | 0.167 | 0.845 | 776.5 | no |
| 50 | — | — | — | — | — | — | — | — | — | yes |
| 70 | — | — | — | — | — | — | — | — | — | yes |
| 90 | — | — | — | — | — | — | — | — | — | yes |
