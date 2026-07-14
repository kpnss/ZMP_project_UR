# Runs summary

Per-axis error over each run after a warm-up of 200 steps, reported as `RMSE = sqrt(mean(e^2))`; the vertical force column is the peak (`max Fz`). Positions in mm, waist attitude in deg, force in N. A `diverged` run fell (MPC QP blew up) and its metrics are not meaningful.

The **ZMP error** is measured against `p_ref` -- the MPC ZMP feedforward for the current instant as predicted at the previous step (for the CP controller: its planned ZMP) -- i.e. the SAME reference the capture-point ZMP-feedback term regulates, NOT the realized command `p_cmd`. Because CP feedback intentionally drives the commanded ZMP away from `p_ref` to correct capture-point errors, a larger ZMP error here does not by itself mean worse balance.

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
| ISMPC (no cp) | ✓ | · | · | · | · | 0.29 | 0.18 | 0.58 | 1.05 | 1.52 | 0.09 | 0.215 | 1.126 | 756.3 | no |
| plain | ✓ | ✓ | ✓ | · | · | 0.20 | 0.19 | 0.73 | 1.06 | 1.49 | 0.05 | 0.170 | 0.767 | 761.2 | no |
| no zmp fb | ✓ | ✓ | · | · | · | 0.22 | 0.29 | 0.87 | 1.05 | 1.49 | 0.03 | 0.111 | 0.995 | 575.7 | no |
| lag | ✓ | ✓ | ✓ | ✓ | · | 9.19 | 16.29 | 0.73 | 1.12 | 1.67 | 0.03 | 0.090 | 0.608 | 785.2 | no |
| openloop | ✓ | ✓ | ✓ | · | ✓ | 2.31 | 2.81 | 2.16 | 1.17 | 1.16 | 1.13 | 0.152 | 1.077 | 763.0 | no |
| CP controller | · | ✓ | ✓ | · | ✓ | 16.17 | 26.96 | 0.55 | 4.55 | 6.05 | 1.08 | 0.159 | 0.389 | 766.7 | no |
| CP controller, no zmp fb | · | ✓ | · | · | ✓ | 18.29 | 30.49 | 0.68 | 4.26 | 5.59 | 1.21 | 0.207 | 0.649 | 772.7 | no |
| CP controller, lag | · | ✓ | ✓ | ✓ | ✓ | 24.38 | 39.93 | 0.12 | 5.40 | 7.13 | 0.83 | 0.209 | 0.456 | 764.8 | no |

## Ablation studies

Each row is one run of a parameter sweep, recomputed from the study's cached raw series. Per-axis plots live in the matching `logs/` dir.

### Poles — alpha

| alpha | ZMP x [mm] | ZMP y [mm] | ZMP z [mm] | COM x [mm] | COM y [mm] | COM z [mm] | waist roll [deg] | waist pitch [deg] | max Fz [N] | diverged |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|:---:|
| -25 | 0.60 | 0.66 | 8.67 | 1.05 | 1.48 | 0.02 | 0.070 | 0.662 | 623.6 | no |
| -15 | 0.28 | 0.32 | 1.74 | 1.04 | 1.49 | 0.04 | 0.146 | 1.154 | 762.5 | no |
| -10 | 0.20 | 0.31 | 1.03 | 1.05 | 1.49 | 0.02 | 0.145 | 1.116 | 766.3 | no |
| -5 | 0.20 | 0.22 | 0.83 | 1.05 | 1.49 | 0.04 | 0.085 | 0.964 | 762.5 | no |
| -3 | 0.20 | 0.19 | 0.73 | 1.06 | 1.49 | 0.05 | 0.170 | 0.767 | 761.2 | no |
| -2 | 0.21 | 0.12 | 0.73 | 1.06 | 1.49 | 0.03 | 0.098 | 0.619 | 761.9 | no |
| -1 | 0.21 | 0.10 | 0.57 | 1.06 | 1.49 | 0.03 | 0.132 | 1.009 | 758.5 | no |

### Poles — beta

| beta | ZMP x [mm] | ZMP y [mm] | ZMP z [mm] | COM x [mm] | COM y [mm] | COM z [mm] | waist roll [deg] | waist pitch [deg] | max Fz [N] | diverged |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|:---:|
| -80 | — | — | — | — | — | — | — | — | — | yes |
| -40 | — | — | — | — | — | — | — | — | — | yes |
| -25 | 0.21 | 0.32 | 2.17 | 1.05 | 1.49 | 0.03 | 0.186 | 1.156 | 771.8 | no |
| -16 | 0.18 | 0.25 | 1.10 | 1.05 | 1.49 | 0.04 | 0.132 | 1.085 | 765.2 | no |
| -8 | 0.20 | 0.19 | 0.73 | 1.06 | 1.49 | 0.05 | 0.170 | 0.767 | 761.2 | no |
| -4 | 0.26 | 0.11 | 0.55 | 1.06 | 1.49 | 0.03 | 0.137 | 1.197 | 758.9 | no |
| -2 | 0.32 | 0.15 | 0.54 | 1.06 | 1.50 | 0.04 | 0.160 | 1.067 | 755.2 | no |

### Poles — gamma

| gamma | ZMP x [mm] | ZMP y [mm] | ZMP z [mm] | COM x [mm] | COM y [mm] | COM z [mm] | waist roll [deg] | waist pitch [deg] | max Fz [N] | diverged |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|:---:|
| -15 | 0.27 | 0.35 | 1.80 | 1.05 | 1.48 | 0.03 | 0.167 | 0.936 | 772.0 | no |
| -10 | 0.21 | 0.33 | 1.22 | 1.05 | 1.48 | 0.02 | 0.171 | 1.070 | 768.2 | no |
| -5 | 0.20 | 0.21 | 0.86 | 1.06 | 1.49 | 0.03 | 0.116 | 0.851 | 762.6 | no |
| -3 | 0.22 | 0.15 | 0.70 | 1.06 | 1.49 | 0.02 | 0.154 | 0.976 | 762.2 | no |
| -2 | 0.20 | 0.19 | 0.73 | 1.06 | 1.49 | 0.05 | 0.170 | 0.767 | 761.2 | no |
| -1 | 0.28 | 0.19 | 0.67 | 1.05 | 1.50 | 0.13 | 0.200 | 1.157 | 760.3 | no |

### ZMP-lag gain g_p

| g_p | ZMP x [mm] | ZMP y [mm] | ZMP z [mm] | COM x [mm] | COM y [mm] | COM z [mm] | waist roll [deg] | waist pitch [deg] | max Fz [N] | diverged |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|:---:|
| 5 | — | — | — | — | — | — | — | — | — | yes |
| 10 | 0.22 | 0.26 | 2.00 | 1.05 | 1.49 | 0.07 | 0.182 | 1.137 | 768.0 | no |
| 20 | 0.20 | 0.19 | 0.73 | 1.06 | 1.49 | 0.05 | 0.170 | 0.767 | 761.2 | no |
| 50 | 0.49 | 0.28 | 0.57 | 1.06 | 1.51 | 0.04 | 0.114 | 1.012 | 757.4 | no |
| 70 | 0.73 | 0.50 | 0.65 | 1.06 | 1.52 | 0.05 | 0.143 | 1.047 | 754.8 | no |
| 90 | — | — | — | — | — | — | — | — | — | yes |
