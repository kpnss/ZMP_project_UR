"""Ablation study on the ZMP-tracking feedback gain k_2, in NO-LAG mode.

In no-lag mode simulation.py sets k_2 = 0 by default; k_2 multiplies the ZMP
tracking-error term of the capture-point feedback law
(ismpc.py: p_cmd = p_ref - k_1*(xi-xi_ref) - k_2*(p_meas-p_ref) - k_i*int).
This sweep overrides the controller's k_2 (leaving k_1/k_i at their no-lag
defaults) to show what adding ZMP feedback does.

Produces per-axis summary plots (metric vs k_2), avg + max of the |error|:
  - ZMP tracking error   : x, y, z            (3 plots)
  - COM tracking error   : x, y, z            (3 plots)
  - Waist attitude error : roll, pitch        (2 plots)
  - Vertical reaction force (max + avg of Fz) (1 plot)

The default k_2 = 0 is marked with a green line; diverged (fell) runs are
excluded from the metric lines and marked with grey verticals.

Usage:
  python ablation_k2.py            # run the sweep, cache raw data, make plots
  python ablation_k2.py --replot   # re-make plots from the cached raw data
"""
import os
import sys
import pickle
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

# reuse the g_p study's infrastructure
import ablation_gp as base
from ablation_gp import (build_world, suppress_c_output, metrics_of,
                         WARMUP_STEPS, AXES, ATT_ANGLES)
from simulation import Hrp4Controller
from scipy.spatial.transform import Rotation as R

K2_VALUES = [-1.0, -0.5, 0.0, 0.5, 1.0, 2.0, 3.0]
K2_DEFAULT = 0.0  # no-lag default in simulation.py
OUT_DIR = os.path.join("logs", "ablation_k2")
RAW_CACHE = os.path.join(OUT_DIR, "raw_runs.pkl")


def run_once_k2(k2, n_steps=None):
    """Run one headless no-lag simulation with the controller's k_2 overridden."""
    world, hrp4 = build_world()
    node = Hrp4Controller(world, hrp4, log_path=None, autosave_every=0,
                          use_kf=True, use_mpc=True, use_cp=True,
                          open_loop=False, use_lag=False, g_p=20.0)
    node.mpc.k_2 = k2  # override the ZMP-tracking feedback gain
    if n_steps is None:
        n_steps = sum(s['ss_duration'] + s['ds_duration'] for s in node.footstep_planner.plan)

    diverged_at = None
    with suppress_c_output():
        for i in range(n_steps):
            try:
                node.customPreStep()
                world.step()
            except Exception:
                diverged_at = i
                break

    log = node.logger.log

    def arr(batch, item, level):
        return np.array(log[batch, item, level])

    w = WARMUP_STEPS
    zmp_err = (arr('desired', 'zmp', 'pos') - arr('current', 'zmp', 'pos'))[w:]
    com_err = (arr('desired', 'com', 'pos') - arr('current', 'com', 'pos'))[w:]
    d_att = R.from_rotvec(arr('desired', 'base', 'pos')).as_euler("xyz", degrees=True)[:, :2]
    c_att = R.from_rotvec(arr('current', 'base', 'pos')).as_euler("xyz", degrees=True)[:, :2]
    att_err = (d_att - c_att)[w:]
    fz = arr('current', 'grf', 'force')[w:, 2]

    plt.close('all')
    return {
        'zmp_err': zmp_err, 'com_err': com_err, 'att_err': att_err, 'fz': fz,
        'diverged_at': diverged_at,
    }


def collect():
    os.makedirs(OUT_DIR, exist_ok=True)
    runs = []
    for k2 in K2_VALUES:
        print(f"Running no-lag, k_2={k2} ...", flush=True)
        run = run_once_k2(k2)
        if run['diverged_at'] is not None:
            print(f"  k_2={k2}: DIVERGED at step {run['diverged_at']}", flush=True)
        runs.append(run)
    data = {'k2': K2_VALUES, 'runs': runs}
    with open(RAW_CACHE, 'wb') as f:
        pickle.dump(data, f)
    print(f"Cached raw runs -> {RAW_CACHE}", flush=True)
    return data


def plot_metric(k2, metrics, avg_key, max_key, ylabel, title, fname,
                avg_label="avg", max_label="max"):
    stable = np.array([not m['diverged'] for m in metrics])
    diverged_k2 = k2[~stable]

    fig, ax = plt.subplots(figsize=(8, 5))
    if stable.any():
        ks = k2[stable]
        avg = np.array([metrics[i][avg_key] for i in range(len(k2)) if stable[i]])
        mx = np.array([metrics[i][max_key] for i in range(len(k2)) if stable[i]])
        ax.plot(ks, avg, "o-", color="tab:blue", label=f"no-lag: {avg_label}")
        ax.plot(ks, mx, "s-", color="tab:red", label=f"no-lag: {max_label}")

    ax.axvline(K2_DEFAULT, color="tab:green", ls="-", alpha=0.6, label="default k_2 = 0")
    for j, kd in enumerate(diverged_k2):
        ax.axvline(kd, color="grey", ls=":", alpha=0.7,
                   label="diverged (fell)" if j == 0 else None)

    ax.set_xlabel("k_2")
    ax.set_ylabel(ylabel)
    ax.set_title(title)
    ax.grid(True, alpha=0.3)
    ax.legend(fontsize=9)
    fig.tight_layout()
    out = os.path.join(OUT_DIR, fname)
    fig.savefig(out, dpi=200)
    plt.close(fig)
    print(f"Saved: {out}")


def make_plots(data):
    k2 = np.array(data['k2'], dtype=float)
    metrics = [metrics_of(r) for r in data['runs']]

    keys = [k for k in metrics[0] if k not in ('diverged', 'diverged_at')]
    np.savez(
        os.path.join(OUT_DIR, "ablation_k2_metrics.npz"),
        k_2=k2,
        diverged=np.array([m['diverged'] for m in metrics]),
        **{f"nolag_{k}": np.array([m[k] for m in metrics]) for k in keys},
    )

    for ax_name in AXES:
        plot_metric(k2, metrics, f'zmp_{ax_name}_avg', f'zmp_{ax_name}_max',
                    f"ZMP {ax_name} |error| [m]",
                    f"ZMP {ax_name} tracking error vs k_2 (no-lag)",
                    f"zmp_error_{ax_name}_vs_k2.png")
    for ax_name in AXES:
        plot_metric(k2, metrics, f'com_{ax_name}_avg', f'com_{ax_name}_max',
                    f"COM {ax_name} |error| [m]",
                    f"COM {ax_name} tracking error vs k_2 (no-lag)",
                    f"com_error_{ax_name}_vs_k2.png")
    for ang in ATT_ANGLES:
        plot_metric(k2, metrics, f'att_{ang}_avg', f'att_{ang}_max',
                    f"waist {ang} |error| [deg]",
                    f"Waist {ang} error vs k_2 (no-lag)",
                    f"waist_{ang}_vs_k2.png")
    plot_metric(k2, metrics, 'fz_avg', 'fz_max',
                "vertical reaction force [N]",
                "Vertical reaction force vs k_2 (no-lag)",
                "vertical_force_vs_k2.png", avg_label="avg Fz", max_label="max Fz")
    print("Done.", flush=True)


def main():
    if "--replot" in sys.argv:
        if not os.path.exists(RAW_CACHE):
            sys.exit(f"No cache at {RAW_CACHE}; run without --replot first.")
        with open(RAW_CACHE, 'rb') as f:
            data = pickle.load(f)
        print(f"Loaded cached raw runs from {RAW_CACHE}", flush=True)
    else:
        data = collect()
    make_plots(data)


if __name__ == "__main__":
    main()
