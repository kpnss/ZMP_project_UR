"""Ablation study on the ZMP-lag gain g_p.

Runs the default simulation (MPC + CP feedback + Kalman filter, i.e. `no args`)
headless for a sweep of g_p values, in two regimes:
  - no lag  (g_p is unused -> a single baseline, drawn as a horizontal line)
  - with lag (g_p enters the lag dynamics and the k_1/k_2/k_i gains)

Produces per-axis summary plots (metric vs g_p), avg + max of the |error|:
  - ZMP tracking error   : x, y, z            (3 plots)
  - COM tracking error   : x, y, z            (3 plots)
  - Waist attitude error : roll, pitch        (2 plots)
  - Vertical reaction force (max + avg of Fz) (1 plot)

Runs that make the robot fall (the MPC QP diverges) are caught, flagged as
`diverged`, excluded from the metric lines, and marked on the plots.

Usage:
  python ablation_gp.py            # run the sweep, cache raw data, make plots
  python ablation_gp.py --replot   # re-make plots from the cached raw data
"""
import os
import sys
import pickle
import warnings
import contextlib
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from scipy.spatial.transform import Rotation as R
import dartpy as dart

from simulation import Hrp4Controller

warnings.filterwarnings("ignore")

G_P_VALUES = [1, 5, 10, 30, 50, 70, 100]
OUT_DIR = os.path.join("logs", "ablation_gp")
RAW_CACHE = os.path.join(OUT_DIR, "raw_runs.pkl")
CURRENT_DIR = os.path.dirname(os.path.abspath(__file__))
# discard the initial double-support settling transient from the metrics
WARMUP_STEPS = 200
AXES = ("x", "y", "z")
ATT_ANGLES = ("roll", "pitch")


@contextlib.contextmanager
def suppress_c_output():
    """Silence CasADi's C++ solve-failure dump (writes straight to fd 1/2)."""
    sys.stdout.flush()
    sys.stderr.flush()
    devnull = os.open(os.devnull, os.O_WRONLY)
    old_out, old_err = os.dup(1), os.dup(2)
    try:
        os.dup2(devnull, 1)
        os.dup2(devnull, 2)
        yield
    finally:
        os.dup2(old_out, 1)
        os.dup2(old_err, 2)
        os.close(devnull)
        os.close(old_out)
        os.close(old_err)


def build_world():
    world = dart.simulation.World()
    urdf = dart.utils.DartLoader()
    hrp4 = urdf.parseSkeleton(os.path.join(CURRENT_DIR, "urdf", "hrp4.urdf"))
    ground = urdf.parseSkeleton(os.path.join(CURRENT_DIR, "urdf", "ground.urdf"))
    world.addSkeleton(hrp4)
    world.addSkeleton(ground)
    world.setGravity([0, 0, -9.81])
    world.setTimeStep(0.01)
    default_inertia = dart.dynamics.Inertia(1e-8, np.zeros(3), 1e-10 * np.identity(3))
    for body in hrp4.getBodyNodes():
        if body.getMass() == 0.0:
            body.setMass(1e-8)
            body.setInertia(default_inertia)
    return world, hrp4


def run_once(use_lag, g_p, n_steps=None):
    """Run one headless simulation. Returns per-axis error series (post warm-up)."""
    world, hrp4 = build_world()
    node = Hrp4Controller(world, hrp4, log_path=None, autosave_every=0,
                          use_kf=True, use_mpc=True, use_cp=True,
                          open_loop=False, use_lag=use_lag, g_p=g_p)
    if n_steps is None:
        n_steps = sum(s['ss_duration'] + s['ds_duration'] for s in node.footstep_planner.plan)

    diverged_at = None
    with suppress_c_output():
        for i in range(n_steps):
            try:
                node.customPreStep()
                world.step()
            except Exception:
                # MPC QP failed / state blew up -> the robot fell
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

    plt.close('all')  # free the live-plot figure created in the logger
    return {
        'zmp_err': zmp_err, 'com_err': com_err, 'att_err': att_err, 'fz': fz,
        'diverged_at': diverged_at,
    }


def collect():
    """Run the sweep (single no-lag baseline + lag sweep) and cache raw series."""
    os.makedirs(OUT_DIR, exist_ok=True)
    # g_p is unused in no-lag mode -> a single baseline run suffices
    print("Running no-lag baseline ...", flush=True)
    baseline = run_once(use_lag=False, g_p=20.0)

    lag_runs = []
    for g_p in G_P_VALUES:
        print(f"Running lag, g_p={g_p} ...", flush=True)
        run = run_once(use_lag=True, g_p=g_p)
        if run['diverged_at'] is not None:
            print(f"  g_p={g_p}: DIVERGED at step {run['diverged_at']}", flush=True)
        lag_runs.append(run)

    data = {'gp': G_P_VALUES, 'baseline': baseline, 'lag_runs': lag_runs}
    with open(RAW_CACHE, 'wb') as f:
        pickle.dump(data, f)
    print(f"Cached raw runs -> {RAW_CACHE}", flush=True)
    return data


def metrics_of(run):
    """Per-axis avg(|err|) and max(|err|); flags runs with no usable data."""
    diverged = run['diverged_at'] is not None
    m = {'diverged': diverged, 'diverged_at': run['diverged_at']}

    def fill(prefix, err, names):
        usable = err.shape[0] > 1
        for k, name in enumerate(names):
            a = np.abs(err[:, k]) if usable else np.array([np.nan])
            m[f'{prefix}_{name}_avg'] = float(np.mean(a))
            m[f'{prefix}_{name}_max'] = float(np.max(a))

    fill('zmp', run['zmp_err'], AXES)
    fill('com', run['com_err'], AXES)
    fill('att', run['att_err'], ATT_ANGLES)
    fz = run['fz']
    if fz.shape[0] > 1:
        m['fz_avg'], m['fz_max'] = float(np.mean(fz)), float(np.max(fz))
    else:
        m['fz_avg'] = m['fz_max'] = float('nan')
    return m


def plot_metric(gp, lag_metrics, baseline, avg_key, max_key, ylabel, title, fname,
                avg_label="avg", max_label="max"):
    stable = np.array([not m['diverged'] for m in lag_metrics])
    diverged_gp = gp[~stable]

    fig, ax = plt.subplots(figsize=(8, 5))
    if stable.any():
        gs = gp[stable]
        avg = np.array([lag_metrics[i][avg_key] for i in range(len(gp)) if stable[i]])
        mx = np.array([lag_metrics[i][max_key] for i in range(len(gp)) if stable[i]])
        ax.plot(gs, avg, "o-", color="tab:blue", label=f"lag: {avg_label}")
        ax.plot(gs, mx, "s-", color="tab:red", label=f"lag: {max_label}")

    ax.axhline(baseline[avg_key], color="tab:blue", ls="--", alpha=0.7,
               label=f"no-lag: {avg_label}")
    ax.axhline(baseline[max_key], color="tab:red", ls="--", alpha=0.7,
               label=f"no-lag: {max_label}")

    for j, gd in enumerate(diverged_gp):
        ax.axvline(gd, color="grey", ls=":", alpha=0.7,
                   label="diverged (fell)" if j == 0 else None)

    ax.set_xlabel("g_p")
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
    gp = np.array(data['gp'], dtype=float)
    baseline = metrics_of(data['baseline'])
    lag_metrics = [metrics_of(r) for r in data['lag_runs']]

    # save scalar metrics table
    keys = [k for k in lag_metrics[0] if k not in ('diverged', 'diverged_at')]
    np.savez(
        os.path.join(OUT_DIR, "ablation_gp_metrics.npz"),
        g_p=gp,
        diverged=np.array([m['diverged'] for m in lag_metrics]),
        **{f"lag_{k}": np.array([m[k] for m in lag_metrics]) for k in keys},
        **{f"nolag_{k}": np.array(baseline[k]) for k in keys},
    )

    # ZMP error per axis
    for ax_name in AXES:
        plot_metric(gp, lag_metrics, baseline,
                    f'zmp_{ax_name}_avg', f'zmp_{ax_name}_max',
                    f"ZMP {ax_name} |error| [m]",
                    f"ZMP {ax_name} tracking error vs g_p",
                    f"zmp_error_{ax_name}_vs_gp.png")

    # COM error per axis
    for ax_name in AXES:
        plot_metric(gp, lag_metrics, baseline,
                    f'com_{ax_name}_avg', f'com_{ax_name}_max',
                    f"COM {ax_name} |error| [m]",
                    f"COM {ax_name} tracking error vs g_p",
                    f"com_error_{ax_name}_vs_gp.png")

    # waist attitude per angle
    for ang in ATT_ANGLES:
        plot_metric(gp, lag_metrics, baseline,
                    f'att_{ang}_avg', f'att_{ang}_max',
                    f"waist {ang} |error| [deg]",
                    f"Waist {ang} error vs g_p",
                    f"waist_{ang}_vs_gp.png")

    # vertical reaction force
    plot_metric(gp, lag_metrics, baseline, 'fz_avg', 'fz_max',
                "vertical reaction force [N]", "Vertical reaction force vs g_p",
                "vertical_force_vs_gp.png", avg_label="avg Fz", max_label="max Fz")

    print("Done.", flush=True)


def main():
    replot = "--replot" in sys.argv
    if replot:
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
