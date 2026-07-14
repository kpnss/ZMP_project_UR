"""Ablation study on the CP-feedback design parameters, in the DEFAULT config.

Sweeps, one at a time, the three closed-loop poles {alpha, beta, gamma} and the
ZMP-lag gain g_p, running the simulation in its default configuration (MPC + CP
feedback + ZMP feedback + Kalman filter; i.e. no --lag, no --no-zmp-fb). With
ZMP feedback on (use_zmp_fb, the default) the capture-point feedback law
  p_cmd = p_ref - k_1*(xi-xi_ref) - k_2*(p-p_ref) - k_i*int(xi_err)
places three closed-loop poles via k_1/k_2/k_i, computed from all three poles
and g_p:
  k_1 = -(a*b + b*g + g*a - eta*(a+b+g-eta)) / (eta*g_p)
  k_2 = -(a + b + g + g_p - eta) / g_p
  k_i =  (a*b*g) / (eta*g_p)
(a=alpha, b=beta, g=gamma). NOTE: the default plant has no ZMP lag, so g_p here
enters ONLY through the feedback-gain design above, not the plant dynamics.

For each swept parameter we vary its value about the default (holding the others
at their simulation.py defaults), recompute the gains, override them on the
controller, and measure tracking. Per parameter this yields the usual per-axis
summary plots (avg + max of |error|):
  - ZMP tracking error   : x, y, z            (3 plots)
  - COM tracking error   : x, y, z            (3 plots)
  - Waist attitude error : roll, pitch        (2 plots)
  - Vertical reaction force (max + avg of Fz) (1 plot)

The default value is marked with a green line; runs that fall (the MPC QP
diverges) are excluded from the metric lines and marked with grey verticals.

Usage:
  python ablation_poles.py            # default config: sweep, cache, make plots
  python ablation_poles.py --lag      # same but with ZMP lag dynamics enabled
                                      #   (writes to logs/ablation_poles_lag/)
  python ablation_poles.py --replot   # re-make plots from the cached raw data
  python ablation_poles.py --lag --replot   # replot the cached lag runs
"""
import os
import sys
import pickle
import contextlib
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from scipy.spatial.transform import Rotation as R
import dartpy as dart

from simulation import Hrp4Controller
from utils import cp_feedback_gains

# discard the initial double-support settling transient from the metrics
WARMUP_STEPS = 200
AXES = ("x", "y", "z")
ATT_ANGLES = ("roll", "pitch")
CURRENT_DIR = os.path.dirname(os.path.abspath(__file__))

# swept parameter -> (sweep values, default). Defaults mirror simulation.py; the
# three poles are negative (stable), g_p is the positive ZMP-lag gain. Each sweep
# straddles the default.
POLES = {
    'alpha': ([-25.0, -15.0, -10.0, -5.0, -3.0, -2.0, -1.0], -3.0),
    'beta':  ([-80.0, -40.0, -25.0, -16.0, -8.0, -4.0, -2.0], -8.0),
    'gamma': ([-15.0, -10.0, -5.0, -3.0, -2, -1], -2.0),
    'g_p':   ([5.0, 10.0, 20.0, 50.0, 70.0, 90.0], 20.0),
}
# plant mode + output paths; overridden in main() when --lag is passed
LAG = False
OUT_DIR = os.path.join("logs", "ablation_poles")
RAW_CACHE = os.path.join(OUT_DIR, "raw_runs.pkl")


# ---- headless-sim infrastructure -------------------------------------------

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


def metrics_of(run):
    """Per-axis avg(|err|), rmse(err) and max(|err|); flags unusable runs."""
    diverged = run['diverged_at'] is not None
    m = {'diverged': diverged, 'diverged_at': run['diverged_at']}

    def fill(prefix, err, names):
        usable = err.shape[0] > 1
        for k, name in enumerate(names):
            a = np.abs(err[:, k]) if usable else np.array([np.nan])
            m[f'{prefix}_{name}_avg'] = float(np.mean(a))
            m[f'{prefix}_{name}_rmse'] = float(np.sqrt(np.mean(a**2)))
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


# ---- sweep -----------------------------------------------------------------

def run_once(param, value, n_steps=None):
    """Run one headless sim (default config, or lag plant if LAG is set) with
    `param` set to `value` (others at the simulation.py defaults). All four
    parameters enter the CP-feedback gains, which we override after construction;
    g_p is passed at construction so node.params['g_p'] -- read by the gain
    formula, and by the lag plant when LAG -- picks it up."""
    world, hrp4 = build_world()
    # g_p feeds the gain design (and the lag plant when LAG); None -> sim default
    g_p = value if param == 'g_p' else None
    node = Hrp4Controller(world, hrp4, log_path=None, autosave_every=0,
                          use_kf=True, use_mpc=True, use_cp=True,
                          open_loop=False, use_lag=LAG, g_p=g_p)

    # poles: take the controller's (simulation.py) defaults, override swept pole
    p = dict(alpha=node.params['alpha'], beta=node.params['beta'],
             gamma=node.params['gamma'])
    if param in p:
        p[param] = value
    k_1, k_2, k_i = cp_feedback_gains(p['alpha'], p['beta'], p['gamma'],
                                      node.params['eta'], node.params['g_p'],
                                      use_zmp_fb=True)
    node.mpc.k_1, node.mpc.k_2, node.mpc.k_i = k_1, k_2, k_i

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
    data = {}
    for param, (values, default) in POLES.items():
        runs = []
        for v in values:
            print(f"Running {param}={v} ...", flush=True)
            run = run_once(param, v)
            if run['diverged_at'] is not None:
                print(f"  {param}={v}: DIVERGED at step {run['diverged_at']}", flush=True)
            runs.append(run)
        data[param] = {'values': values, 'default': default, 'runs': runs}
    with open(RAW_CACHE, 'wb') as f:
        pickle.dump(data, f)
    print(f"Cached raw runs -> {RAW_CACHE}", flush=True)
    return data


def plot_metric(pole, xs, default, metrics, avg_key, max_key, ylabel, title, fname,
                avg_label="avg", max_label="max"):
    stable = np.array([not m['diverged'] for m in metrics])
    diverged_x = xs[~stable]

    fig, ax = plt.subplots(figsize=(8, 5))
    if stable.any():
        xstab = xs[stable]
        avg = np.array([metrics[i][avg_key] for i in range(len(xs)) if stable[i]])
        mx = np.array([metrics[i][max_key] for i in range(len(xs)) if stable[i]])
        ax.plot(xstab, avg, "o-", color="tab:blue", label=avg_label)
        ax.plot(xstab, mx, "s-", color="tab:red", label=max_label)

    ax.axvline(default, color="tab:green", ls="-", alpha=0.6,
               label=f"default {pole} = {default}")
    for j, xd in enumerate(diverged_x):
        ax.axvline(xd, color="grey", ls=":", alpha=0.7,
                   label="diverged (fell)" if j == 0 else None)

    ax.set_xlabel(pole)
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
    for pole, d in data.items():
        xs = np.array(d['values'], dtype=float)
        default = d['default']
        metrics = [metrics_of(r) for r in d['runs']]

        keys = [k for k in metrics[0] if k not in ('diverged', 'diverged_at')]
        np.savez(
            os.path.join(OUT_DIR, f"ablation_{pole}_metrics.npz"),
            **{pole: xs},
            diverged=np.array([m['diverged'] for m in metrics]),
            **{f"lag_{k}": np.array([m[k] for m in metrics]) for k in keys},
        )

        for ax_name in AXES:
            plot_metric(pole, xs, default, metrics,
                        f'zmp_{ax_name}_avg', f'zmp_{ax_name}_max',
                        f"ZMP {ax_name} |error| [m]",
                        f"ZMP {ax_name} tracking error vs {pole}",
                        f"zmp_error_{ax_name}_vs_{pole}.png")
        for ax_name in AXES:
            plot_metric(pole, xs, default, metrics,
                        f'com_{ax_name}_avg', f'com_{ax_name}_max',
                        f"COM {ax_name} |error| [m]",
                        f"COM {ax_name} tracking error vs {pole}",
                        f"com_error_{ax_name}_vs_{pole}.png")
        for ang in ATT_ANGLES:
            plot_metric(pole, xs, default, metrics,
                        f'att_{ang}_avg', f'att_{ang}_max',
                        f"waist {ang} |error| [deg]",
                        f"Waist {ang} error vs {pole}",
                        f"waist_{ang}_vs_{pole}.png")
        plot_metric(pole, xs, default, metrics, 'fz_avg', 'fz_max',
                    "vertical reaction force [N]",
                    f"Vertical reaction force vs {pole}",
                    f"vertical_force_vs_{pole}.png",
                    avg_label="avg Fz", max_label="max Fz")
    print("Done.", flush=True)


def main():
    global LAG, OUT_DIR, RAW_CACHE
    LAG = "--lag" in sys.argv
    if LAG:
        OUT_DIR = os.path.join("logs", "ablation_poles_lag")
        RAW_CACHE = os.path.join(OUT_DIR, "raw_runs.pkl")
        print("Lag mode enabled (--lag).", flush=True)

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
