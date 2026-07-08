"""Generate runs_summary.md: a table of every run we did (config comparison +
ablations).

- Config comparison: reruns headless configs (kf on) that isolate the
  mpc/no-mpc, cp/no-cp, lag/no-lag and open-loop axes.
- Ablations: recomputes metrics from each study's cached `raw_runs.pkl`
  (raw error series -> no re-simulation needed).

Per axis, over the N post-warm-up steps of the error series e[t], errors are
reported as RMSE = sqrt(mean(e^2)); the vertical reaction force is reported as
its peak (max Fz). Positions in mm, waist attitude in deg, force in N.
`diverged` = the run fell (MPC QP blew up); its metrics are not meaningful.

Usage:  python make_runs_summary.py
"""
import pickle
import numpy as np

from ablation_poles import (build_world, suppress_c_output, metrics_of,
                            WARMUP_STEPS)
from simulation import Hrp4Controller

OUT_MD = "runs_summary.md"


# ---- config comparison (fresh headless runs) -------------------------------

# Each config is defined by the named feature toggles (mpc = use_mpc,
# cp fb = use_cp, zmp fb = use_zmp_fb, lag sys = use_lag,
# openloop ref = open_loop) plus an explicit display label. Run args and
# displayed flags are derived from these -- no hardcoded booleans to drift.
FLAG_NAMES = ["mpc", "cp fb", "zmp fb", "lag sys", "openloop ref"]

CONFIGS = [
    {"label": "ISMPC (no cp)",            "mpc": True,  "cp fb": False, "zmp fb": False,  "lag sys": False, "openloop ref": False},
    {"label": "plain",                    "mpc": True,  "cp fb": True,  "zmp fb": True,  "lag sys": False, "openloop ref": False},
    {"label": "no zmp fb",                "mpc": True,  "cp fb": True,  "zmp fb": False, "lag sys": False, "openloop ref": False},
    {"label": "lag",                      "mpc": True,  "cp fb": True,  "zmp fb": True,  "lag sys": True,  "openloop ref": False},
    {"label": "openloop",                 "mpc": True,  "cp fb": True,  "zmp fb": True,  "lag sys": False, "openloop ref": True},
    {"label": "CP controller",            "mpc": False, "cp fb": True,  "zmp fb": True,  "lag sys": False, "openloop ref": False},
    {"label": "CP controller, no zmp fb", "mpc": False, "cp fb": True,  "zmp fb": False, "lag sys": False, "openloop ref": False},
    {"label": "CP controller, lag",       "mpc": False, "cp fb": True,  "zmp fb": True,  "lag sys": True,  "openloop ref": False},
]


def config_label(cfg):
    return cfg["label"]


def display_flags(cfg):
    """Flags as shown: the CP controller (no mpc) always retrieves open-loop refs."""
    d = {n: cfg[n] for n in FLAG_NAMES}
    d["openloop ref"] = cfg["openloop ref"] or not cfg["mpc"]
    return d


def run_config(use_mpc, use_cp, use_zmp_fb, use_lag, open_loop):
    world, hrp4 = build_world()
    node = Hrp4Controller(world, hrp4, log_path=None, autosave_every=0,
                          use_kf=True, use_mpc=use_mpc, use_cp=use_cp,
                          open_loop=open_loop, use_lag=use_lag,
                          use_zmp_fb=use_zmp_fb)
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

    from scipy.spatial.transform import Rotation as R
    w = WARMUP_STEPS
    zmp_err = (arr('desired', 'zmp', 'pos') - arr('current', 'zmp', 'pos'))[w:]
    com_err = (arr('desired', 'com', 'pos') - arr('current', 'com', 'pos'))[w:]
    d_att = R.from_rotvec(arr('desired', 'base', 'pos')).as_euler("xyz", degrees=True)[:, :2]
    c_att = R.from_rotvec(arr('current', 'base', 'pos')).as_euler("xyz", degrees=True)[:, :2]
    att_err = (d_att - c_att)[w:]
    fz = arr('current', 'grf', 'force')[w:, 2]
    import matplotlib.pyplot as plt
    plt.close('all')
    return metrics_of({'zmp_err': zmp_err, 'com_err': com_err, 'att_err': att_err,
                       'fz': fz, 'diverged_at': diverged_at})


# ---- formatting ------------------------------------------------------------

def fmt_mm(x, dv=False):
    return "—" if dv or x != x else f"{x*1000:.2f}"


def fmt_deg(x, dv=False):
    return "—" if dv or x != x else f"{x:.3f}"


def fmt_n(x, dv=False):
    return "—" if dv or x != x else f"{x:.1f}"


ERR_HDR = ("ZMP x [mm] | ZMP y [mm] | ZMP z [mm] | "
           "COM x [mm] | COM y [mm] | COM z [mm] | "
           "waist roll [deg] | waist pitch [deg]")
ERR_SEP = "---:|---:|---:|---:|---:|---:|---:|---:"


def err_cells(m, stat, dv):
    """The 8 per-axis error cells for a given stat ('avg'|'rmse'|'max')."""
    return " | ".join([
        fmt_mm(m[f'zmp_x_{stat}'], dv), fmt_mm(m[f'zmp_y_{stat}'], dv), fmt_mm(m[f'zmp_z_{stat}'], dv),
        fmt_mm(m[f'com_x_{stat}'], dv), fmt_mm(m[f'com_y_{stat}'], dv), fmt_mm(m[f'com_z_{stat}'], dv),
        fmt_deg(m[f'att_roll_{stat}'], dv), fmt_deg(m[f'att_pitch_{stat}'], dv),
    ])


# errors are reported as RMSE; the vertical force is reported as its peak.
FZ_HDR, FZ_KEY = "max Fz [N]", 'fz_max'


# ---- table renderers -------------------------------------------------------

def config_subtable(rows):
    flag_hdr = " | ".join(FLAG_NAMES)
    flag_sep = "|".join([":---:"] * len(FLAG_NAMES))
    lines = [
        f"| config | {flag_hdr} | {ERR_HDR} | {FZ_HDR} | diverged |",
        f"|---|{flag_sep}|{ERR_SEP}|---:|:---:|",
    ]
    for label, flags, m in rows:
        dv = m['diverged']
        flag_cells = " | ".join("✓" if flags[n] else "·" for n in FLAG_NAMES)
        lines.append(
            f"| {label} | {flag_cells} | {err_cells(m, 'rmse', dv)} | "
            f"{fmt_n(m[FZ_KEY], dv)} | {'yes' if dv else 'no'} |"
        )
    return lines


def ablation_subtable(param_label, xs, metrics):
    lines = [
        f"| {param_label} | {ERR_HDR} | {FZ_HDR} | diverged |",
        f"|---:|{ERR_SEP}|---:|:---:|",
    ]
    for x, m in zip(xs, metrics):
        dv = m['diverged']
        lines.append(
            f"| {x:g} | {err_cells(m, 'rmse', dv)} | {fmt_n(m[FZ_KEY], dv)} | "
            f"{'yes' if dv else 'no'} |"
        )
    return lines


def config_section():
    rows = []
    for cfg in CONFIGS:
        label = config_label(cfg)
        print(f"Running config: {label} ...", flush=True)
        m = run_config(cfg["mpc"], cfg["cp fb"], cfg["zmp fb"], cfg["lag sys"],
                       cfg["openloop ref"])
        rows.append((label, display_flags(cfg), m))

    out = [
        "## Configuration comparison (kf on)",
        "",
        "Fresh headless runs on the current code/params. Each variant flips one "
        "feature off the **plain** baseline (ISMPC + capture-point + ZMP feedback).",
        "",
        "Flag columns:",
        "",
        "- **mpc** — intrinsically-stable MPC (off = the CP feedback controller)",
        "- **cp fb** — capture-point feedback correction on the ZMP command",
        "- **zmp fb** — `k_2*(p-p_ref)` ZMP-position feedback term (`use_zmp_fb`)",
        "- **lag sys** — ZMP first-order-lag plant dynamics (`use_lag`)",
        "- **openloop ref** — track open-loop references (always on for the CP "
        "controller, which retrieves them by construction)",
        "",
        *config_subtable(rows),
        "",
    ]
    return "\n".join(out)


def ablation_section(title, param_label, xs, runs, note=""):
    metrics = [metrics_of(r) for r in runs]
    out = [f"### {title}", ""]
    if note:
        out += [note, ""]
    out += [
        *ablation_subtable(param_label, xs, metrics),
        "",
    ]
    return "\n".join(out)


def load_pkl(path):
    with open(path, 'rb') as f:
        return pickle.load(f)


def main():
    parts = [
        "# Runs summary",
        "",
        "Per-axis error over each run after a warm-up of "
        f"{WARMUP_STEPS} steps, reported as `RMSE = sqrt(mean(e^2))`; the "
        "vertical force column is the peak (`max Fz`). "
        "Positions in mm, waist attitude in deg, force in N. "
        "A `diverged` run fell (MPC QP blew up) and its metrics are not meaningful.",
        "",
        config_section(),
        "## Ablation studies",
        "",
        "Each row is one run of a parameter sweep, recomputed from the study's "
        "cached raw series. Per-axis plots live in the matching `logs/` dir.",
        "",
    ]

    poles = load_pkl("logs/ablation_poles/raw_runs.pkl")
    for pole in ("alpha", "beta", "gamma"):
        d = poles[pole]
        parts.append(ablation_section(f"Poles — {pole}", pole,
                                      d['values'], d['runs']))

    gp = poles['g_p']
    parts.append(ablation_section(
        "ZMP-lag gain g_p", "g_p", gp['values'], gp['runs']))

    with open(OUT_MD, "w") as f:
        f.write("\n".join(parts))
    print(f"Wrote {OUT_MD}", flush=True)


if __name__ == "__main__":
    main()
