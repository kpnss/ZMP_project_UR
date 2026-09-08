"""Generate runs_summary.md: a table of every run we did (config comparison +
ablations).

- Config comparison: reruns headless configs (kf on) that isolate the
  mpc/no-mpc, cp/no-cp, lag/no-lag and open-loop axes. Each run is also saved
  to `logs/<stem>.npz` (same naming as simulation.py) for replotting.
- Ablations: recomputes metrics from each study's cached `raw_runs.pkl`
  (raw error series -> no re-simulation needed).

Per axis, over the N post-warm-up steps of the error series e[t], errors are
reported as RMSE = sqrt(mean(e^2)); the vertical reaction force is reported as
its peak (max Fz). Positions in mm, waist attitude in deg, force in N.
`diverged` = the run fell (MPC QP blew up); its metrics are not meaningful.

Usage:  python make_runs_summary.py
"""
import os
import pickle
import numpy as np

from ablation_poles import (build_world, suppress_c_output, metrics_of,
                            WARMUP_STEPS)
from simulation import Hrp4Controller
from plot_logs import kf_suffix, mpc_suffix, cp_suffix, variant_label

OUT_MD = "runs_summary.md"
# each config run is also dumped here as `<stem>.npz`, using the same file
# naming as an interactive `simulation.py` run, so plot_logs.py can pick them up
LOG_DIR = "logs"


# ---- config comparison (fresh headless runs) -------------------------------

# Each config is defined by the named feature toggles (mpc = use_mpc,
# cp fb = use_cp, zmp fb = use_zmp_fb, lag sys = use_lag,
# openloop ref = open_loop). Run args, displayed flags, the log stem and the
# display label are all derived from these -- no hardcoded booleans to drift.
# Labels live in plot_logs.VARIANT_LABELS so the plots use the same names.
FLAG_NAMES = ["mpc", "cp fb", "zmp fb", "lag sys", "openloop ref"]

CONFIGS = [
    {"mpc": True,  "cp fb": False, "zmp fb": False,  "lag sys": False, "openloop ref": False},
    {"mpc": True,  "cp fb": True,  "zmp fb": True,  "lag sys": False, "openloop ref": False},
    {"mpc": True,  "cp fb": True,  "zmp fb": False, "lag sys": False, "openloop ref": False},
    {"mpc": True,  "cp fb": True,  "zmp fb": True,  "lag sys": True,  "openloop ref": False},
    {"mpc": True,  "cp fb": True,  "zmp fb": True,  "lag sys": False, "openloop ref": True},
    {"mpc": False, "cp fb": True,  "zmp fb": True,  "lag sys": False, "openloop ref": False},
    {"mpc": False, "cp fb": True,  "zmp fb": False, "lag sys": False, "openloop ref": False},
    {"mpc": False, "cp fb": True,  "zmp fb": True,  "lag sys": True,  "openloop ref": False},
]


def config_label(cfg):
    return variant_label(config_stem(cfg))


def config_stem(cfg):
    """Log stem for a config, matching the suffix simulation.py builds."""
    stem = ("log" + kf_suffix(True) + mpc_suffix(cfg["mpc"])
            + cp_suffix(cfg["cp fb"]))
    # the k_2*(p-p_ref) term lives inside the CP feedback law, so with cp fb off
    # the zmp fb flag is a no-op and does not earn a suffix
    if cfg["cp fb"] and not cfg["zmp fb"]:
        stem += "_nozmpfb"
    if cfg["openloop ref"]:
        stem += "_openloop"
    if cfg["lag sys"]:
        stem += "_lag"
    return stem


def display_flags(cfg):
    """Flags as shown: the CP controller (no mpc) always retrieves open-loop refs."""
    d = {n: cfg[n] for n in FLAG_NAMES}
    d["openloop ref"] = cfg["openloop ref"] or not cfg["mpc"]
    return d


def run_config(use_mpc, use_cp, use_zmp_fb, use_lag, open_loop, log_stem=None):
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

    # dump the raw series so plot_logs.py can replot these exact runs
    if log_stem is not None:
        log_path = os.path.join(LOG_DIR, f"{log_stem}.npz")
        node.logger.save_npz(
            log_path,
            time_step=node.params['world_time_step'],
            use_kf=node.use_kf,
            use_mpc=node.use_mpc,
            open_loop=node.open_loop,
            use_cp=node.use_cp,
            steps=node.time,
        )
        print(f"  saved {log_path}", flush=True)

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
    # poles/design params actually used by the config runs (read from a throwaway node)
    _w, _hrp4 = build_world()
    _pp = Hrp4Controller(_w, _hrp4, log_path=None, autosave_every=0, use_kf=True,
                         use_mpc=True, use_cp=True, open_loop=False, use_lag=False,
                         use_zmp_fb=True).params
    import matplotlib.pyplot as _plt; _plt.close('all')
    poles_line = (f"Poles used for this table: `alpha = {_pp['alpha']:g}`, "
                  f"`beta = {_pp['beta']:g}`, `gamma = {_pp['gamma']:g}`; "
                  f"ZMP-lag gain `g_p = {_pp['g_p']:g}`; `eta = {_pp['eta']:.3f}`.")

    rows = []
    for cfg in CONFIGS:
        label = config_label(cfg)
        print(f"Running config: {label} ...", flush=True)
        m = run_config(cfg["mpc"], cfg["cp fb"], cfg["zmp fb"], cfg["lag sys"],
                       cfg["openloop ref"], log_stem=config_stem(cfg))
        rows.append((label, display_flags(cfg), m))

    out = [
        "## Configuration comparison (kf on)",
        "",
        poles_line,
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
        "The **ZMP error** is measured against `p_ref` -- the MPC ZMP feedforward "
        "for the current instant as predicted at the previous step (for the CP "
        "controller: its planned ZMP) -- i.e. the SAME reference the capture-point "
        "ZMP-feedback term regulates, NOT the realized command `p_cmd`. Because CP "
        "feedback intentionally drives the commanded ZMP away from `p_ref` to correct "
        "capture-point errors, a larger ZMP error here does not by itself mean worse "
        "balance.",
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
