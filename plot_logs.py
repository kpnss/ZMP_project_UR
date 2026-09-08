import argparse
import os
import re
from pathlib import Path
import numpy as np
import matplotlib.pyplot as plt
from scipy.spatial.transform import Rotation as R


AXES = ("x", "y", "z")
# waist attitude angles measured in Balance_control.pdf Fig. 7(a): roll (x), pitch (y)
ATTITUDE_ANGLES = ("roll", "pitch")


def load_run(path):
    data = np.load(path, allow_pickle=False)
    time_step = float(data["time_step"]) if "time_step" in data else 0.01
    use_kf    = bool(data["meta_use_kf"])    if "meta_use_kf"    in data else None
    use_mpc   = bool(data["meta_use_mpc"])   if "meta_use_mpc"   in data else None
    use_cp    = bool(data["meta_use_cp"])    if "meta_use_cp"    in data else None
    open_loop = bool(data["meta_open_loop"]) if "meta_open_loop" in data else False
    return data, time_step, use_kf, use_mpc, use_cp, open_loop


def kf_label(use_kf):
    if use_kf is None:
        return ""
    return " [KF]" if use_kf else " [no KF]"


def kf_suffix(use_kf):
    if use_kf is None:
        return ""
    return "_kf" if use_kf else "_no_kf"


def mpc_label(use_mpc, open_loop=False):
    if open_loop:
        return " [open-loop MPC]"
    if use_mpc is None:
        return ""
    return " [MPC]" if use_mpc else " [CP ctrl]"


def mpc_suffix(use_mpc, open_loop=False):
    if open_loop:
        return "_open_loop"
    if use_mpc is None:
        return ""
    return "_mpc" if use_mpc else "_no_mpc"


def cp_label(use_cp):
    if use_cp is None:
        return ""
    return " [CP fb]" if use_cp else " [no CP fb]"


def cp_suffix(use_cp):
    if use_cp is None:
        return ""
    return "_cp" if use_cp else "_no_cp"


def rotvec_to_attitude_deg(rotvec):
    # (N, 3) rotation vectors -> roll, pitch (deg), the waist attitude of Fig. 7(a)
    euler = R.from_rotvec(np.asarray(rotvec)).as_euler("xyz", degrees=True)
    return euler[:, 0:2]


def rms(values):
    return float(np.sqrt(np.mean(np.square(values))))


def get_series(data, batch, item, level):
    key = f"{batch}_{item}_{level}"
    if key not in data:
        raise KeyError(f"Missing key '{key}' in log file.")
    return data[key]


def add_position_plots(ax, desired, current, title, label_prefix):
    t_des = np.arange(len(desired))
    t_cur = np.arange(len(current))

    for dim, axis_name in enumerate(AXES):
        ax[dim].plot(t_des, desired[:, dim], "-",  label=f"{label_prefix} desired {axis_name}")
        ax[dim].plot(t_cur, current[:, dim], "--", label=f"{label_prefix} current {axis_name}")
        ax[dim].set_ylabel(f"{title} {axis_name} [m]")
        ax[dim].grid(True, alpha=0.3)
    ax[-1].set_xlabel("sample")


def add_error_plots(ax, t, desired, current, title, label_prefix):
    error = desired - current
    for dim, axis_name in enumerate(AXES):
        ax[dim].plot(t, error[:, dim], label=f"{label_prefix} {title} err {axis_name}")
        ax[dim].set_ylabel(f"{title} err {axis_name} [m]")
        ax[dim].grid(True, alpha=0.3)
    ax[-1].set_xlabel("time [s]")


def truncate_pair(a, b):
    n = min(len(a), len(b))
    return a[:n], b[:n], n


def compute_xi(com_pos, com_vel, eta):
    return com_pos + com_vel / eta


def add_attitude_plots(ax, t, desired, current, label_prefix):
    d_att = rotvec_to_attitude_deg(desired)
    c_att = rotvec_to_attitude_deg(current)
    for dim, angle in enumerate(ATTITUDE_ANGLES):
        err_rms = rms(d_att[:, dim] - c_att[:, dim])
        ax[dim].plot(t, d_att[:, dim], "-",  label=f"{label_prefix} desired {angle}")
        ax[dim].plot(t, c_att[:, dim], "--", label=f"{label_prefix} current {angle} (RMS err {err_rms:.3f} deg)")
        ax[dim].set_ylabel(f"waist {angle} [deg]")
        ax[dim].grid(True, alpha=0.3)
    ax[-1].set_xlabel("time [s]")


def plot_single_run(log_path, run_label, eta):
    data, dt, use_kf, use_mpc, use_cp, open_loop = load_run(log_path)
    d_com, c_com, n_com = truncate_pair(
        get_series(data, "desired", "com", "pos"),
        get_series(data, "current", "com", "pos"),
    )
    d_com_vel = get_series(data, "desired", "com", "vel")
    c_com_vel = get_series(data, "current", "com", "vel")
    d_zmp, c_zmp, n_zmp = truncate_pair(
        get_series(data, "desired", "zmp", "pos"),
        get_series(data, "current", "zmp", "pos"),
    )

    n_xi = min(len(d_com), len(c_com), len(d_com_vel), len(c_com_vel))
    d_xi = compute_xi(d_com[:n_xi], d_com_vel[:n_xi], eta)
    c_xi = compute_xi(c_com[:n_xi], c_com_vel[:n_xi], eta)

    t_com = np.arange(n_com) * dt
    t_zmp = np.arange(n_zmp) * dt
    t_xi  = np.arange(n_xi)  * dt

    stem    = Path(log_path).stem
    kf_lbl  = kf_label(use_kf)
    kf_sfx  = kf_suffix(use_kf)
    mpc_lbl = mpc_label(use_mpc, open_loop)
    mpc_sfx = mpc_suffix(use_mpc, open_loop)
    tag = f"{kf_lbl}{mpc_lbl}{cp_label(use_cp)}"
    sfx = f"{kf_sfx}{mpc_sfx}{cp_suffix(use_cp)}"

    fig1, ax1 = plt.subplots(3, 1, figsize=(10, 8), sharex=True)
    fig1.suptitle(f"COM trajectories{tag} - {run_label}")
    add_position_plots(ax1, d_com, c_com, "COM", run_label)
    ax1[0].legend(loc="upper right", fontsize=8)

    fig2, ax2 = plt.subplots(3, 1, figsize=(10, 8), sharex=True)
    fig2.suptitle(f"ZMP trajectories{tag} - {run_label}")
    add_position_plots(ax2, d_zmp, c_zmp, "ZMP", run_label)
    ax2[0].legend(loc="upper right", fontsize=8)

    fig3, ax3 = plt.subplots(3, 1, figsize=(10, 8), sharex=True)
    fig3.suptitle(f"Capture point trajectories{tag} - {run_label}")
    add_position_plots(ax3, d_xi, c_xi, "CP", run_label)
    ax3[0].legend(loc="upper right", fontsize=8)

    fig4, ax4 = plt.subplots(3, 1, figsize=(10, 8), sharex=True)
    fig4.suptitle(f"Tracking errors (desired-current){tag} - {run_label}")
    add_error_plots(ax4, t_com, d_com, c_com, "COM", run_label)
    add_error_plots(ax4, t_zmp, d_zmp, c_zmp, "ZMP", run_label)
    ax4[0].legend(loc="upper right", fontsize=8)

    figures = [
        (fig1, f"com_trajectories.png"),
        (fig2, f"zmp_trajectories.png"),
        (fig3, f"capture_point_trajectories.png"),
        (fig4, f"tracking_errors.png"),
    ]

    # waist attitude (Balance_control.pdf Fig. 7(a))
    if "current_base_pos" in data:
        d_base, c_base, n_base = truncate_pair(
            get_series(data, "desired", "base", "pos"),
            get_series(data, "current", "base", "pos"),
        )
        t_base = np.arange(n_base) * dt
        fig5, ax5 = plt.subplots(2, 1, figsize=(10, 6), sharex=True)
        fig5.suptitle(f"Waist attitude{tag} - {run_label}")
        add_attitude_plots(ax5, t_base, d_base, c_base, run_label)
        ax5[0].legend(loc="upper right", fontsize=8)
        figures.append((fig5, f"waist_attitude.png"))

    # vertical reaction force (Balance_control.pdf Fig. 7(d))
    if "current_grf_force" in data:
        grf = get_series(data, "current", "grf", "force")
        fz = grf[:, 2]
        t_grf = np.arange(len(fz)) * dt
        fz_max = float(np.max(fz))
        fig6, ax6 = plt.subplots(1, 1, figsize=(10, 4))
        fig6.suptitle(f"Vertical reaction force{tag} - {run_label}")
        ax6.plot(t_grf, fz, label=f"{run_label} Fz")
        ax6.axhline(fz_max, color="red", linestyle="--", label=f"max {fz_max:.1f} N")
        ax6.set_ylabel("vertical reaction force [N]")
        ax6.set_xlabel("time [s]")
        ax6.grid(True, alpha=0.3)
        ax6.legend(loc="upper right", fontsize=8)
        figures.append((fig6, f"vertical_force.png"))

    plt.tight_layout()

    return figures


def plot_comparison(log_paths, eta):
    runs = []
    for path in log_paths:
        data, dt, use_kf, use_mpc, use_cp, open_loop = load_run(path)
        d_com, c_com, n_com = truncate_pair(
            get_series(data, "desired", "com", "pos"),
            get_series(data, "current", "com", "pos"),
        )
        d_com_vel = get_series(data, "desired", "com", "vel")
        c_com_vel = get_series(data, "current", "com", "vel")
        d_zmp, c_zmp, n_zmp = truncate_pair(
            get_series(data, "desired", "zmp", "pos"),
            get_series(data, "current", "zmp", "pos"),
        )

        n_xi = min(len(d_com), len(c_com), len(d_com_vel), len(c_com_vel))
        d_xi = compute_xi(d_com[:n_xi], d_com_vel[:n_xi], eta)
        c_xi = compute_xi(c_com[:n_xi], c_com_vel[:n_xi], eta)
        stem = Path(path).stem

        att_err = None
        if "current_base_pos" in data:
            d_base, c_base, _ = truncate_pair(
                get_series(data, "desired", "base", "pos"),
                get_series(data, "current", "base", "pos"),
            )
            att_err = rotvec_to_attitude_deg(d_base) - rotvec_to_attitude_deg(c_base)

        grf_z = None
        if "current_grf_force" in data:
            grf_z = get_series(data, "current", "grf", "force")[:, 2]

        runs.append({
            "label": stem + kf_label(use_kf) + mpc_label(use_mpc, open_loop) + cp_label(use_cp),
            "dt": dt,
            "t_com": np.arange(n_com) * dt,
            "t_zmp": np.arange(n_zmp) * dt,
            "t_xi":  np.arange(n_xi)  * dt,
            "com_err": d_com - c_com,
            "zmp_err": d_zmp - c_zmp,
            "xi_err":  d_xi  - c_xi,
            "att_err": att_err,
            "grf_z":   grf_z,
        })

    fig, ax = plt.subplots(3, 1, figsize=(10, 8), sharex=True)
    fig.suptitle("COM tracking error comparison")
    for run in runs:
        for dim, axis_name in enumerate(AXES):
            ax[dim].plot(run["t_com"], run["com_err"][:, dim], label=f'{run["label"]} {axis_name}')
            ax[dim].set_ylabel(f"COM err {axis_name} [m]")
            ax[dim].grid(True, alpha=0.3)
    ax[-1].set_xlabel("time [s]")
    ax[0].legend(loc="upper right", fontsize=8)

    fig2, ax2 = plt.subplots(3, 1, figsize=(10, 8), sharex=True)
    fig2.suptitle("ZMP tracking error comparison")
    for run in runs:
        for dim, axis_name in enumerate(AXES):
            ax2[dim].plot(run["t_zmp"], run["zmp_err"][:, dim], label=f'{run["label"]} {axis_name}')
            ax2[dim].set_ylabel(f"ZMP err {axis_name} [m]")
            ax2[dim].grid(True, alpha=0.3)
    ax2[-1].set_xlabel("time [s]")
    ax2[0].legend(loc="upper right", fontsize=8)

    fig3, ax3 = plt.subplots(3, 1, figsize=(10, 8), sharex=True)
    fig3.suptitle("Capture point tracking error comparison")
    for run in runs:
        for dim, axis_name in enumerate(AXES):
            ax3[dim].plot(run["t_xi"], run["xi_err"][:, dim], label=f'{run["label"]} {axis_name}')
            ax3[dim].set_ylabel(f"CP err {axis_name} [m]")
            ax3[dim].grid(True, alpha=0.3)
    ax3[-1].set_xlabel("time [s]")
    ax3[0].legend(loc="upper right", fontsize=8)

    figures = [
        (fig,  "comparison_com_errors.png"),
        (fig2, "comparison_zmp_errors.png"),
        (fig3, "comparison_capture_point_errors.png"),
    ]

    # waist attitude error comparison (Balance_control.pdf Fig. 7(a))
    att_runs = [r for r in runs if r["att_err"] is not None]
    if att_runs:
        fig4, ax4 = plt.subplots(2, 1, figsize=(10, 6), sharex=True)
        fig4.suptitle("Waist attitude error comparison")
        for run in att_runs:
            t = np.arange(len(run["att_err"])) * run["dt"]
            for dim, angle in enumerate(ATTITUDE_ANGLES):
                r = rms(run["att_err"][:, dim])
                ax4[dim].plot(t, run["att_err"][:, dim], label=f'{run["label"]} {angle} (RMS {r:.3f})')
                ax4[dim].set_ylabel(f"{angle} err [deg]")
                ax4[dim].grid(True, alpha=0.3)
        ax4[-1].set_xlabel("time [s]")
        ax4[0].legend(loc="upper right", fontsize=8)
        figures.append((fig4, "comparison_waist_attitude.png"))

    # vertical reaction force comparison (Balance_control.pdf Fig. 7(d))
    grf_runs = [r for r in runs if r["grf_z"] is not None]
    if grf_runs:
        fig5, ax5 = plt.subplots(1, 1, figsize=(10, 4))
        fig5.suptitle("Vertical reaction force comparison")
        for run in grf_runs:
            t = np.arange(len(run["grf_z"])) * run["dt"]
            fz_max = float(np.max(run["grf_z"]))
            ax5.plot(t, run["grf_z"], label=f'{run["label"]} (max {fz_max:.1f} N)')
        ax5.set_ylabel("vertical reaction force [N]")
        ax5.set_xlabel("time [s]")
        ax5.grid(True, alpha=0.3)
        ax5.legend(loc="upper right", fontsize=8)
        figures.append((fig5, "comparison_vertical_force.png"))

    plt.tight_layout()

    return figures


# ---- ZMP error replot on a shared scale ------------------------------------
# Each run's own figure is auto-scaled, so two runs cannot be compared by eye.
# These build one symmetric y-limit per axis (and one x-limit) across a set of
# logs, then re-render every run with those limits. The MPC and CP-controller
# runs differ by ~70x in ZMP error, so --split-mpc scales each family on its own
# and marks the MPC bounds on the CP plots for reference.

ZMP_ERR_NAME = "zmp_errors.png"
ZMP_ERR_COMPARISON_NAME = "comparison_zmp_errors_same_scale.png"
ZMP_ERR_GROUP_NAMES = {True: "comparison_zmp_errors_mpc.png",
                       False: "comparison_zmp_errors_no_mpc.png"}
# axes kept on ONE scale across every run even under --split-mpc: the two
# families do not separate in z the way they do in x/y, so splitting it there
# only makes the two plots harder to compare
SHARED_ZMP_AXES = ("z",)

# Display name per run stem -- the config labels of the runs summary. Single
# source of truth: make_runs_summary.config_label() reads this too, so the table
# rows, the plot titles and the logs/zmp_plots/ links cannot drift apart.
VARIANT_LABELS = {
    "log_kf_mpc_no_cp":         "ISMPC (no cp)",
    "log_kf_mpc_cp":            "plain",
    "log_kf_mpc_cp_nozmpfb":    "no zmp fb",
    "log_kf_mpc_cp_lag":        "lag",
    "log_kf_mpc_cp_openloop":   "openloop",
    "log_kf_no_mpc_cp":         "CP controller",
    "log_kf_no_mpc_cp_nozmpfb": "CP controller, no zmp fb",
    "log_kf_no_mpc_cp_lag":     "CP controller, lag",
}


def variant_label(stem):
    """Summary label for a run stem; unknown stems keep their filename."""
    return VARIANT_LABELS.get(stem, stem)


def variant_slug(label):
    """Filename-safe variant name: punctuation dropped, spaces to hyphens."""
    cleaned = re.sub(r"[^0-9A-Za-z ]+", " ", label)
    return re.sub(r"-+", "-", "-".join(cleaned.split()))


def collect_zmp_errors(log_paths, warmup):
    runs = []
    for path in log_paths:
        data, dt, use_kf, use_mpc, use_cp, open_loop = load_run(path)
        d_zmp, c_zmp, n = truncate_pair(
            get_series(data, "desired", "zmp", "pos"),
            get_series(data, "current", "zmp", "pos"),
        )
        runs.append({
            "stem": Path(path).stem,
            "label": variant_label(Path(path).stem),
            "tag": kf_label(use_kf) + mpc_label(use_mpc, open_loop) + cp_label(use_cp),
            "use_mpc": bool(use_mpc),
            "t": (np.arange(n) * dt)[warmup:],
            "err": (d_zmp - c_zmp)[warmup:],
        })
    return runs


def common_zmp_ylim(runs, percentile, margin=1.05):
    """One symmetric y-limit per axis, shared by every run in the group."""
    ylim = []
    for dim in range(len(AXES)):
        stacked = np.concatenate([np.abs(r["err"][:, dim]) for r in runs])
        bound = stacked.max() if percentile >= 100 else np.percentile(stacked, percentile)
        bound = max(float(bound), 1e-6) * margin
        ylim.append((-bound, bound))
    return ylim


def common_zmp_xlim(runs):
    return (0.0, max(float(r["t"][-1]) for r in runs))


def apply_zmp_limits(ax, ylim, xlim, ref_ylim=None):
    """Set the shared limits; ref_ylim marks another group's scale for reference."""
    for dim, axis_name in enumerate(AXES):
        if ref_ylim is not None and ref_ylim[dim] is not None:
            bound = ref_ylim[dim][1]
            visible = bound <= ylim[dim][1]
            label = f"MPC scale +/-{bound * 1000:.2f} mm"
            if visible:
                for sign in (1, -1):
                    ax[dim].axhline(sign * bound, color="tab:red", ls="--", lw=1.0,
                                    alpha=0.8, label=label if sign == 1 else None)
            else:
                # the MPC family is wider on this axis, so its bounds fall off-plot
                ax[dim].plot([], [], color="tab:red", ls="--", lw=1.0,
                             label=f"{label} (off-scale)")
        ax[dim].set_ylabel(f"ZMP err {axis_name} [m]")
        ax[dim].set_ylim(*ylim[dim])
        ax[dim].set_xlim(*xlim)
        ax[dim].grid(True, alpha=0.3)
    ax[-1].set_xlabel("time [s]")


def plot_zmp_error_run(run, ylim, xlim, note, ref_ylim=None):
    fig, ax = plt.subplots(3, 1, figsize=(10, 8), sharex=True)
    fig.suptitle(f"ZMP tracking errors - {run['label']}{note}")
    for dim, axis_name in enumerate(AXES):
        e = run["err"][:, dim]
        ax[dim].axhline(0.0, color="grey", lw=0.8, alpha=0.6)
        ax[dim].plot(run["t"], e, label=f"{axis_name} (RMS {rms(e) * 1000:.2f} mm)")
    apply_zmp_limits(ax, ylim, xlim, ref_ylim)
    for dim in range(len(AXES)):
        ax[dim].legend(loc="upper right", fontsize=8)
    fig.tight_layout()
    return fig


def plot_zmp_error_overlay(runs, ylim, xlim, note, title, ref_ylim=None):
    fig, ax = plt.subplots(3, 1, figsize=(10, 8), sharex=True)
    fig.suptitle(f"{title}{note}")
    for run in runs:
        for dim in range(len(AXES)):
            ax[dim].plot(run["t"], run["err"][:, dim], lw=0.9,
                         label=f"{run['stem']}{run['tag']}")
    apply_zmp_limits(ax, ylim, xlim, ref_ylim)
    ax[0].legend(loc="upper right", fontsize=7)
    # the run labels only need listing once, but each axis has its own reference
    # bound, so give the lower panels a legend holding just that entry
    for dim in range(1, len(AXES)):
        handles, labels = ax[dim].get_legend_handles_labels()
        ref = [(h, l) for h, l in zip(handles, labels) if l.startswith("MPC scale")]
        if ref:
            ax[dim].legend([h for h, _ in ref], [l for _, l in ref],
                           loc="upper right", fontsize=7)
    fig.tight_layout()
    return fig


def save_fig(fig, out_dir, name):
    os.makedirs(out_dir, exist_ok=True)
    out_path = os.path.join(out_dir, name)
    fig.savefig(out_path, dpi=300)
    plt.close(fig)
    print(f"Saved: {out_path}")


def report_limits(what, ylim):
    dims = ", ".join(f"{AXES[d]} +/-{ylim[d][1] * 1000:.2f}" for d in range(len(AXES)))
    print(f"{what} ZMP err limits [mm]: {dims}")


def replot_zmp_errors(log_paths, warmup, percentile, split_mpc=False, overlay=True):
    runs = collect_zmp_errors(log_paths, warmup)
    xlim = common_zmp_xlim(runs)  # the time axis stays shared across every run
    note = "" if percentile >= 100 else f"  [y-limits at p{percentile:g}, clipped]"

    global_ylim = common_zmp_ylim(runs, percentile)
    shared = [d for d, name in enumerate(AXES) if name in SHARED_ZMP_AXES]

    # {is_mpc: (runs, ylim)}; one entry means a single scale over everything
    groups = {}
    if split_mpc and any(r["use_mpc"] for r in runs) and any(not r["use_mpc"] for r in runs):
        for is_mpc in (True, False):
            g = [r for r in runs if r["use_mpc"] is is_mpc]
            ylim = common_zmp_ylim(g, percentile)
            for d in shared:
                ylim[d] = global_ylim[d]
            groups[is_mpc] = (g, ylim)
            report_limits("MPC" if is_mpc else "CP ctrl (no MPC)", ylim)
        if shared:
            names = "/".join(SHARED_ZMP_AXES)
            print(f"  ({names} held on one scale across all runs)")
    else:
        groups[None] = (runs, global_ylim)
        report_limits("Common", global_ylim)

    # on the wider CP-controller plots, mark where the MPC scale would sit --
    # except on the axes both families already share, where it is the same line
    mpc_ylim = None
    if True in groups:
        mpc_ylim = [None if d in shared else b for d, b in enumerate(groups[True][1])]

    for key, (group, ylim) in groups.items():
        ref = mpc_ylim if key is False else None
        for run in group:
            save_fig(plot_zmp_error_run(run, ylim, xlim, note, ref),
                     os.path.join("logs", run["stem"]), ZMP_ERR_NAME)
        if overlay and len(group) > 1:
            title = ("ZMP tracking error comparison (common scale)" if key is None
                     else "ZMP tracking error comparison - "
                          + ("MPC runs" if key else "CP-controller runs"))
            name = ZMP_ERR_COMPARISON_NAME if key is None else ZMP_ERR_GROUP_NAMES[key]
            save_fig(plot_zmp_error_overlay(group, ylim, xlim, note, title, ref),
                     os.path.join("logs", "comparison"), name)


def main():
    parser = argparse.ArgumentParser(description="Plot simulation logs exported by Logger.save_npz.")
    parser.add_argument("logs", nargs="*", help="One or more .npz log files (default: latest in logs/).")
    parser.add_argument(
        "--compare",
        action="store_true",
        help="Compare tracking errors across multiple runs.",
    )
    parser.add_argument(
        "--zmp-errors",
        action="store_true",
        help="Replot only the ZMP tracking error, with y-limits shared by every "
             "given log (default: all of logs/*.npz) so the runs are comparable.",
    )
    parser.add_argument(
        "--percentile",
        type=float,
        default=100.0,
        help="With --zmp-errors, take the shared y-limit from this percentile of "
             "|error| instead of the max, so one saturated run does not flatten "
             "every other plot (default: 100 = max).",
    )
    parser.add_argument(
        "--warmup",
        type=int,
        default=0,
        help="With --zmp-errors, drop this many initial samples before "
             "plotting and before computing the shared limits.",
    )
    parser.add_argument(
        "--split-mpc",
        action="store_true",
        help="With --zmp-errors, scale the MPC and CP-controller runs separately "
             "(they differ by ~70x, so one scale flattens the MPC ones) and mark "
             "the MPC bounds on the CP-controller plots for reference.",
    )
    parser.add_argument(
        "--eta",
        type=float,
        default=None,
        help="Natural frequency for CP (default: sqrt(g/h) with g=9.81, h=0.72).",
    )
    parser.add_argument(
        "--gravity",
        type=float,
        default=9.81,
        help="Gravity used to compute eta when --eta is not provided.",
    )
    parser.add_argument(
        "--lip-height",
        type=float,
        default=0.72,
        help="LIP height used to compute eta when --eta is not provided.",
    )
    args = parser.parse_args()

    if not args.logs:
        if args.zmp_errors:
            # the shared scale is only meaningful over the whole set of runs
            args.logs = sorted(str(p) for p in Path("logs").glob("*.npz"))
            if not args.logs:
                raise FileNotFoundError("No .npz files found in logs/")
            print(f"Auto-selected {len(args.logs)} logs from logs/")
        else:
            candidates = sorted(Path("logs").glob("*.npz"), key=lambda p: p.stat().st_ctime)
            if not candidates:
                raise FileNotFoundError("No .npz files found in logs/")
            args.logs = [str(candidates[-1])]
            print(f"Auto-selected: {args.logs[0]}")

    if args.compare and len(args.logs) < 2:
        raise ValueError("--compare requires at least two log files.")

    eta = args.eta if args.eta is not None else np.sqrt(args.gravity / args.lip_height)

    if args.zmp_errors:
        replot_zmp_errors(args.logs, args.warmup, args.percentile,
                          split_mpc=args.split_mpc)
    elif args.compare:
        out_dir = os.path.join("logs", "comparison")
        os.makedirs(out_dir, exist_ok=True)
        for fig, name in plot_comparison(args.logs, eta):
            out_path = os.path.join(out_dir, name)
            fig.savefig(out_path, dpi=300)
            print(f"Saved: {out_path}")
    else:
        for log in args.logs:
            stem = Path(log).stem
            out_dir = os.path.join("logs", stem)
            os.makedirs(out_dir, exist_ok=True)
            for fig, name in plot_single_run(log, stem, eta):
                out_path = os.path.join(out_dir, name)
                fig.savefig(out_path, dpi=300)
                print(f"Saved: {out_path}")


if __name__ == "__main__":
    main()