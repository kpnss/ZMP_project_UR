import argparse
import os
from pathlib import Path
import numpy as np
import matplotlib.pyplot as plt


AXES = ("x", "y", "z")


def load_run(path):
    data = np.load(path, allow_pickle=False)
    time_step = float(data["time_step"]) if "time_step" in data else 0.01
    use_kf    = bool(data["meta_use_kf"])    if "meta_use_kf"    in data else None
    use_mpc   = bool(data["meta_use_mpc"])   if "meta_use_mpc"   in data else None
    open_loop = bool(data["meta_open_loop"]) if "meta_open_loop" in data else False
    return data, time_step, use_kf, use_mpc, open_loop


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


def plot_single_run(log_path, run_label, eta):
    data, dt, use_kf, use_mpc, open_loop = load_run(log_path)
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
    tag = f"{kf_lbl}{mpc_lbl}"
    sfx = f"{kf_sfx}{mpc_sfx}"

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

    plt.tight_layout()

    return [
        (fig1, f"plot{sfx}_{stem}_com_trajectories.png"),
        (fig2, f"plot{sfx}_{stem}_zmp_trajectories.png"),
        (fig3, f"plot{sfx}_{stem}_capture_point_trajectories.png"),
        (fig4, f"plot{sfx}_{stem}_tracking_errors.png"),
    ]


def plot_comparison(log_paths, eta):
    runs = []
    for path in log_paths:
        data, dt, use_kf, use_mpc, open_loop = load_run(path)
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
        runs.append({
            "label": stem + kf_label(use_kf) + mpc_label(use_mpc, open_loop),
            "dt": dt,
            "t_com": np.arange(n_com) * dt,
            "t_zmp": np.arange(n_zmp) * dt,
            "t_xi":  np.arange(n_xi)  * dt,
            "com_err": d_com - c_com,
            "zmp_err": d_zmp - c_zmp,
            "xi_err":  d_xi  - c_xi,
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

    plt.tight_layout()

    return [
        (fig,  "comparison_com_errors.png"),
        (fig2, "comparison_zmp_errors.png"),
        (fig3, "comparison_capture_point_errors.png"),
    ]


def main():
    parser = argparse.ArgumentParser(description="Plot simulation logs exported by Logger.save_npz.")
    parser.add_argument("logs", nargs="*", help="One or more .npz log files (default: latest in logs/).")
    parser.add_argument(
        "--compare",
        action="store_true",
        help="Compare tracking errors across multiple runs.",
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
        candidates = sorted(Path("logs").glob("*.npz"), key=lambda p: p.stat().st_ctime)
        if not candidates:
            raise FileNotFoundError("No .npz files found in logs/")
        args.logs = [str(candidates[-1])]
        print(f"Auto-selected: {args.logs[0]}")

    if args.compare and len(args.logs) < 2:
        raise ValueError("--compare requires at least two log files.")

    eta = args.eta if args.eta is not None else np.sqrt(args.gravity / args.lip_height)

    if args.compare:
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