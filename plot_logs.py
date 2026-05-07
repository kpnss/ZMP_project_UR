import argparse
from pathlib import Path
import numpy as np
import matplotlib.pyplot as plt


AXES = ("x", "y", "z")


def load_run(path):
    data = np.load(path, allow_pickle=False)
    time_step = float(data["time_step"]) if "time_step" in data else 0.01
    return data, time_step


def get_series(data, batch, item, level):
    key = f"{batch}_{item}_{level}"
    if key not in data:
        raise KeyError(f"Missing key '{key}' in log file.")
    return data[key]


def compute_xy_limits(desired, current, xy_limits=None):
    if xy_limits is not None:
        return xy_limits

    all_xy = np.vstack([desired[:, :2], current[:, :2]])
    x_low, x_high = np.percentile(all_xy[:, 0], [1, 99])
    y_low, y_high = np.percentile(all_xy[:, 1], [1, 99])

    # Add a small margin so trajectories are not touching plot borders.
    x_margin = max(1e-3, 0.05 * (x_high - x_low))
    y_margin = max(1e-3, 0.05 * (y_high - y_low))
    return (x_low - x_margin, x_high + x_margin, y_low - y_margin, y_high + y_margin)


def add_position_plots_xy(ax, desired, current, title, label_prefix, xy_limits=None):
    t_des = np.arange(len(desired))
    t_cur = np.arange(len(current))

    ax[0].plot(desired[:, 0], desired[:, 1], "-", alpha=0.7, label=f"{label_prefix} desired xy")
    ax[0].plot(current[:, 0], current[:, 1], "--", alpha=0.7, label=f"{label_prefix} current xy")

    # Overlay all timestamps as points on the same x-y plane.
    ax[0].scatter(desired[:, 0], desired[:, 1], c=t_des, cmap="Blues", s=10, alpha=0.5, label=f"{label_prefix} desired samples")
    ax[0].scatter(current[:, 0], current[:, 1], c=t_cur, cmap="Reds", s=10, alpha=0.5, label=f"{label_prefix} current samples")

    ax[0].set_xlabel(f"{title} x [m]")
    ax[0].set_ylabel(f"{title} y [m]")
    ax[0].axis("equal")
    x_min, x_max, y_min, y_max = compute_xy_limits(desired, current, xy_limits)
    ax[0].set_xlim(x_min, x_max)
    ax[0].set_ylim(y_min, y_max)
    ax[0].grid(True, alpha=0.3)

    ax[1].plot(t_des, desired[:, 2], "-", label=f"{label_prefix} desired z")
    ax[1].plot(t_cur, current[:, 2], "--", label=f"{label_prefix} current z")
    ax[1].set_xlim(0, max(len(desired), len(current)) - 1)
    ax[1].set_ylabel(f"{title} z [m]")
    ax[1].set_xlabel("sample")
    ax[1].grid(True, alpha=0.3)


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


def plot_single_run(log_path, run_label, eta, xy_limits=None):
    data, dt = load_run(log_path)
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
    t_xi = np.arange(n_xi) * dt

    fig1, ax1 = plt.subplots(2, 1, figsize=(10, 7))
    fig1.suptitle(f"COM trajectories - {run_label}")
    add_position_plots_xy(ax1, d_com, c_com, "COM", run_label, xy_limits=xy_limits)
    ax1[0].legend(loc="upper right", fontsize=8)

    fig2, ax2 = plt.subplots(2, 1, figsize=(10, 7))
    fig2.suptitle(f"ZMP trajectories - {run_label}")
    add_position_plots_xy(ax2, d_zmp, c_zmp, "ZMP", run_label, xy_limits=xy_limits)
    ax2[0].legend(loc="upper right", fontsize=8)

    fig3, ax3 = plt.subplots(2, 1, figsize=(10, 7))
    fig3.suptitle(f"Capture point trajectories - {run_label}")
    add_position_plots_xy(ax3, d_xi, c_xi, "CP", run_label, xy_limits=xy_limits)
    ax3[0].legend(loc="upper right", fontsize=8)

    fig4, ax4 = plt.subplots(3, 1, figsize=(10, 8), sharex=True)
    fig4.suptitle(f"Tracking errors (desired-current) - {run_label}")
    add_error_plots(ax4, t_com, d_com, c_com, "COM", run_label)
    add_error_plots(ax4, t_zmp, d_zmp, c_zmp, "ZMP", run_label)
    ax4[0].legend(loc="upper right", fontsize=8)

    fig5, ax5 = plt.subplots(3, 1, figsize=(10, 8), sharex=True)
    fig5.suptitle(f"Capture point error (desired-current) - {run_label}")
    add_error_plots(ax5, t_xi, d_xi, c_xi, "CP", run_label)
    ax5[0].legend(loc="upper right", fontsize=8)

    plt.tight_layout()


def plot_comparison(log_paths, eta):
    runs = []
    for path in log_paths:
        data, dt = load_run(path)
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
        runs.append(
            {
                "label": Path(path).stem,
                "dt": dt,
                "t_com": np.arange(n_com) * dt,
                "t_zmp": np.arange(n_zmp) * dt,
                "t_xi": np.arange(n_xi) * dt,
                "com_err": d_com - c_com,
                "zmp_err": d_zmp - c_zmp,
                "xi_err": d_xi - c_xi,
            }
        )

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


def main():
    parser = argparse.ArgumentParser(description="Plot simulation logs exported by Logger.save_npz.")
    parser.add_argument("logs", nargs="+", help="One or more .npz log files.")
    parser.add_argument(
        "--compare",
        action="store_true",
        help="Compare tracking errors across multiple runs.",
    )
    parser.add_argument(
        "--xy-limits",
        nargs=4,
        type=float,
        metavar=("X_MIN", "X_MAX", "Y_MIN", "Y_MAX"),
        help="Manual limits for x-y trajectory plots.",
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

    if args.compare and len(args.logs) < 2:
        raise ValueError("--compare requires at least two log files.")

    xy_limits = tuple(args.xy_limits) if args.xy_limits is not None else None
    eta = args.eta if args.eta is not None else np.sqrt(args.gravity / args.lip_height)

    if args.compare:
        plot_comparison(args.logs, eta)
    else:
        for log in args.logs:
            plot_single_run(log, Path(log).stem, eta, xy_limits=xy_limits)

    # plt.show()
    #save figures to files instead of showing them interactively
    for i, fig in enumerate(plt.get_fignums()):
        plt.figure(fig)
        plt.savefig(f"logs/plot_{i}.png", dpi=300)
    print("Plots saved as logs/plot_*.png.")


if __name__ == "__main__":
    main()
