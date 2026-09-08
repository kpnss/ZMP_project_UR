"""Record MP4s of chosen runs_summary configurations.

dartpy's own `Viewer.record()` cannot be used: the wheel bundles the OSG core
libraries but none of the `osgdb_*` image plugins, so OSG has no way to write a
PNG. Instead each run is rendered onto a private Xvfb display and captured with
ffmpeg's x11grab -- nothing appears on, or is recorded from, the real desktop.

Because x11grab samples on wall-clock time, the simulation is paced to real time
(stride * fps * world_dt == 1), so the video plays at 1x. Stepping is driven
manually (`viewer.simulate(False)` + `viewer.frame()`) rather than via
`viewer.run()`, so the run ends with the footstep plan instead of waiting for a
window to close.

Requires: Xvfb (`sudo dnf install xorg-x11-server-Xvfb`) and ffmpeg.

Usage:
  python record_videos.py                        # the default set of variants
  python record_videos.py plain CP-controller    # by slug (see logs/zmp_plots/)
  python record_videos.py --benchmark            # check llvmpipe keeps up
"""
import argparse
import os
import shutil
import subprocess
import sys
import time

OUT_DIR = os.path.join("logs", "videos")
DEFAULT_VARIANTS = ["ISMPC (no cp)", "plain", "CP controller"]

WIDTH, HEIGHT = 1280, 720
FPS = 25
STRIDE = 4        # world steps per frame; STRIDE * FPS * 0.01 == 1 -> real time
DISPLAY_NUM = 99


# ---- variant lookup (imports dartpy, so keep it out of the orchestrator) ----

def _configs():
    from make_runs_summary import CONFIGS, config_label
    from plot_logs import variant_slug
    return [(cfg, config_label(cfg), variant_slug(config_label(cfg))) for cfg in CONFIGS]


def find_config(name):
    for cfg, label, slug in _configs():
        if name in (label, slug):
            return cfg, label, slug
    known = ", ".join(slug for _, _, slug in _configs())
    sys.exit(f"Unknown variant '{name}'. Known slugs: {known}")


# ---- worker: renders one run on the current DISPLAY -------------------------

def run_sim(slug, fps, stride, max_steps=None, pace=True, wait_start=False):
    import dartpy as dart
    from ablation_poles import build_world
    from simulation import Hrp4Controller

    cfg, label, _ = find_config(slug)
    world, hrp4 = build_world()
    node = Hrp4Controller(world, hrp4, log_path=None, autosave_every=0,
                          use_kf=True, use_mpc=cfg["mpc"], use_cp=cfg["cp fb"],
                          open_loop=cfg["openloop ref"], use_lag=cfg["lag sys"],
                          use_zmp_fb=cfg["zmp fb"])
    n_steps = sum(s['ss_duration'] + s['ds_duration'] for s in node.footstep_planner.plan)
    if max_steps is not None:
        n_steps = min(n_steps, max_steps)

    viewer = dart.gui.osg.Viewer()
    viewer.addWorldNode(node)
    viewer.simulate(False)          # the loop below owns the stepping
    viewer.setUpViewInWindow(0, 0, WIDTH, HEIGHT)
    # Fixed 3/4 view centred on the mid-point of the ~2.2 m walk. Close enough
    # that the robot fills most of the frame, far enough that it never leaves it,
    # and off-axis so the lateral (y) sway stays visible.
    viewer.setCameraHomePosition([3.0, -3.6, 1.5], [1.1, 0.0, 0.75], [0.0, 0.0, 1.0])

    frame_dt = stride / (stride * fps)   # == 1/fps
    for _ in range(5):                   # map the window and draw the first frame
        viewer.frame()
        time.sleep(0.05)

    if wait_start:
        # the parent starts the capture only now, so the several seconds spent
        # importing dartpy and parsing the URDF never reach the video
        print("READY", flush=True)
        sys.stdin.readline()

    t0 = time.perf_counter()
    n_frames = 0
    for i in range(n_steps):
        try:
            node.customPreStep()
            world.step()
        except Exception:
            print(f"[{slug}] DIVERGED at step {i}", flush=True)
            break
        if i % stride == 0:
            viewer.frame()
            n_frames += 1
            if pace:
                slack = t0 + n_frames * frame_dt - time.perf_counter()
                if slack > 0:
                    time.sleep(slack)
    elapsed = time.perf_counter() - t0
    print(f"[{slug}] {n_frames} frames in {elapsed:.1f}s "
          f"({n_frames / elapsed:.1f} fps rendered, target {fps})", flush=True)


# ---- orchestrator ----------------------------------------------------------

def start_xvfb(display):
    if not shutil.which("Xvfb"):
        sys.exit("Xvfb not found. Install it: sudo dnf install -y xorg-x11-server-Xvfb")
    proc = subprocess.Popen(
        ["Xvfb", display, "-screen", "0", f"{WIDTH}x{HEIGHT}x24", "-nolisten", "tcp"],
        stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    sock = f"/tmp/.X11-unix/X{display.lstrip(':')}"
    for _ in range(100):
        if os.path.exists(sock):
            time.sleep(0.3)
            return proc
        if proc.poll() is not None:
            sys.exit(f"Xvfb exited immediately (is {display} already in use?)")
        time.sleep(0.1)
    proc.terminate()
    sys.exit(f"Xvfb did not come up on {display}")


def start_ffmpeg(display, out_path, fps):
    return subprocess.Popen(
        ["ffmpeg", "-y", "-loglevel", "error",
         "-f", "x11grab", "-framerate", str(fps), "-draw_mouse", "0",
         "-video_size", f"{WIDTH}x{HEIGHT}", "-i", display,
         "-c:v", "libx264", "-preset", "medium", "-crf", "20",
         "-pix_fmt", "yuv420p", out_path],
        stdin=subprocess.PIPE, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)


def stop_ffmpeg(proc):
    try:
        proc.communicate(b"q", timeout=30)
    except subprocess.TimeoutExpired:
        proc.terminate()
        proc.wait(timeout=10)


def main():
    parser = argparse.ArgumentParser(description="Record MP4s of runs_summary configs.")
    parser.add_argument("variants", nargs="*", help=f"Labels or slugs (default: {DEFAULT_VARIANTS}).")
    parser.add_argument("--fps", type=int, default=FPS)
    parser.add_argument("--stride", type=int, default=STRIDE)
    parser.add_argument("--max-steps", type=int, default=None, help="Cap run length (quick check).")
    parser.add_argument("--display", default=f":{DISPLAY_NUM}")
    parser.add_argument("--benchmark", action="store_true",
                        help="Render 300 unpaced steps and report the achievable fps.")
    parser.add_argument("--worker", metavar="SLUG", help=argparse.SUPPRESS)
    parser.add_argument("--wait-start", action="store_true", help=argparse.SUPPRESS)
    args = parser.parse_args()

    if args.worker:                     # re-entry inside the Xvfb environment
        run_sim(args.worker, args.fps, args.stride, args.max_steps,
                pace=not args.benchmark, wait_start=args.wait_start)
        return

    if args.stride * args.fps * 0.01 != 1.0:
        print(f"warning: stride*fps*dt = {args.stride * args.fps * 0.01:g}; "
              f"video will not play at 1x", flush=True)

    xvfb = start_xvfb(args.display)
    env = dict(os.environ, DISPLAY=args.display)
    os.makedirs(OUT_DIR, exist_ok=True)
    try:
        for name in (args.variants or DEFAULT_VARIANTS):
            _, label, slug = find_config(name)
            cmd = [sys.executable, __file__, "--worker", slug,
                   "--fps", str(args.fps), "--stride", str(args.stride)]
            if args.max_steps is not None:
                cmd += ["--max-steps", str(args.max_steps)]
            if args.benchmark:
                cmd += ["--benchmark"]
                print(f"benchmarking {slug} ...", flush=True)
                subprocess.run(cmd, env=env, check=True)
                continue

            out_path = os.path.join(OUT_DIR, f"{slug}.mp4")
            print(f"recording {label} -> {out_path}", flush=True)
            worker = subprocess.Popen(cmd + ["--wait-start"], env=env, text=True,
                                      stdin=subprocess.PIPE, stdout=subprocess.PIPE)
            grab = None
            try:
                for line in worker.stdout:
                    if line.strip() == "READY":
                        grab = start_ffmpeg(args.display, out_path, args.fps)
                        time.sleep(0.5)      # let the grab latch on
                        worker.stdin.write("\n")
                        worker.stdin.flush()
                        continue
                    print(f"  {line.rstrip()}", flush=True)
                worker.wait(timeout=60)
            finally:
                if grab is not None:
                    stop_ffmpeg(grab)
                if worker.poll() is None:
                    worker.terminate()
            if worker.returncode:
                sys.exit(f"[{slug}] worker failed with code {worker.returncode}")
            size = os.path.getsize(out_path) / 1e6 if os.path.exists(out_path) else 0
            print(f"  wrote {out_path} ({size:.1f} MB)", flush=True)
    finally:
        xvfb.terminate()
        xvfb.wait(timeout=10)


if __name__ == "__main__":
    main()
