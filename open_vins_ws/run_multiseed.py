#!/usr/bin/env python3
"""Run OpenVINS (ros1_serial_msckf, monocular) several times per sequence.

    python3 run_multiseed.py m3ed  --runs 10 [--jobs 4] [--sequences ...]
    python3 run_multiseed.py kitti --runs 10
    python3 run_multiseed.py tartanground --runs 5

Run on the host; every run is a roslaunch inside the docker container (ov_docker.sh).
Writes results/multi_seed/<dataset>/<sequence>/run_XX/
    traj_estimate_tum.txt   IMU pose in global, TUM format (missing if there were no poses)
    run.log                 sequence, run, seed, status, poses, run time, fps
    traj_timing.txt         per-frame processing time of the estimator
    estimator.log           console output of the estimator
Finished runs (run.log exists) are skipped, so the script can be re-run to resume. Jobs go run
by run (run_00 of every sequence, then run_01, ...), --jobs of them at the same time.

TartanGround has no bags on disk (100 GB on the slow dataset HDD): its jobs go sequence by
sequence instead, tartanground_to_bag.py writes the bag of a sequence to tmp_bags/ (SSD) before
its first missing run, and the bag is deleted after its last one. run_00 is the single run of
run_tartanground.sh (results/tartanground), imported by import_tartanground_run00.py.

OpenVINS has no seed option: VioManager calls cv::setRNGSeed(0), so every run uses seed 0 and
runs only differ through multi-threading. Configs and launch arguments match the single-run
results in results/<dataset> (run_m3ed.sh / run_kitti.sh).
"""
import argparse
import queue
import subprocess
import threading
import time
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path

import numpy as np

WS = Path(__file__).resolve().parent
OUT = WS / "results" / "multi_seed"
DATA_HOST = Path("/media/efe/Data/datasets")  # mounted as /datasets in the container
SEED = 0  # cv::setRNGSeed(0) in VioManager
MIN_POSES = 10
# after the last message the node always aborts while unloading its libraries (exit code -6);
# a node that died without this message stopped somewhere inside the bag
NORMAL_EXIT = "class_loader::LibraryUnloadException"


def m3ed_config(seq):
    return "m3ed_" + "_".join(seq.split("_")[:3 if "urban" in seq else 2])  # m3ed_car_urban_day / m3ed_car_forest


TMP_BAGS = WS / "tmp_bags"  # mounted as /catkin_ws/tmp_bags in the container


def tartanground_sequences():
    out = subprocess.run(["python3", str(WS / "tartanground_to_bag.py"), "--list"], check=True, capture_output=True, text=True)
    return out.stdout.split()


DATASETS = {  # sequences, config folder, bag path inside the container
    "m3ed": (lambda: sorted(p.name for p in (DATA_HOST / "VIO/M3ED/vins").iterdir() if (p / "data.bag").exists()),
             m3ed_config, lambda seq: f"/datasets/VIO/M3ED/vins/{seq}/data.bag"),
    "kitti": (lambda: sorted(p.name for p in (DATA_HOST / "VIO/KITTI").glob("*_drive_*")),
              lambda seq: f"kitti_{seq.split('_drive_')[0]}_tuned", lambda seq: f"/datasets/VIO/KITTI/{seq}/vins/{seq}.bag"),
    "tartanground": (tartanground_sequences, lambda seq: "tartanground_diff", lambda seq: f"/catkin_ws/tmp_bags/{seq}.bag"),
}

log_lock = threading.Lock()


def log(msg):
    with log_lock:
        print(f"[{time.strftime('%F %T')}] {msg}", flush=True)


def run_one(dataset, seq, run, ports):
    _, config_of, bag_of = DATASETS[dataset]
    run_dir = OUT / dataset / seq / f"run_{run:02d}"
    run_dir.mkdir(parents=True, exist_ok=True)
    for name in ("traj_estimate.txt", "traj_timing.txt", "traj_estimate_tum.txt"):
        (run_dir / name).unlink(missing_ok=True)
    inside = f"/catkin_ws/{run_dir.relative_to(WS)}"
    port = ports.get()
    log(f"START {seq} run_{run:02d}")
    start = time.time()
    try:
        with open(run_dir / "estimator.log", "w") as fh:
            subprocess.run(
                [str(WS / "ov_docker.sh"), "roslaunch", "-p", str(port), "ov_msckf", "serial.launch",
                 f"config:={config_of(seq)}", f"dataset:={seq}", f"bag:={bag_of(seq)}", "bag_start:=0",
                 "max_cameras:=1", "use_stereo:=false", "dosave:=true", f"path_est:={inside}/traj_estimate.txt",
                 "dotime:=true", f"path_time:={inside}/traj_timing.txt"],
                stdout=fh, stderr=subprocess.STDOUT, stdin=subprocess.DEVNULL)
    finally:
        ports.put(port)
    dt = time.time() - start

    def load(name, **kw):
        f = run_dir / name
        return np.loadtxt(f, ndmin=2, **kw) if f.exists() and f.stat().st_size else np.zeros((0, 8))

    est = load("traj_estimate.txt")
    timing = load("traj_timing.txt", delimiter=",")
    console = (run_dir / "estimator.log").read_text(errors="replace")
    if "has died" in console and NORMAL_EXIT not in console:
        status = f"FAILED crashed ({len(est)} poses)"
    elif len(est) < MIN_POSES:
        status = f"FAILED no_init ({len(est)} poses)"
    else:
        status = "OK"
    if len(est) >= MIN_POSES:  # ov_eval writes pose + covariance columns, evo wants plain TUM
        np.savetxt(run_dir / "traj_estimate_tum.txt", est[:, :8], fmt="%.6f")
    (run_dir / "traj_estimate.txt").unlink(missing_ok=True)
    # per-frame processing time of the estimator (last column of traj_timing.txt), without bag reading
    proc = timing[:, -1] if len(timing) else np.array([np.nan])
    (run_dir / "run.log").write_text(
        f"sequence: {seq}\nrun: {run}\nseed: {SEED}\nstatus: {status}\nframes: {len(timing)}\nposes: {len(est)}\n"
        f"time_s: {dt:.1f}\nfps: {1 / proc.mean():.2f}\nframe_ms_mean: {1e3 * proc.mean():.2f}\n"
        f"frame_ms_max: {1e3 * proc.max():.2f}\nfinished: {time.strftime('%F %T')}\nconfig: {config_of(seq)}\n")
    log(f"END {seq} run_{run:02d} {status} poses={len(est)} time={dt:.0f}s")


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("dataset", choices=list(DATASETS))
    parser.add_argument("--runs", type=int, default=10)
    parser.add_argument("--sequences", nargs="+")
    parser.add_argument("--jobs", type=int, default=1, help="runs at the same time (more than 1 makes the fps meaningless)")
    parser.add_argument("--port", type=int, default=11450, help="first ROS master port, one per job")
    args = parser.parse_args()

    list_seqs = DATASETS[args.dataset][0]
    seqs = args.sequences or list_seqs()
    # run-major order: run_00 of every sequence, then run_01 of every sequence, ...
    jobs = [(seq, run) for run in range(args.runs) for seq in seqs]
    jobs = [j for j in jobs if not (OUT / args.dataset / j[0] / f"run_{j[1]:02d}" / "run.log").exists()]
    if args.dataset == "tartanground":  # sequence-major, one temporary bag per sequence
        if args.jobs != 1:
            parser.error("tartanground runs one job at a time")
        subprocess.run([str(WS / "ov_docker.sh"), "true"], check=True)
        log(f"{len(jobs)} runs to do, 1 at a time, sequence by sequence")
        ports = queue.Queue()
        ports.put(args.port)
        for seq in dict.fromkeys(s for s, _ in jobs):
            TMP_BAGS.mkdir(exist_ok=True)
            t0 = time.time()
            subprocess.run(["python3", str(WS / "tartanground_to_bag.py"), "--bag-dir", str(TMP_BAGS), seq], check=True)
            log(f"BAG {seq} {time.time() - t0:.0f}s")
            try:
                for s, run in jobs:
                    if s == seq:
                        run_one(args.dataset, seq, run, ports)
            finally:
                (TMP_BAGS / f"{seq}.bag").unlink(missing_ok=True)
        log("ALL DONE")
        raise SystemExit
    ports = queue.Queue()
    for p in range(args.port, args.port + args.jobs):
        ports.put(p)
    subprocess.run([str(WS / "ov_docker.sh"), "true"], check=True)  # container up before the parallel jobs
    log(f"{len(jobs)} runs to do, {args.jobs} at a time")
    with ThreadPoolExecutor(args.jobs) as pool:
        for f in [pool.submit(run_one, args.dataset, seq, run, ports) for seq, run in jobs]:
            f.result()
    log("ALL DONE")
