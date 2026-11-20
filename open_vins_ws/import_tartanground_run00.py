#!/usr/bin/env python3
"""Import the single TartanGround runs (run_tartanground.sh, results/tartanground/<sequence>) as
run_00 of the repeated runs in results/multi_seed/tartanground, in the layout of run_multiseed.py:

    run_00/traj_estimate_tum.txt, traj_timing.txt, estimator.log (= the console log, run.log there)
    run_00/run.log   status, poses, run time, fps etc., the same keys and rules as run_multiseed.py

Same config (tartanground_diff, unchanged since), same launch arguments and one run at a time, so
these are equivalent to a run of run_multiseed.py. The run time comes from logs/runs.log.

    python3 import_tartanground_run00.py
"""
import re
import shutil

import numpy as np

from run_multiseed import MIN_POSES, NORMAL_EXIT, OUT, SEED, WS

SRC = WS / "results" / "tartanground"
done = {m[1]: (m[0], float(m[2])) for m in
        re.findall(r"\[(\S+ \S+)\] \[(\S+)\] exit=\S+ bag \d+s, run (\d+)s", (SRC / "logs" / "runs.log").read_text())}

for src in sorted(p for p in SRC.iterdir() if (p / "run.log").exists()):
    seq = src.name
    dst = OUT / "tartanground" / seq / "run_00"
    if (dst / "run.log").exists():
        continue
    dst.mkdir(parents=True, exist_ok=True)
    shutil.copy(src / "run.log", dst / "estimator.log")
    shutil.copy(src / "traj_timing.txt", dst / "traj_timing.txt")
    est = np.loadtxt(src / "traj_estimate.txt", ndmin=2) if (src / "traj_estimate.txt").stat().st_size else np.zeros((0, 8))
    timing = np.loadtxt(src / "traj_timing.txt", delimiter=",", ndmin=2)
    console = (src / "run.log").read_text(errors="replace")
    if "has died" in console and NORMAL_EXIT not in console:
        status = f"FAILED crashed ({len(est)} poses)"
    elif len(est) < MIN_POSES:
        status = f"FAILED no_init ({len(est)} poses)"
    else:
        status = "OK"
    if len(est) >= MIN_POSES:
        np.savetxt(dst / "traj_estimate_tum.txt", est[:, :8], fmt="%.6f")
    finished, time_s = done[seq]
    proc = timing[:, -1]
    (dst / "run.log").write_text(
        f"sequence: {seq}\nrun: 0\nseed: {SEED}\nstatus: {status}\nframes: {len(timing)}\nposes: {len(est)}\n"
        f"time_s: {time_s:.1f}\nfps: {1 / proc.mean():.2f}\nframe_ms_mean: {1e3 * proc.mean():.2f}\n"
        f"frame_ms_max: {1e3 * proc.max():.2f}\nfinished: {finished}\nconfig: tartanground_diff\n"
        f"imported_from: results/tartanground/{seq} (run_tartanground.sh)\n")
    print(seq, status)
