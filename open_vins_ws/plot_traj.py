#!/usr/bin/env python3
"""Top-view plot of ground truth vs OpenVINS estimate for every KITTI, M3ED and TartanGround sequence, SE(3)
aligned on the left and Sim(3) aligned on the right (6000x3000 px).

    python3 plot_traj.py [kitti] [m3ed] [tartanground]

Writes
    results/kitti/<sequence>/<odometry nr>_<sequence>_traj.png        (single run)
    results/multi_seed/m3ed/<sequence>/<sequence>_traj.png            (run_00 of the 10 runs)
    results/tartanground/<sequence>/<sequence>_traj.png               (single run)
"""
import copy
import sys
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from evo.tools import file_interface
from matplotlib.lines import Line2D

from eval_multiseed import DIVERGED_ATE_PCT, DIVERGED_SCALE, MAX_GT_GAP, RES, associate

C_GT, C_EST = "#2a78d6", "#eb6834"
INK, INK2, GRID, SURFACE = "#0b0b0b", "#52514e", "#e4e3df", "#fcfcfb"
KITTI_DATA = Path("/media/efe/Data/datasets/VIO/KITTI")
TARTAN_DATA = Path("/media/efe/Data/datasets/VIO/TartanGround/vins")
KITTI_ODOMETRY = {  # raw drive -> KITTI odometry sequence
    "2011_10_03_drive_0027": "00", "2011_10_03_drive_0042": "01", "2011_10_03_drive_0034": "02",
    "2011_09_30_drive_0016": "04", "2011_09_30_drive_0018": "05", "2011_09_30_drive_0020": "06",
    "2011_09_30_drive_0027": "07", "2011_09_30_drive_0028": "08", "2011_09_30_drive_0033": "09",
    "2011_09_30_drive_0034": "10",
}


def kitti_plane(xyz):
    """OXTS ground truth: x east, y north."""
    return xyz[:, 0], xyz[:, 1], "east [m]", "north [m]"


def m3ed_plane(xyz):
    """Ground truth frame is the first IMU frame (x left, y down, z backward)."""
    return -xyz[:, 0], -xyz[:, 2], "right [m]", "forward [m]"


def ned_plane(xyz):
    """TartanGround world frame is NED: x north, y east."""
    return xyz[:, 1], xyz[:, 0], "east [m]", "north [m]"


def jobs(dataset):
    if dataset == "kitti":
        for d in sorted((RES / "kitti").glob("*_drive_*")):
            nr = KITTI_ODOMETRY.get(d.name, "")
            gt = d / "groundtruth_tum.txt"
            if not gt.exists():  # eval_kitti.py only copies it for drives with an estimate
                gt = KITTI_DATA / d.name / "vins" / "gt_tum.txt"
            yield (f"KITTI odometry {nr}  ·  {d.name}", "OpenVINS mono [tuned config, single run]", "groundtruth (OXTS)",
                   gt, d / "traj_estimate_tum.txt", d / f"{nr}_{d.name}_traj.png", kitti_plane)
    elif dataset == "tartanground":
        for d in sorted(p for p in (RES / "tartanground").iterdir() if (p / "run.log").exists()):
            gt = d / "groundtruth_tum.txt"
            if not gt.exists():  # eval_tartanground.py only copies it for sequences with an estimate
                gt = TARTAN_DATA / d.name / "gt_tum.txt"
            yield (f"TartanGround  ·  {d.name}", "OpenVINS mono [single run]", "groundtruth",
                   gt, d / "traj_estimate_tum.txt", d / f"{d.name}_traj.png", ned_plane)
    else:
        for d in sorted((RES / "multi_seed" / "m3ed").glob("car_*")):
            yield (f"M3ED  ·  {d.name}", "OpenVINS mono [run_00 of 10]", "groundtruth",
                   d / "groundtruth_tum.txt", d / "run_00" / "traj_estimate_tum.txt", d / f"{d.name}_traj.png", m3ed_plane)


def plot(dataset, title, method, gt_label, gt_file, est_file, out, plane):
    gt = file_interface.read_tum_trajectory_file(gt_file)
    gt_dur = gt.timestamps[-1] - gt.timestamps[0]
    est = None
    if est_file.exists() and np.loadtxt(est_file, ndmin=2).shape[0] >= 10:
        est = file_interface.read_tum_trajectory_file(est_file)
        ref, est = associate(gt, est, MAX_GT_GAP[dataset])
        coverage = 100 * (est.timestamps[-1] - est.timestamps[0]) / gt_dur
        panels = []
        for correct_scale in (False, True):
            aligned = copy.deepcopy(est)
            _, _, scale = aligned.align(ref, correct_scale=correct_scale)
            rmse = np.sqrt(np.mean(np.sum((aligned.positions_xyz - ref.positions_xyz) ** 2, axis=1)))
            panels.append((aligned, rmse, scale))
        sim3_rmse, scale = panels[1][1], panels[1][2]
        diverged = 100 * sim3_rmse / gt.path_length > DIVERGED_ATE_PCT or not DIVERGED_SCALE[0] <= scale <= DIVERGED_SCALE[1]
        status = f"estimate covers {coverage:.0f}% of the drive  ·  " + ("diverged" if diverged else "completed the drive")
    else:
        status = "no estimate (never initialized)"

    fig, axes = plt.subplots(1, 2, figsize=(20, 10), facecolor=SURFACE)
    gx, gy, xlabel, ylabel = plane(gt.positions_xyz)
    names = [("SE(3) aligned (rigid, metric scale kept)", False), ("Sim(3) aligned (rigid + scale)", True)]
    for i, (ax, (name, correct_scale)) in enumerate(zip(axes, names)):
        ax.plot(gx, gy, color=C_GT, lw=2.0, solid_capstyle="round", zorder=3)
        ax.plot(gx[0], gy[0], "o", ms=10, mfc="white", mec=INK, mew=1.8, zorder=6)
        if est is not None:
            aligned, rmse, s = panels[i]
            ex, ey, _, _ = plane(aligned.positions_xyz)
            ax.plot(ex, ey, color=C_EST, lw=2.0, solid_capstyle="round", zorder=4)
            ax.plot(ex[-1], ey[-1], "x", ms=9, mew=2.5, color=C_EST, zorder=6)
            sub = f"ATE RMSE {rmse:.2f} m" + (f"   scale {s:.3f}" if correct_scale else "")
        else:
            sub = "no estimate"
        ax.set_title(f"{name}\n{sub}", loc="left", fontsize=13, color=INK)
        ax.set_aspect("equal", adjustable="datalim")
        ax.set_facecolor(SURFACE)
        ax.grid(True, color=GRID, lw=0.8)
        ax.set_axisbelow(True)
        for sp in ax.spines.values():
            sp.set_visible(False)
        ax.tick_params(colors=INK2, length=0, labelsize=10)
        ax.set_xlabel(xlabel, color=INK2)
        ax.set_ylabel(ylabel, color=INK2)

    fig.suptitle(f"{title}  ·  {method}  ·  GT {gt.path_length:.0f} m, {status}",
                 x=0.01, ha="left", y=0.985, fontsize=16, color=INK)
    handles = [Line2D([], [], color=C_GT, lw=2, label=gt_label),
               Line2D([], [], color=C_EST, lw=2, label="OpenVINS estimate"),
               Line2D([], [], ls="", marker="o", ms=10, mfc="white", mec=INK, mew=1.8, label="groundtruth start"),
               Line2D([], [], ls="", marker="x", ms=9, mew=2.5, color=C_EST, label="estimate end")]
    fig.legend(handles=handles, loc="lower center", ncol=4, frameon=False, fontsize=12, labelcolor=INK,
               bbox_to_anchor=(0.5, 0.005))
    fig.tight_layout(rect=(0, 0.05, 1, 0.95), w_pad=6)
    fig.savefig(out, dpi=300, facecolor=SURFACE)
    plt.close(fig)
    print("saved", out)


if __name__ == "__main__":
    plt.rcParams.update({"font.family": "DejaVu Sans"})
    for dataset in sys.argv[1:] or ["kitti", "m3ed", "tartanground"]:
        for job in jobs(dataset):
            plot(dataset, *job)
