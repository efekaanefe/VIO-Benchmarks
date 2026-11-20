#!/usr/bin/env python3
"""Plot ground truth vs estimate for every M3ED sequence, once with SE(3) and once with
Sim(3) alignment. Run eval_m3ed.py first (it writes the TUM files this reads).

Writes results/m3ed/trajectories_se3.png and results/m3ed/trajectories_sim3.png
"""
import copy
import glob
import os

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from evo.core.trajectory import PoseTrajectory3D
from evo.tools import file_interface
from scipy.spatial.transform import Rotation, Slerp

WS = os.path.dirname(os.path.abspath(__file__))
RES = os.path.join(WS, "results", "m3ed")
MAX_GT_GAP = 0.15  # s, same association as eval_m3ed.py
C_GT, C_EST = "#2a78d6", "#eb6834"
INK, INK2, GRID, SURFACE = "#0b0b0b", "#52514e", "#e4e3df", "#fcfcfb"


def associate(traj_ref, traj_est):
    """Interpolate the ground truth at the estimate timestamps."""
    t_ref, t_est = traj_ref.timestamps, traj_est.timestamps
    i = np.searchsorted(t_ref, t_est)
    ok = (i > 0) & (i < len(t_ref))
    ok[ok] &= (t_ref[i[ok]] - t_ref[i[ok] - 1]) < MAX_GT_GAP
    t = t_est[ok]
    xyz = np.column_stack([np.interp(t, t_ref, traj_ref.positions_xyz[:, k]) for k in range(3)])
    quat = Slerp(t_ref, Rotation.from_quat(traj_ref.orientations_quat_wxyz[:, [1, 2, 3, 0]]))(t).as_quat()
    ref = PoseTrajectory3D(xyz, quat[:, [3, 0, 1, 2]], t)
    est = PoseTrajectory3D(traj_est.positions_xyz[ok], traj_est.orientations_quat_wxyz[ok], t)
    return ref, est


def ground_plane(xyz):
    """The ground truth frame is the first IMU frame (x left, y down, z backward):
    return (right, forward) so the plot is a top view with the car starting upwards."""
    return -xyz[:, 0], -xyz[:, 2]


seqs = []
for est_file in sorted(glob.glob(os.path.join(RES, "*", "traj_estimate_tum.txt"))):
    seq = os.path.basename(os.path.dirname(est_file))
    gt_full = file_interface.read_tum_trajectory_file(os.path.join(RES, seq, "groundtruth_tum.txt"))
    ref, est = associate(gt_full, file_interface.read_tum_trajectory_file(est_file))
    seqs.append((seq, gt_full, ref, est))

plt.rcParams.update({"font.family": "DejaVu Sans", "font.size": 11})
for name, correct_scale, label in [("se3", False, "SE(3) alignment (rotation + translation)"),
                                   ("sim3", True, "Sim(3) alignment (rotation + translation + scale)")]:
    cols = 4
    rows = int(np.ceil(len(seqs) / cols))
    fig, axes = plt.subplots(rows, cols, figsize=(6.2 * cols, 6.2 * rows), facecolor=SURFACE)
    for ax in axes.ravel()[len(seqs):]:
        ax.axis("off")
    for ax, (seq, gt_full, ref, est) in zip(axes.ravel(), seqs):
        aligned = copy.deepcopy(est)
        _, _, scale = aligned.align(ref, correct_scale=correct_scale)
        err = np.linalg.norm(aligned.positions_xyz - ref.positions_xyz, axis=1)
        rmse = np.sqrt(np.mean(err**2))

        gx, gy = ground_plane(gt_full.positions_xyz)
        ex, ey = ground_plane(aligned.positions_xyz)
        ax.plot(gx, gy, color=C_GT, lw=2.0, label="Ground truth", solid_capstyle="round")
        ax.plot(ex, ey, color=C_EST, lw=1.6, ls=(0, (5, 2)), label="OpenVINS estimate")
        ax.plot(gx[0], gy[0], "o", ms=8, color=INK, mec=SURFACE, mew=2, label="Start", zorder=5)

        ax.set_aspect("equal", adjustable="datalim")
        ax.set_facecolor(SURFACE)
        ax.grid(True, color=GRID, lw=0.8)
        ax.set_axisbelow(True)
        for s in ax.spines.values():
            s.set_color(GRID)
        ax.tick_params(colors=INK2, length=0, labelsize=10)
        ax.set_xlabel("right [m]", color=INK2)
        ax.set_ylabel("forward [m]", color=INK2)
        sub = f"ATE RMSE {rmse:.2f} m  ·  max {err.max():.1f} m  ·  path {gt_full.path_length:.0f} m"
        if correct_scale:
            sub += f"  ·  scale {scale:.3f}"
        ax.set_title(seq, color=INK, fontsize=13, fontweight="bold", loc="left", pad=24)
        ax.text(0, 1.02, sub, transform=ax.transAxes, color=INK2, fontsize=10.5, va="bottom")

    handles, labels = axes.ravel()[0].get_legend_handles_labels()
    fig.suptitle(f"M3ED car sequences, OpenVINS monocular VIO vs ground truth: {label}",
                 color=INK, fontsize=20, fontweight="bold", x=0.02, ha="left", y=0.995)
    fig.legend(handles, labels, loc="upper right", bbox_to_anchor=(0.985, 0.999), ncol=3,
               frameon=False, fontsize=13, labelcolor=INK2)
    fig.text(0.02, 0.978, "Top view in the frame of the first ground truth pose. Estimate aligned to ground truth over all matched poses.",
             color=INK2, fontsize=12.5, ha="left", va="top")
    fig.tight_layout(rect=(0, 0, 1, 0.972), h_pad=2.5, w_pad=1.5)
    out = os.path.join(RES, f"trajectories_{name}.png")
    fig.savefig(out, dpi=200, facecolor=SURFACE)
    plt.close(fig)
    print("saved", out)
