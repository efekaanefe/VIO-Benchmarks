#!/usr/bin/env python3
"""Evaluate the OpenVINS TartanGround trajectories in results/tartanground with evo (ATE + RPE).

Run on the host (needs evo):  python3 eval_tartanground.py
Writes per sequence: traj_estimate_tum.txt and groundtruth_tum.txt (8-column TUM files
usable with the evo CLI) and for all sequences: results/tartanground/evo_results.csv
RPE pose pairs are picked by distance travelled along the ground truth (pairs_from_reference),
as in eval_multiseed.py / make_table.py.
"""
import copy
import csv
import glob
import os
import shutil

import numpy as np
from evo.core import metrics
from evo.core.trajectory import PoseTrajectory3D
from evo.core.metrics import PoseRelation, Unit
from evo.tools import file_interface
from scipy.spatial.transform import Rotation, Slerp

WS = os.path.dirname(os.path.abspath(__file__))
RES = os.path.join(WS, "results", "tartanground")
DATA = os.environ.get("BAG_HOST", "/media/efe/Data/datasets/VIO/TartanGround/vins")
MAX_GT_GAP = 0.15  # s, ground truth is the 10 hz camera pose, estimates between poses further apart are dropped
RPE_DELTAS_M = [10.0, 50.0, 100.0]  # distance travelled between pose pairs
STATS = ["rmse", "mean", "median", "std", "min", "max"]


def write_groundtruth(seq, out):
    """gt_tum.txt holds the simulator pose of lcam_front (= IMU) at every camera timestamp (10 hz)."""
    shutil.copyfile(os.path.join(DATA, seq, "gt_tum.txt"), out)


def associate(traj_ref, traj_est):
    """Interpolate the ground truth at the estimate timestamps (they coincide up to rounding)."""
    t_ref, t_est = traj_ref.timestamps, traj_est.timestamps
    i = np.searchsorted(t_ref, t_est)  # t_ref[i-1] <= t_est < t_ref[i]
    ok = (i > 0) & (i < len(t_ref))
    ok[ok] &= (t_ref[i[ok]] - t_ref[i[ok] - 1]) < MAX_GT_GAP
    t = t_est[ok]
    xyz = np.column_stack([np.interp(t, t_ref, traj_ref.positions_xyz[:, k]) for k in range(3)])
    quat = Slerp(t_ref, Rotation.from_quat(traj_ref.orientations_quat_wxyz[:, [1, 2, 3, 0]]))(t).as_quat()
    ref = PoseTrajectory3D(xyz, quat[:, [3, 0, 1, 2]], t)
    est = PoseTrajectory3D(traj_est.positions_xyz[ok], traj_est.orientations_quat_wxyz[ok], t)
    return ref, est


def stats(metric, data):
    metric.process_data(data)
    s = metric.get_all_statistics()
    return {k: s[k] for k in STATS}, len(metric.error)


rows = []
for est_file in sorted(glob.glob(os.path.join(RES, "*", "traj_estimate.txt"))):
    seq = os.path.basename(os.path.dirname(est_file))
    est = np.loadtxt(est_file, ndmin=2)
    if len(est) < 10:
        print(f"{seq}: only {len(est)} poses, skipped")
        continue
    # ov_eval writes pose + covariance columns, evo wants plain TUM
    tum_file = est_file.replace(".txt", "_tum.txt")
    np.savetxt(tum_file, est[:, :8], fmt="%.6f")
    gt_file = os.path.join(RES, seq, "groundtruth_tum.txt")
    write_groundtruth(seq, gt_file)

    traj_ref = file_interface.read_tum_trajectory_file(gt_file)
    traj_est = file_interface.read_tum_trajectory_file(tum_file)
    gt_dur = traj_ref.timestamps[-1] - traj_ref.timestamps[0]
    gt_path = traj_ref.path_length
    est_t0 = traj_est.timestamps[0] - traj_ref.timestamps[0]
    traj_ref, traj_est = associate(traj_ref, traj_est)

    # ATE: SE(3) Umeyama alignment without scale, the IMU makes monocular VIO metric
    aligned = copy.deepcopy(traj_est)
    aligned.align(traj_ref, correct_scale=False)
    # the scale a Sim(3) alignment would apply tells how far off the estimated scale is
    scaled = copy.deepcopy(traj_est)
    _, _, scale = scaled.align(traj_ref, correct_scale=True)
    info = {
        "sequence": seq,
        "gt_duration_s": gt_dur,
        "gt_path_m": gt_path,
        "est_start_s": est_t0,
        "matched": traj_est.num_poses,
        "matched_path_m": traj_ref.path_length,
        "est_path_m": traj_est.path_length,
        "sim3_scale": scale,
    }
    for name, rel, est_traj in [
        ("ATE trans [m]", PoseRelation.translation_part, aligned),
        ("ATE rot [deg]", PoseRelation.rotation_angle_deg, aligned),
        ("ATE trans Sim3 [m]", PoseRelation.translation_part, scaled),
    ]:
        s, n = stats(metrics.APE(rel), (traj_ref, est_traj))
        rows.append({**info, "metric": name, "pairs": n, **s})

    # RPE: drift over fixed travelled distances, all pose pairs
    for d in RPE_DELTAS_M:
        for name, rel in [
            (f"RPE trans @{d:g}m [m]", PoseRelation.translation_part),
            (f"RPE rot @{d:g}m [deg]", PoseRelation.rotation_angle_deg),
        ]:
            m = metrics.RPE(rel, delta=d, delta_unit=Unit.meters, all_pairs=True, pairs_from_reference=True)
            try:
                s, n = stats(m, (traj_ref, traj_est))
            except Exception as e:  # sequence shorter than the delta
                print(f"{seq}: {name} failed: {e}")
                continue
            rows.append({**info, "metric": name, "pairs": n, **s})

out = os.path.join(RES, "evo_results.csv")
with open(out, "w", newline="") as f:
    w = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
    w.writeheader()
    w.writerows(rows)

seen = set()
for r in rows:
    if r["sequence"] not in seen:
        seen.add(r["sequence"])
        print(f"\n{r['sequence']}: gt {r['gt_duration_s']:.1f} s / {r['gt_path_m']:.0f} m, "
              f"estimate starts at {r['est_start_s']:.1f} s, {r['matched']} matched poses, "
              f"path gt {r['matched_path_m']:.0f} m vs est {r['est_path_m']:.0f} m, "
              f"Sim3 scale {r['sim3_scale']:.3f}")
        print(f"  {'metric':24s} {'pairs':>6s} " + " ".join(f"{k:>9s}" for k in STATS))
    print(f"  {r['metric']:24s} {r['pairs']:6d} " + " ".join(f"{r[k]:9.3f}" for k in STATS))
print("\nsaved", out)
