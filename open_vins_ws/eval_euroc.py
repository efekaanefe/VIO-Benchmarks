#!/usr/bin/env python3
"""Evaluate the OpenVINS EuRoC trajectories in results/euroc_mav with evo (ATE + RPE).

Run on the host (needs evo):  python3 eval_euroc.py
Writes per sequence: traj_estimate_tum.txt (8-column TUM file usable with the evo CLI)
and for all sequences: results/euroc_mav/evo_results.csv
"""
import copy
import csv
import glob
import os

import numpy as np
from evo.core import metrics, sync
from evo.core.metrics import PoseRelation, Unit
from evo.tools import file_interface

WS = os.path.dirname(os.path.abspath(__file__))
RES = os.path.join(WS, "results", "euroc_mav")
GT = os.path.join(WS, "src", "open_vins", "ov_data", "euroc_mav")
MAX_DIFF = 0.01  # s, timestamp association tolerance
RPE_DELTAS_M = [1.0, 5.0, 10.0]  # distance travelled between pose pairs
STATS = ["rmse", "mean", "median", "std", "min", "max"]


def stats(metric, data):
    metric.process_data(data)
    s = metric.get_all_statistics()
    return {k: s[k] for k in STATS}, len(metric.error)


rows = []
for est_file in sorted(glob.glob(os.path.join(RES, "*", "traj_estimate.txt"))):
    seq = os.path.basename(os.path.dirname(est_file))
    # ov_eval writes pose + covariance columns, evo wants plain TUM
    tum_file = est_file.replace(".txt", "_tum.txt")
    np.savetxt(tum_file, np.loadtxt(est_file)[:, :8], fmt="%.6f")

    traj_ref = file_interface.read_tum_trajectory_file(os.path.join(GT, seq + ".txt"))
    traj_est = file_interface.read_tum_trajectory_file(tum_file)
    n_est = traj_est.num_poses
    traj_ref, traj_est = sync.associate_trajectories(traj_ref, traj_est, max_diff=MAX_DIFF)
    info = {
        "sequence": seq,
        "est_poses": n_est,
        "matched": traj_est.num_poses,
        "duration_s": traj_est.timestamps[-1] - traj_est.timestamps[0],
        "path_m": traj_ref.path_length,
    }

    # ATE: SE(3) Umeyama alignment, no scale correction (stereo VIO has metric scale)
    aligned = copy.deepcopy(traj_est)
    aligned.align(traj_ref, correct_scale=False)
    for name, rel, unit in [
        ("ATE trans [m]", PoseRelation.translation_part, None),
        ("ATE rot [deg]", PoseRelation.rotation_angle_deg, None),
    ]:
        s, n = stats(metrics.APE(rel), (traj_ref, aligned))
        rows.append({**info, "metric": name, "pairs": n, **s})

    # RPE: drift over fixed travelled distances, all pose pairs
    for d in RPE_DELTAS_M:
        for name, rel in [
            (f"RPE trans @{d:g}m [m]", PoseRelation.translation_part),
            (f"RPE rot @{d:g}m [deg]", PoseRelation.rotation_angle_deg),
        ]:
            m = metrics.RPE(rel, delta=d, delta_unit=Unit.meters, all_pairs=True)
            s, n = stats(m, (traj_ref, traj_est))
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
        print(f"\n{r['sequence']}: matched {r['matched']}/{r['est_poses']} poses, "
              f"{r['duration_s']:.1f} s, {r['path_m']:.2f} m")
        print(f"  {'metric':24s} {'pairs':>6s} " + " ".join(f"{k:>8s}" for k in STATS))
    print(f"  {r['metric']:24s} {r['pairs']:6d} " + " ".join(f"{r[k]:8.4f}" for k in STATS))
print("\nsaved", out)
