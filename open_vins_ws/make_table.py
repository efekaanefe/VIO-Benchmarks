#!/usr/bin/env python3
"""Write the summary LaTeX table results/table.tex (one row per dataset).

    python3 make_table.py

Runs per dataset:
    M3ED          results/multi_seed/m3ed/<sequence>/run_XX           (run_multiseed.py, one run at a time)
    TartanGround  results/multi_seed/tartanground/<sequence>/run_XX   (run_multiseed.py, one run at a time)
    KITTI  results/kitti/<sequence>                     (run_kitti.sh, single run, 3 in parallel)

Same evaluation as eval_multiseed.py. Per run: ATE RMSE in % of the ground truth path after SE(3)
and after Sim(3) alignment; RPE translation RMSE over RPE_DELTA_M of ground truth path in % of
the segment, on the estimate as is (SE(3), rigid alignment does not change RPE) and rescaled by
the Sim(3) scale; fps = 1 / mean per-frame processing time of the estimator. A run failed if it
has no trajectory or diverged (eval_multiseed.DIVERGED_*). Each column is the mean over the
sequences of the per-sequence mean over the runs that did not fail.
"""
import copy
from multiprocessing import Pool
from pathlib import Path

import numpy as np
import pandas as pd
from evo.core import metrics
from evo.core.metrics import PoseRelation, Unit
from evo.tools import file_interface

from eval_multiseed import DIVERGED_ATE_PCT, DIVERGED_SCALE, MAX_GT_GAP, RES, associate, run_info

RPE_DELTA_M = 100.0


def runs(dataset):
    """(sequence, run, estimate file, ground truth file, fps) for every run of a dataset."""
    if dataset in ("m3ed", "tartanground"):
        for run_dir in sorted((RES / "multi_seed" / dataset).glob("*/run_*")):
            yield (run_dir.parent.name, run_dir.name, run_dir / "traj_estimate_tum.txt",
                   run_dir.parent / "groundtruth_tum.txt", float(run_info(run_dir).get("fps", "nan")))
    else:
        for seq_dir in sorted((RES / "kitti").glob("*_drive_*")):
            t = seq_dir / "traj_timing.txt"
            timing = np.loadtxt(t, delimiter=",", ndmin=2) if t.exists() and t.stat().st_size else np.zeros((0, 8))
            fps = 1 / timing[:, -1].mean() if len(timing) else np.nan
            yield seq_dir.name, "run_00", seq_dir / "traj_estimate_tum.txt", seq_dir / "groundtruth_tum.txt", fps


def evaluate(job):
    dataset, seq, run, est_file, gt_file, fps = job
    row = {"dataset": dataset, "sequence": seq, "run": run, "fps": fps}
    if not est_file.exists() or np.loadtxt(est_file, ndmin=2).shape[0] < 10:
        return {**row, "failed": True}
    traj_ref = file_interface.read_tum_trajectory_file(gt_file)
    gt_path = traj_ref.path_length
    ref, est = associate(traj_ref, file_interface.read_tum_trajectory_file(est_file), MAX_GT_GAP[dataset])
    rigid, scaled = copy.deepcopy(est), copy.deepcopy(est)
    rigid.align(ref, correct_scale=False)
    _, _, scale = scaled.align(ref, correct_scale=True)
    rescaled = copy.deepcopy(est)
    rescaled.scale(scale)

    def rmse(metric, e):
        metric.process_data((ref, e))
        return metric.get_statistic(metrics.StatisticsType.rmse)

    row["ate_se3"] = 100 * rmse(metrics.APE(PoseRelation.translation_part), rigid) / gt_path
    row["ate_sim3"] = 100 * rmse(metrics.APE(PoseRelation.translation_part), scaled) / gt_path
    for name, e in [("rpe_se3", est), ("rpe_sim3", rescaled)]:
        rpe = metrics.RPE(PoseRelation.translation_part, delta=RPE_DELTA_M, delta_unit=Unit.meters,
                          all_pairs=True, pairs_from_reference=True)
        try:
            row[name] = 100 * rmse(rpe, e) / RPE_DELTA_M
        except Exception:  # sequence shorter than the RPE delta
            row[name] = np.nan
    row["failed"] = row["ate_sim3"] > DIVERGED_ATE_PCT or not DIVERGED_SCALE[0] <= scale <= DIVERGED_SCALE[1]
    return row


if __name__ == "__main__":
    jobs = [(d, *r) for d in ["kitti", "m3ed", "tartanground"] for r in runs(d)]
    with Pool(8) as pool:
        df = pd.DataFrame(pool.map(evaluate, jobs))
    df.to_csv(RES / "table_runs.csv", index=False)

    rows = []
    for dataset, label in [("kitti", "KITTI"), ("m3ed", "M3ED"), ("tartanground", "TartanGround")]:
        g = df[df.dataset == dataset]
        n_seq = g.sequence.nunique()
        n_runs = g.groupby("sequence").size()
        n = n_runs.iloc[0] if n_runs.nunique() == 1 else f"{n_runs.min()}--{n_runs.max()}"
        per_seq = g[~g.failed].groupby("sequence")[["ate_se3", "ate_sim3", "rpe_se3", "rpe_sim3", "fps"]].mean()
        m = per_seq.mean()  # mean over the sequences (NaN, e.g. RPE of a sequence shorter than 100 m, skipped)
        failed = 100 * g.failed.mean()
        rows.append(f"      {label} & ${n_seq}{{\\times}}{n}$ & {failed:.1f} & {m.ate_se3:.2f} & {m.ate_sim3:.2f} & "
                    f"{m.rpe_se3:.2f} & {m.rpe_sim3:.2f} & {m.fps:.1f} \\\\")
        print(f"{label}: {n_seq} sequences x {n} runs, failed {g.failed.sum()}/{len(g)} "
              f"({', '.join(sorted(set(g[g.failed].sequence))) or 'none'}), averaged over {len(per_seq)} sequences")

    tex = r"""\begin{table}[t]
  \centering
  \small
  \setlength{\tabcolsep}{4pt}
  \begin{tabular}{lccccccc}
    \toprule
     & & & \multicolumn{2}{c}{\textbf{ATE (\%)}} & \multicolumn{2}{c}{\textbf{RPE-t (\%)}} & \\
    \cmidrule(lr){4-5}\cmidrule(lr){6-7}
    \textbf{Dataset} & \textbf{Runs ($X{\times}N$)} & \textbf{Failed (\%)} & $SE(3)$ & $Sim(3)$ & $SE(3)$ & $Sim(3)$ & \textbf{Avg FPS} \\
    \midrule
""" + "\n".join(r[2:] for r in rows) + r"""
    \bottomrule
  \end{tabular}\par
  \vspace{0.4cm}
  {\footnotesize\color{gray} OpenVINS, monocular camera + IMU. $X$ sequences $\times$ $N$ runs per sequence.
  ATE: RMSE in \% of the ground-truth path length; RPE-t: translational RMSE over """ + f"{RPE_DELTA_M:g}" + r""" m
  segments in \% of the segment length. Mean over the sequences of the per-sequence mean over the runs
  that did not fail (no trajectory, or diverged: $Sim(3)$ ATE $>$ """ + f"{DIVERGED_ATE_PCT:g}" + r"""\,\% of the path
  or scale outside """ + f"{DIVERGED_SCALE[0]:g}--{DIVERGED_SCALE[1]:g}" + r"""). FPS: estimator processing rate.}
\end{table}
"""
    (RES / "table.tex").write_text(tex)
    print(tex)
