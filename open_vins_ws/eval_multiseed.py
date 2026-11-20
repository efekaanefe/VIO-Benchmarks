#!/usr/bin/env python3
"""Evaluate the repeated OpenVINS runs in results/multi_seed/<dataset> (written by run_multiseed.py).

    python3 eval_multiseed.py m3ed
    python3 eval_multiseed.py kitti

Same metrics and tables as the DPVO multi-seed evaluation: Sim(3)-aligned ATE and RPE over
10/50/100 m of ground truth path on the rescaled estimate, computed for every run. Each error is
reported as an absolute value and normalised by distance:
    ATE trans        % of the ground truth path length
    RPE trans @d     % of the segment length d
    RPE rot @d       deg per 100 m (rmse / d * 100)
    ATE rot          not normalised (an orientation offset does not grow with distance)
OpenVINS is metric, so two more are added: ATE after SE(3) alignment (no scale correction) and
the scale of the Sim(3) alignment.

Every run gets an outcome: ok, diverged (ATE above DIVERGED_ATE_PCT of the path or scale outside
DIVERGED_SCALE), no_init (no trajectory) or crashed (stopped before the end of the bag). The
statistics only use the ok runs.

Writes into results/multi_seed/<dataset>/:
    runs.csv                  every sequence x run x metric (full evo statistics, seed, outcome)
    summary_by_sequence.csv   per sequence x metric: outcome counts, statistics of the per-run RMSE
    summary_by_run.csv        per run x metric: mean/median over the sequences
    summary_overall.csv       per metric: statistics over the runs of the per-run dataset mean
    REPORT.md                 the same tables, readable
    <sequence>/groundtruth_tum.txt, <sequence>/metrics.csv (one row per run)
"""
import argparse
import copy
import re
import shutil
from pathlib import Path

import numpy as np
import pandas as pd
from evo.core import metrics
from evo.core.metrics import PoseRelation, Unit
from evo.core.trajectory import PoseTrajectory3D
from evo.tools import file_interface
from scipy.spatial.transform import Rotation, Slerp

RES = Path(__file__).resolve().parent / "results"
OUT = RES / "multi_seed"
MAX_GT_GAP = {"m3ed": 0.15, "kitti": 0.05, "tartanground": 0.15}  # s, estimates between ground truth poses further apart are dropped
RPE_DELTAS_M = [10.0, 50.0, 100.0]
STATS = ["rmse", "mean", "median", "std", "min", "max"]
AGG = ["mean", "std", "median", "min", "max"]
DIVERGED_ATE_PCT = 10.0  # Sim(3) ATE RMSE in % of the ground truth path
DIVERGED_SCALE = (0.5, 2.0)
OUTCOMES = ["ok", "diverged", "no_init", "crashed"]


def associate(traj_ref, traj_est, max_gap):
    """Ground truth timestamps do not coincide with the camera timestamps of the estimate, so
    interpolate the ground truth at the estimate timestamps (same as eval_m3ed.py)."""
    t_ref, t_est = traj_ref.timestamps, traj_est.timestamps
    i = np.searchsorted(t_ref, t_est)  # t_ref[i-1] <= t_est < t_ref[i]
    ok = (i > 0) & (i < len(t_ref))
    ok[ok] &= (t_ref[i[ok]] - t_ref[i[ok] - 1]) < max_gap
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


def normalise(metric, rmse, gt_path):
    """(normalised value, unit) for a metric, or (nan, '') if it has no distance normalisation."""
    if metric.startswith("ATE trans"):
        return 100 * rmse / gt_path, "% of path"
    m = re.match(r"RPE (trans|rot) @(\d+)m", metric)
    if m and m.group(1) == "trans":
        return 100 * rmse / float(m.group(2)), "% of segment"
    if m:
        return 100 * rmse / float(m.group(2)), "deg/100m"
    return np.nan, ""


def evaluate_run(traj_ref_full, est_file, max_gap):
    traj_est = file_interface.read_tum_trajectory_file(est_file)
    est_poses = traj_est.num_poses
    est_start = traj_est.timestamps[0] - traj_ref_full.timestamps[0]
    traj_ref, traj_est = associate(traj_ref_full, traj_est, max_gap)
    scaled = copy.deepcopy(traj_est)
    _, _, scale = scaled.align(traj_ref, correct_scale=True)
    rescaled = copy.deepcopy(traj_est)
    rescaled.scale(scale)
    rigid = copy.deepcopy(traj_est)
    rigid.align(traj_ref, correct_scale=False)

    head = {"est_poses": est_poses, "matched": traj_est.num_poses, "est_start_s": est_start, "sim3_scale": scale}
    rows = []
    jobs = [("ATE trans Sim3 [m]", metrics.APE(PoseRelation.translation_part), scaled),
            ("ATE rot [deg]", metrics.APE(PoseRelation.rotation_angle_deg), scaled)]
    for d in RPE_DELTAS_M:
        for name, rel in [(f"RPE trans @{d:g}m [m]", PoseRelation.translation_part),
                          (f"RPE rot @{d:g}m [deg]", PoseRelation.rotation_angle_deg)]:
            # segments of d metres on the ground truth, so a scale error does not change their length
            jobs.append((name, metrics.RPE(rel, delta=d, delta_unit=Unit.meters, all_pairs=True,
                                           pairs_from_reference=True), rescaled))
    jobs.append(("ATE trans SE3 [m]", metrics.APE(PoseRelation.translation_part), rigid))
    for name, metric, est in jobs:
        try:
            s, n = stats(metric, (traj_ref, est))
        except Exception:  # sequence shorter than the RPE delta
            continue
        rows.append({**head, "metric": name, "pairs": n, **s})
    # the scale as a metric of its own, to get the same statistics over the runs
    rows.append({**head, "metric": "Sim3 scale [gt/est]", "pairs": traj_est.num_poses,
                 **{k: (0.0 if k == "std" else scale) for k in STATS}})
    return rows


def run_info(run_dir):
    info = {}
    log = run_dir / "run.log"
    if log.exists():
        for line in log.read_text().splitlines():
            key, _, value = line.partition(": ")
            info[key] = value
    return info


def evaluate(dataset):
    base = OUT / dataset
    rows, outcomes = [], []
    for seq_dir in sorted(p for p in base.iterdir() if p.is_dir() and any(p.glob("run_*/run.log"))):
        seq = seq_dir.name
        gt_file = seq_dir / "groundtruth_tum.txt"
        if not gt_file.exists():  # same ground truth (IMU frame) as the single-run results
            shutil.copy(RES / dataset / seq / "groundtruth_tum.txt", gt_file)
        traj_ref = file_interface.read_tum_trajectory_file(gt_file)
        for run_dir in sorted(p for p in seq_dir.glob("run_*") if (p / "run.log").exists()):
            info = run_info(run_dir)
            status = info.get("status", "missing")
            run = {"sequence": seq, "run": int(run_dir.name[4:]), "seed": int(info.get("seed", -1)),
                   "time_s": float(info.get("time_s", "nan")),
                   "fps": float(info.get("fps", "nan")), "gt_path_m": traj_ref.path_length}
            est_file = run_dir / "traj_estimate_tum.txt"
            res = evaluate_run(traj_ref, est_file, MAX_GT_GAP[dataset]) if est_file.exists() else []
            if res:  # estimator processing rate (run.log), as a metric to get the same statistics
                res.append({**res[-1], "metric": "Processing rate [fps]",
                            **{k: (0.0 if k == "std" else run["fps"]) for k in STATS}})
            if not res:
                outcome = "crashed" if "crashed" in status else "no_init"
            elif "crashed" in status:
                outcome = "crashed"
            else:
                ate_pct = 100 * res[0]["rmse"] / traj_ref.path_length
                scale = res[0]["sim3_scale"]
                bad = ate_pct > DIVERGED_ATE_PCT or not DIVERGED_SCALE[0] <= scale <= DIVERGED_SCALE[1]
                outcome = "diverged" if bad else "ok"
            outcomes.append({**run, "outcome": outcome, "status": status})
            for r in res:
                norm, unit = normalise(r["metric"], r["rmse"], traj_ref.path_length)
                rows.append({**run, "outcome": outcome, **r, "rmse_norm": norm, "norm_unit": unit})
        print(f"{seq}: " + ", ".join(f"{sum(o['sequence'] == seq and o['outcome'] == k for o in outcomes)} {k}"
                                    for k in OUTCOMES), flush=True)
    summarise(dataset, base, pd.DataFrame(rows), pd.DataFrame(outcomes))


def summarise(dataset, base, runs, outcomes):
    metric_order = list(dict.fromkeys(runs["metric"]))
    runs.to_csv(base / "runs.csv", index=False)
    ok = runs[runs.outcome == "ok"]
    counts = (outcomes.groupby(["sequence", "outcome"]).size().unstack(fill_value=0)
              .reindex(columns=OUTCOMES, fill_value=0).add_prefix("n_"))
    counts.insert(0, "n_runs", counts.sum(axis=1))

    # per sequence: one row per run (wide, failed runs included), and statistics over the ok runs
    for seq, o in outcomes.groupby("sequence"):
        g = runs[runs["sequence"] == seq]
        head = o.set_index("run")[["seed", "time_s", "fps", "outcome", "status"]]
        parts = [head]
        if len(g):
            parts.append(g.drop_duplicates("run").set_index("run")[["matched", "est_start_s", "sim3_scale"]])
            parts.append(g.pivot(index="run", columns="metric", values="rmse")[[m for m in metric_order if m in set(g["metric"])]])
            norm = g.pivot(index="run", columns="metric", values="rmse_norm").dropna(axis=1, how="all")
            norm.columns = [f"{c.rsplit(' [', 1)[0]} [{g[g.metric == c].norm_unit.iloc[0]}]" for c in norm.columns]
            parts.append(norm)
        pd.concat(parts, axis=1).to_csv(base / seq / "metrics.csv")

    by_seq = ok.groupby(["sequence", "metric"], sort=False).agg(
        norm_unit=("norm_unit", "first"),
        **{f"abs_{a}": ("rmse", a) for a in AGG}, **{f"norm_{a}": ("rmse_norm", a) for a in AGG}).reset_index()
    # every sequence x metric, also where no run was ok
    grid = pd.MultiIndex.from_product([sorted(outcomes["sequence"].unique()), metric_order], names=["sequence", "metric"])
    by_seq = by_seq.set_index(["sequence", "metric"]).reindex(grid).reset_index()
    by_seq["norm_unit"] = by_seq["metric"].map(runs.drop_duplicates("metric").set_index("metric")["norm_unit"])
    by_seq = (by_seq.merge(outcomes.groupby("sequence")["gt_path_m"].first().reset_index(), on="sequence")
              .merge(counts.reset_index(), on="sequence"))
    lead = ["sequence", "metric", "gt_path_m", "n_runs"] + [f"n_{k}" for k in OUTCOMES] + ["norm_unit"]
    by_seq = by_seq[lead + [c for c in by_seq.columns if c not in lead]]
    by_seq.to_csv(base / "summary_by_sequence.csv", index=False)

    # per run: aggregate over the sequences. Only sequences that are ok in every run, so that
    # each row averages the same set of sequences.
    common = sorted(counts.index[counts.n_ok == counts.n_runs.max()])
    complete = ok[ok["sequence"].isin(common)]
    by_run = complete.groupby(["run", "metric"], sort=False).agg(
        n_sequences=("rmse", "size"), norm_unit=("norm_unit", "first"),
        abs_mean=("rmse", "mean"), abs_median=("rmse", "median"),
        norm_mean=("rmse_norm", "mean"), norm_median=("rmse_norm", "median")).reset_index()
    ok_per_run = outcomes[outcomes.outcome == "ok"].groupby("run").size()
    by_run.insert(2, "n_ok_all_sequences", by_run["run"].map(ok_per_run).fillna(0).astype(int))
    by_run.to_csv(base / "summary_by_run.csv", index=False)

    overall = by_run.groupby("metric", sort=False).agg(
        n_runs=("abs_mean", "size"), norm_unit=("norm_unit", "first"),
        **{f"abs_{a}": ("abs_mean", a) for a in AGG}, **{f"norm_{a}": ("norm_mean", a) for a in AGG}).reset_index()
    overall.to_csv(base / "summary_overall.csv", index=False)

    write_report(dataset, base, outcomes, counts, by_seq, by_run, overall, metric_order, common)
    print("saved", base / "REPORT.md")


def fmt(v, digits=2):
    return "–" if pd.isna(v) else f"{v:.{digits}f}"


def outcome_text(c):
    return ", ".join(f"{c[f'n_{k}']} {k}" for k in OUTCOMES if c[f"n_{k}"])


def write_report(dataset, base, outcomes, counts, by_seq, by_run, overall, metric_order, common):
    n_runs = counts.n_runs
    count = f"{n_runs.min()}" if n_runs.min() == n_runs.max() else f"{n_runs.min()}–{n_runs.max()}"
    total = counts[[f"n_{k}" for k in OUTCOMES]].sum()
    L = [f"# OpenVINS results: {dataset} (repeated runs)", "",
         f"{len(counts)} sequences, {count} runs each, monocular camera + IMU. OpenVINS has no seed option: "
         "it fixes the OpenCV seed to 0, so the runs are repeats of the same configuration and differ only "
         "through multi-threading.", "",
         f"**Outcome of the {int(n_runs.sum())} runs: " + ", ".join(f"{int(total[f'n_{k}'])} {k}" for k in OUTCOMES) + ".** "
         f"*diverged*: Sim(3) ATE above {DIVERGED_ATE_PCT:g} % of the path or Sim(3) scale outside "
         f"{DIVERGED_SCALE[0]:g}–{DIVERGED_SCALE[1]:g}; *no_init*: no trajectory; *crashed*: stopped before the end "
         "of the bag. Only the *ok* runs enter the statistics; the others are in `runs.csv` and each "
         "`<sequence>/metrics.csv`.", "",
         "All errors are computed after a Sim(3) alignment to ground truth, the same way as for DPVO, so the "
         "two can be compared. OpenVINS estimates metric scale, so two tables are added: the ATE after an "
         "SE(3) alignment (no scale correction) and the Sim(3) scale (below 1: the estimate is too long). "
         "The last table is the processing rate: frames per second of estimator time (tracking + filter "
         "update, without reading the bag), with one run at a time on the machine. "
         "Every number below is built from the per-run **RMSE** of the metric.", "",
         "Normalised errors: ATE trans as % of the ground truth path length; RPE trans as % of the segment "
         "length; RPE rot in deg per 100 m. ATE rot has no distance normalisation. RPE segments are measured "
         "along the ground truth path. Ground truth and estimate are the IMU pose.", ""]

    L += ["## 1. Per sequence: statistics over the runs", ""]
    for metric in metric_order:
        g = by_seq[by_seq.metric == metric]
        unit = metric.rsplit("[", 1)[1].rstrip("]")
        nunit = g.norm_unit.iloc[0]
        L += [f"### {metric.rsplit(' [', 1)[0]}", ""]
        head = f"| sequence | path [m] | runs | outcome | mean [{unit}] | std | median | min | max |"
        sep = "|---|---:|---:|---|---:|---:|---:|---:|---:|"
        if nunit:
            head += f" mean [{nunit}] | std | median | min | max |"
            sep += "---:|---:|---:|---:|---:|"
        L += [head, sep]
        for _, r in g.iterrows():
            row = (f"| {r.sequence} | {r.gt_path_m:.0f} | {r.n_runs} | {outcome_text(r)} | " +
                   " | ".join(fmt(r[f"abs_{a}"], 3) for a in AGG) + " |")
            if nunit:
                row += " " + " | ".join(fmt(r[f"norm_{a}"], 3) for a in AGG) + " |"
            L.append(row)
        L.append("")

    n_seq = len(counts)
    L += ["## 2. Per run: mean over the sequences", "",
          "Run *k* is the *k*-th trial of every sequence, so one row is one complete pass over the dataset. "
          "The last rows are statistics over those passes. So that every row averages the same sequences, "
          f"only the {len(common)} of {n_seq} sequences that are ok in every run are averaged" +
          ("." if len(common) == n_seq else
           " (left out: " + ", ".join(s for s in counts.index if s not in common) + ").") +
          " *ok* counts the ok sequences of that pass over all " + f"{n_seq}.", ""]
    ok_per_run = by_run.drop_duplicates("run").set_index("run")["n_ok_all_sequences"]
    for label, col, units in [("Absolute", "abs_mean", {m: m.rsplit("[", 1)[1].rstrip("]") for m in metric_order}),
                              ("Normalised", "norm_mean", dict(zip(by_run.metric, by_run.norm_unit)))]:
        cols = [m for m in metric_order if units.get(m)]
        table = by_run.pivot(index="run", columns="metric", values=col)[cols]
        L += [f"### {label}", "",
              "| run | ok | " + " | ".join(f"{m.rsplit(' [', 1)[0]} [{units[m]}]" for m in cols) + " |",
              "|---|---:|" + "---:|" * len(cols)]
        for run, r in table.iterrows():
            L.append(f"| run_{run:02d} | {ok_per_run[run]}/{n_seq} | " + " | ".join(fmt(r[m], 3) for m in cols) + " |")
        for a in AGG:
            L.append(f"| **{a}** | | " + " | ".join(f"**{fmt(table[m].agg(a), 3)}**" for m in cols) + " |")
        L.append("")
    L += ["`summary_by_run.csv` also has the median over the sequences for every run.", ""]
    notes = base / "REPORT_NOTES.md"  # hand-written sections, kept when the report is regenerated
    if notes.exists():
        L += [notes.read_text()]
    (base / "REPORT.md").write_text("\n".join(L))


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("datasets", nargs="+", help="m3ed and/or kitti")
    for dataset in parser.parse_args().datasets:
        evaluate(dataset)
