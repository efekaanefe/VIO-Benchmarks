#!/usr/bin/env python3
"""Download a monocular-VIO subset of TartanGround diff-drive sequences.

Per trajectory only lcam_front images, imu and metadata (poses) are fetched.
The first --max-per-env trajectories of every environment are selected.
"""
import argparse
import os
import shutil
from collections import defaultdict

import tartanair as ta

VERSION = "diff"
FILES = ("image_lcam_front.zip", "imu.zip", "metadata.zip")


def list_trajectories(max_per_env, envs):
    """Return {env: [(traj, size_gb), ...]} from the toolkit's file list."""
    listing = os.path.join(os.path.dirname(ta.__file__), "download_ground_files.txt")
    sizes = defaultdict(float)
    with open(listing) as f:
        for line in f:
            parts = line.split()
            tokens = parts[0].split("/")
            if len(tokens) == 4 and tokens[1] == f"Data_{VERSION}" and tokens[3] in FILES:
                sizes[(tokens[0], tokens[2])] += float(parts[1])
    selected = defaultdict(list)
    for (env, traj), size in sorted(sizes.items()):
        if envs and env not in envs:
            continue
        if len(selected[env]) < max_per_env:
            selected[env].append((traj, size))
    return selected


def is_done(output_dir, env, traj):
    # Extracted trajectories keep the folder but lose their zips (delete_zip=True)
    path = os.path.join(output_dir, env, f"Data_{VERSION}", traj)
    return (os.path.isdir(os.path.join(path, "image_lcam_front"))
            and os.path.isdir(os.path.join(path, "imu"))
            and not any(f.endswith(".zip") for f in os.listdir(path)))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output_dir", required=True)
    parser.add_argument("--max-per-env", type=int, default=3)
    parser.add_argument("--envs", nargs="*", default=[], help="Restrict to these environments")
    parser.add_argument("--margin_gb", type=float, default=20.0, help="Free space to keep after download")
    parser.add_argument("--dry-run", action="store_true")
    args = parser.parse_args()

    os.makedirs(args.output_dir, exist_ok=True)
    selected = list_trajectories(args.max_per_env, set(args.envs))
    todo = {env: [(t, s) for t, s in trajs if not is_done(args.output_dir, env, t)]
            for env, trajs in selected.items()}

    for env, trajs in selected.items():
        print(f"{env:28s} {', '.join(t for t, _ in trajs):20s} {sum(s for _, s in trajs):6.2f} GB")
    total = sum(s for trajs in selected.values() for _, s in trajs)
    remaining = sum(s for trajs in todo.values() for _, s in trajs)
    n_total = sum(len(v) for v in selected.values())
    n_todo = sum(len(v) for v in todo.values())
    free = shutil.disk_usage(args.output_dir).free / 1e9
    print(f"\nSelected {n_total} trajectories in {len(selected)} envs: {total:.1f} GB")
    print(f"Remaining {n_todo} trajectories: {remaining:.1f} GB | free disk: {free:.1f} GB")

    if remaining + args.margin_gb > free:
        raise SystemExit(f"Not enough space (need {remaining:.1f} + {args.margin_gb:.0f} GB margin).")
    if args.dry_run:
        return

    ta.init(args.output_dir)
    for env, trajs in todo.items():
        if not trajs:
            continue
        print(f"\n=== {env}: {[t for t, _ in trajs]}")
        ta.download_ground(env=[env], version=[VERSION], traj=[t for t, _ in trajs],
                           modality=["image", "imu", "meta"], camera_name=["lcam_front"],
                           unzip=True, delete_zip=True)


if __name__ == "__main__":
    main()
