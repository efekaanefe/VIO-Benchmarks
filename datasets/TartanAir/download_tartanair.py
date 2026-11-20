#!/usr/bin/env python3
"""Download a monocular-VIO subset of TartanAir v2 (lcam_front images + poses + imu).

TartanAir v2 zips bundle every trajectory of an env/difficulty, so instead of the
toolkit we read the remote zips with HTTP range requests and extract only the
selected trajectories. Selection: round-robin over every (env, difficulty), each
round adding the trajectory closest to that group's median length, until the
budget is used. This keeps the env/easy/hard distribution of the full dataset.
"""
import argparse
import io
import json
import os
import random
import shutil
import statistics
import tempfile
import time
import zipfile
from collections import defaultdict
from concurrent.futures import ThreadPoolExecutor

import requests
import tartanair as ta

REPO_URL = "https://huggingface.co/datasets/theairlabcmu/tartanair2/resolve/main"
DIFFICULTIES = ("easy", "hard")


class HTTPFile(io.RawIOBase):
    """Seekable read-only file over HTTP range requests, with an optional local cache of one span."""

    def __init__(self, url):
        self.session = requests.Session()
        for attempt in range(5):
            try:
                r = self.session.head(url, allow_redirects=True, timeout=60)
                r.raise_for_status()
                break
            except requests.RequestException:
                if attempt == 4:
                    raise
                time.sleep(5 * (attempt + 1))
        self.url, self.size, self.pos = r.url, int(r.headers["Content-Length"]), 0
        self.cache_start, self.cache = 0, None

    def readable(self):
        return True

    def seekable(self):
        return True

    def tell(self):
        return self.pos

    def seek(self, offset, whence=0):
        self.pos = {0: offset, 1: self.pos + offset, 2: self.size + offset}[whence]
        return self.pos

    def _get(self, start, end, stream=False):
        for attempt in range(5):
            try:
                r = self.session.get(self.url, headers={"Range": f"bytes={start}-{end - 1}"},
                                     timeout=120, stream=stream)
                r.raise_for_status()
                if r.status_code != 206:
                    raise IOError("server ignored Range header")
                return r
            except (requests.RequestException, IOError):
                if attempt == 4:
                    raise
                time.sleep(5 * (attempt + 1))

    def readinto(self, b):
        end = min(self.pos + len(b), self.size)
        if self.pos >= end:
            return 0
        if self.cache is not None and self.cache_start <= self.pos and end <= self.cache_start + len(self.cache):
            data = self.cache[self.pos - self.cache_start:end - self.cache_start]
        else:
            data = self._get(self.pos, end).content
        b[:len(data)] = data
        self.pos += len(data)
        return len(data)

    def prefetch(self, start, end, tmp_dir):
        """Download [start, end) once into a memory-mapped temp file so later reads are local."""
        import mmap
        tmp = tempfile.TemporaryFile(dir=tmp_dir)
        with self._get(start, end, stream=True) as r:
            for chunk in r.iter_content(8 << 20):
                tmp.write(chunk)
        tmp.flush()
        if tmp.tell() != end - start:
            raise IOError(f"short read: {tmp.tell()} of {end - start} bytes")
        self.cache_start, self.cache = start, mmap.mmap(tmp.fileno(), 0, access=mmap.ACCESS_READ)


def open_remote_zip(rel_path):
    remote = HTTPFile(f"{REPO_URL}/{rel_path}")
    return remote, zipfile.ZipFile(io.BufferedReader(remote, 1 << 20))


def traj_of(name):
    parts = name.split("/")
    return parts[2] if len(parts) > 3 else None


def index_group(env, diff):
    """Return {traj: compressed_bytes} for the lcam_front image zip of one env/difficulty."""
    _, z = open_remote_zip(f"{env}/Data_{diff}/image_lcam_front.zip")
    sizes = defaultdict(int)
    for info in z.infolist():
        if traj_of(info.filename):
            sizes[traj_of(info.filename)] += info.compress_size
    return (env, diff), dict(sizes)


def build_index(index_path, workers):
    if os.path.exists(index_path):
        with open(index_path) as f:
            return {tuple(k.split("|")): v for k, v in json.load(f).items()}
    listing = os.path.join(os.path.dirname(ta.__file__), "download_files.txt")
    groups = sorted({tuple(line.split()[0].split("/")[:2]) for line in open(listing)
                     if line.split()[0].endswith("/image_lcam_front.zip")})
    groups = [(env, sub[len("Data_"):]) for env, sub in groups if sub[len("Data_"):] in DIFFICULTIES]
    print(f"Indexing {len(groups)} remote zips...")
    with ThreadPoolExecutor(workers) as pool:
        index = dict(pool.map(lambda g: index_group(*g), groups))
    with open(index_path, "w") as f:
        json.dump({"|".join(k): v for k, v in index.items()}, f, indent=1)
    return index


def select(index, budget_gb, max_per_group):
    """Round-robin: each round add, per group, the remaining trajectory closest to the group median."""
    queues = {}
    for group, trajs in index.items():
        med = statistics.median(trajs.values())
        queues[group] = sorted(trajs, key=lambda t: (abs(trajs[t] - med), t))
    # Fixed shuffled group order so a partial last round is not biased towards early letters
    order = sorted(queues)
    random.Random(0).shuffle(order)
    selected, total = defaultdict(list), 0.0
    for rnd in range(max_per_group):
        for group in order:
            if rnd >= len(queues[group]):
                continue
            traj = queues[group][rnd]
            size = index[group][traj] / 1e9
            if total + size > budget_gb:
                return selected, total
            selected[group].append(traj)
            total += size
    return selected, total


def extract_trajs(rel_path, trajs, output_dir, tmp_dir):
    """Extract the members of the given trajectories from a remote zip."""
    remote, z = open_remote_zip(rel_path)
    by_traj = defaultdict(list)
    for info in z.infolist():
        if traj_of(info.filename) in trajs:
            by_traj[traj_of(info.filename)].append(info)
    for traj, members in by_traj.items():
        # The bulk (images / imu arrays) of one trajectory is stored contiguously: fetch that
        # span in one request; small stragglers like pose files are read with their own requests
        bulk = [m for m in members if m.filename.count("/") > 3 and not m.is_dir()]
        start = min(m.header_offset for m in bulk)
        end = min(max(m.header_offset + 30 + len(m.filename.encode()) + len(m.extra) + m.compress_size + 1024
                      for m in bulk), remote.size)
        if end - start < 1.5 * sum(m.compress_size for m in bulk) + (1 << 20):
            remote.prefetch(start, end, tmp_dir)
        for m in members:
            z.extract(m, output_dir)
        if remote.cache is not None:
            remote.cache.close()
            remote.cache = None


def is_done(output_dir, env, diff, traj):
    path = os.path.join(output_dir, env, f"Data_{diff}", traj)
    return os.path.exists(os.path.join(path, ".done"))


def download_traj(output_dir, env, diff, traj, tmp_dir):
    path = os.path.join(output_dir, env, f"Data_{diff}", traj)
    for name in ("image_lcam_front.zip", "imu.zip"):
        extract_trajs(f"{env}/Data_{diff}/{name}", {traj}, output_dir, tmp_dir)
    n_img = len(os.listdir(os.path.join(path, "image_lcam_front")))
    n_pose = sum(1 for _ in open(os.path.join(path, "pose_lcam_front.txt")))
    if n_img != n_pose:
        raise IOError(f"{path}: {n_img} images vs {n_pose} poses")
    open(os.path.join(path, ".done"), "w").close()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output_dir", required=True)
    parser.add_argument("--budget_gb", type=float, default=180.0)
    parser.add_argument("--max-per-group", type=int, default=99, help="Max trajectories per env/difficulty")
    parser.add_argument("--envs", nargs="*", default=[], help="Restrict to these environments")
    parser.add_argument("--margin_gb", type=float, default=20.0, help="Free space to keep after download")
    parser.add_argument("--workers", type=int, default=4)
    parser.add_argument("--dry-run", action="store_true")
    args = parser.parse_args()

    os.makedirs(args.output_dir, exist_ok=True)
    index = build_index(os.path.join(os.path.dirname(os.path.abspath(args.output_dir)), "traj_index.json"),
                        args.workers * 4)
    selected, total = select(index, args.budget_gb, args.max_per_group)
    if args.envs:  # subset of the global selection, so partial runs keep the same distribution
        selected = {g: t for g, t in selected.items() if g[0] in args.envs}
        total = sum(index[g][t] for g, trajs in selected.items() for t in trajs) / 1e9

    envs = sorted({env for env, _ in selected})
    for env in envs:
        row = "  ".join(f"{d}: {','.join(selected.get((env, d), [])) or '-':12s}" for d in DIFFICULTIES)
        print(f"{env:28s} {row}")
    jobs = [(env, diff, t) for (env, diff), trajs in sorted(selected.items()) for t in trajs]
    todo = [j for j in jobs if not is_done(args.output_dir, *j)]
    remaining = sum(index[(e, d)][t] for e, d, t in todo) / 1e9
    n_diff = {d: sum(1 for j in jobs if j[1] == d) for d in DIFFICULTIES}
    free = shutil.disk_usage(args.output_dir).free / 1e9
    print(f"\nSelected {len(jobs)} trajectories ({n_diff['easy']} easy / {n_diff['hard']} hard) "
          f"in {len(envs)} envs: {total:.1f} GB")
    print(f"Remaining {len(todo)} trajectories: {remaining:.1f} GB | free disk: {free:.1f} GB")
    if remaining + args.margin_gb > free:
        raise SystemExit(f"Not enough space (need {remaining:.1f} + {args.margin_gb:.0f} GB margin).")
    if args.dry_run:
        return

    tmp_dir = os.path.join(args.output_dir, ".tmp")
    os.makedirs(tmp_dir, exist_ok=True)

    def run(job):
        for attempt in range(3):
            try:
                download_traj(args.output_dir, *job, tmp_dir)
                print(f"done {'/'.join(job)}", flush=True)
                return True
            except Exception as e:
                print(f"retry {'/'.join(job)} ({attempt + 1}/3): {e}", flush=True)
        return False

    with ThreadPoolExecutor(args.workers) as pool:
        failed = [j for j, ok in zip(todo, pool.map(run, todo)) if not ok]
    shutil.rmtree(tmp_dir, ignore_errors=True)
    if failed:
        raise SystemExit(f"{len(failed)} trajectories failed, re-run to retry: {failed}")


if __name__ == "__main__":
    main()
