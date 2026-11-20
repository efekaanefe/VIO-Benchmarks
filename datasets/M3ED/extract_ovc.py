#!/usr/bin/env python3
"""Pull only the OVC (grayscale stereo / RGB / IMU + calib) part of M3ED
data.h5 files over HTTP range requests, skipping events and lidar.

Output: <output_dir>/<seq>/<seq>_data.h5 with the same /ovc/... layout as the
original file, plus <seq>_pose_gt.h5 (full download, it is small).
"""
import argparse
import os
import sys
import time
from concurrent.futures import ThreadPoolExecutor

import fsspec
import h5py
import requests
import tqdm
import yaml

BASE_URL = "https://m3ed-dist.s3.us-west-2.amazonaws.com/processed"
DATASETS_LIST_URL = "https://raw.githubusercontent.com/daniilidis-group/m3ed/main/dataset_list.yaml"
OVC_META = ["imu", "ts", "ts_map_prophesee_left_t", "ts_map_prophesee_right_t"]


def url(seq, suffix):
    return f"{BASE_URL}/{seq}/{seq}_{suffix}"


def exists(u):
    return requests.head(u, timeout=30).ok


def chunk_list(dset):
    """[(chunk_offset, filter_mask, byte_offset, size)] sorted by file position."""
    chunks = []
    if hasattr(dset.id, "chunk_iter"):
        dset.id.chunk_iter(lambda c: chunks.append(
            (c.chunk_offset, c.filter_mask, c.byte_offset, c.size)))
    else:
        for i in range(dset.id.get_num_chunks()):
            c = dset.id.get_chunk_info(i)
            chunks.append((c.chunk_offset, c.filter_mask, c.byte_offset, c.size))
    return sorted(chunks, key=lambda c: c[2])


def group_runs(chunks, max_bytes):
    """Merge file-contiguous chunks into single range requests."""
    runs, cur = [], []
    for c in chunks:
        if cur and (c[2] != cur[-1][2] + cur[-1][3]
                    or c[2] + c[3] - cur[0][2] > max_bytes):
            runs.append(cur)
            cur = []
        cur.append(c)
    if cur:
        runs.append(cur)
    return runs


def fetch_run(session, u, run, retries=5):
    start, end = run[0][2], run[-1][2] + run[-1][3] - 1
    for attempt in range(retries):
        try:
            r = session.get(u, headers={"Range": f"bytes={start}-{end}"},
                            timeout=120)
            r.raise_for_status()
            if len(r.content) != end - start + 1:
                raise IOError("short read")
            return [(c[0], c[1], r.content[c[2] - start:c[2] - start + c[3]])
                    for c in run]
        except Exception:
            if attempt == retries - 1:
                raise
            time.sleep(2 ** attempt)


def copy_chunked(src, dst_group, name, u, workers, run_bytes, label):
    """Copy a chunked dataset by fetching its raw compressed chunks in parallel."""
    d = dst_group.create_dataset(
        name, shape=src.shape, dtype=src.dtype, chunks=src.chunks,
        compression=src.compression, compression_opts=src.compression_opts,
        shuffle=src.shuffle, fletcher32=src.fletcher32)
    for k, v in src.attrs.items():
        d.attrs[k] = v
    runs = group_runs(chunk_list(src), run_bytes)
    total = sum(c[3] for run in runs for c in run)
    session = requests.Session()
    session.mount("https://", requests.adapters.HTTPAdapter(
        pool_connections=workers, pool_maxsize=workers))
    with ThreadPoolExecutor(workers) as ex, tqdm.tqdm(
            desc=label, total=total, unit="B", unit_scale=True,
            unit_divisor=1024) as bar:
        # h5py is not thread-safe: fetch in threads, write here
        for chunks in ex.map(lambda run: fetch_run(session, u, run), runs):
            for offset, mask, data in chunks:
                d.id.write_direct_chunk(offset, data, mask)
                bar.update(len(data))


def extract_ovc(seq, out_path, cameras, workers, run_bytes):
    tmp = out_path + ".part"
    u = url(seq, "data.h5")
    # Small metadata reads only; big chunks go through copy_chunked
    remote = fsspec.open(u, "rb", block_size=2**20,
                         cache_type="blockcache").open()
    t = time.time()
    with h5py.File(remote, "r") as src, h5py.File(tmp, "w") as dst:
        for k, v in src.attrs.items():
            dst.attrs[k] = v
        ovc = dst.create_group("ovc")
        for k, v in src["ovc"].attrs.items():
            ovc.attrs[k] = v
        for name in OVC_META:
            if name in src["ovc"]:
                src.copy(src["ovc"][name], ovc, name)
        for cam in cameras:
            if cam not in src["ovc"]:
                print(f"  {seq}: ovc/{cam} missing, skipping")
                continue
            g = ovc.create_group(cam)
            for k, v in src["ovc"][cam].attrs.items():
                g.attrs[k] = v
            for name, obj in src["ovc"][cam].items():
                if isinstance(obj, h5py.Dataset) and obj.chunks and obj.nbytes > 2**26:
                    copy_chunked(obj, g, name, u, workers, run_bytes,
                                 f"{seq}/{cam}/{name}")
                else:
                    src.copy(obj, g, name)
    os.replace(tmp, out_path)
    size = os.path.getsize(out_path) / 1e9
    print(f"  {seq}: {size:.2f} GB in {time.time() - t:.0f}s")


def download_small(u, path):
    r = requests.get(u, timeout=120)
    r.raise_for_status()
    with open(path + ".part", "wb") as f:
        f.write(r.content)
    os.replace(path + ".part", path)


if __name__ == "__main__":
    p = argparse.ArgumentParser()
    p.add_argument("--vehicle", nargs="+", default=["car", "falcon"])
    p.add_argument("--environment", nargs="+",
                   default=["urban", "indoor", "forest", "outdoor"])
    p.add_argument("--cameras", nargs="+", default=["left", "right", "rgb"],
                   choices=["left", "right", "rgb"])
    p.add_argument("--train_test", choices=["train", "test", "both"],
                   default="train",
                   help="Test sequences have no public pose_gt")
    p.add_argument("--sequences", nargs="+",
                   help="Explicit sequence names (overrides filters)")
    p.add_argument("--yaml", default=None)
    p.add_argument("--output_dir", default="data")
    p.add_argument("--workers", type=int, default=16,
                   help="Parallel HTTP range requests")
    p.add_argument("--run_mb", type=int, default=32,
                   help="Max bytes per merged range request")
    p.add_argument("--no_download", action="store_true",
                   help="Only list the selected sequences")
    args = p.parse_args()

    if args.yaml and os.path.exists(args.yaml):
        with open(args.yaml) as f:
            dataset_list = yaml.safe_load(f)
    else:
        dataset_list = yaml.safe_load(requests.get(DATASETS_LIST_URL).text)

    seqs = []
    for entry in dataset_list:
        name = entry["file"]
        if entry["filetype"] != "data":
            continue
        if args.sequences:
            if name in args.sequences:
                seqs.append(entry)
            continue
        if name.split("_")[0] not in args.vehicle:
            continue
        if not any(e in name for e in args.environment):
            continue
        if args.train_test == "train" and entry["is_test_file"]:
            continue
        if args.train_test == "test" and not entry["is_test_file"]:
            continue
        seqs.append(entry)

    print(f"{len(seqs)} sequences, cameras: {', '.join(args.cameras)}")
    for e in seqs:
        print(f"  {e['file']}{' (test)' if e['is_test_file'] else ''}")
    if args.no_download:
        sys.exit()

    failed = []
    for e in seqs:
        seq = e["file"]
        seq_dir = os.path.join(args.output_dir, seq)
        os.makedirs(seq_dir, exist_ok=True)
        print(f"[{seq}]")
        try:
            if not exists(url(seq, "data.h5")):
                print(f"  {seq}: not on server, skipping")
                failed.append(seq)
                continue
            data_path = os.path.join(seq_dir, f"{seq}_data.h5")
            if os.path.exists(data_path):
                print(f"  {seq}: data already extracted")
            else:
                extract_ovc(seq, data_path, args.cameras,
                            args.workers, args.run_mb * 2**20)
            pose_path = os.path.join(seq_dir, f"{seq}_pose_gt.h5")
            if not e["is_test_file"] and not os.path.exists(pose_path):
                download_small(url(seq, "pose_gt.h5"), pose_path)
                print(f"  {seq}: pose_gt downloaded")
        except Exception as ex:  # keep going on network hiccups
            print(f"  {seq}: FAILED: {ex}")
            failed.append(seq)

    if failed:
        print(f"Failed/skipped: {', '.join(failed)} (re-run to retry)")
        sys.exit(1)
    print("Done.")
