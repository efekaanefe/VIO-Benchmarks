#!/usr/bin/env python3
"""Turn a KITTI raw drive into the generic CSVs that `make_rosbag.py` consumes.

KITTI is awkward for VIO for three reasons, all handled here:

* The IMU is not a CSV but 30-column OXTS text files, one per sample.
* The `_sync` archive downsamples the IMU to the 10 Hz camera rate, which is useless for
  pre-integration; the 100 Hz stream only exists in `_extract`.
* The camera-to-IMU transform is not given directly, only as a chain of three calibration
  files that must be composed.

So by default this pairs the **rectified images from `_sync`** (clean pinhole, no
distortion) with the **100 Hz OXTS from `_extract`**. Both archives carry absolute UTC
timestamps, so mixing them is well defined.

    python3 kitti_to_csv.py /path/to/2011_09_26 --drive 0002 -o kitti_0002

Writes `imu.csv`, `images.csv`, `gt_tum.txt` and prints the calibration block for the
VINS-Mono config. No ROS required.
"""

from __future__ import annotations

import argparse
import datetime
import re
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Sequence

import numpy as np

# KITTI OXTS column layout, from the dataset's own dataformat.txt.
OXTS_COLUMNS = [
    "lat", "lon", "alt", "roll", "pitch", "yaw",
    "vn", "ve", "vf", "vl", "vu",
    "ax", "ay", "az", "af", "al", "au",
    "wx", "wy", "wz", "wf", "wl", "wu",
    "pos_accuracy", "vel_accuracy", "navstat", "numsats",
    "posmode", "velmode", "orimode",
]

# OXTS publishes each inertial quantity twice: once body-fixed along the vehicle's
# front/left/top axes (ax/ay/az, wx/wy/wz) and once resolved into an earth-level frame
# (af/al/au, wf/wl/wu). `dataformat.txt` describes both in near-identical words, but the
# difference between the two is explained to corr > 0.98 by gravity tilting through
# roll/pitch, which identifies the f/l/u family as the levelled one.
#
# VIO needs the body-fixed triple: the estimator solves for attitude itself, so levelled
# input double-counts the levelling, and the camera-to-IMU extrinsic built from
# calib_imu_to_velo.txt is defined against the IMU's body frame. Gravity is present in
# both families, so these are specific force either way.
ACCEL_FIELDS = ("ax", "ay", "az")
GYRO_FIELDS = ("wx", "wy", "wz")

EARTH_RADIUS_M = 6378137.0


@dataclass(frozen=True)
class Calibration:
    """Everything the VINS-Mono config needs, already composed."""

    fx: float
    fy: float
    cx: float
    cy: float
    width: int
    height: int
    T_imu_cam: np.ndarray  # 4x4, camera -> IMU (VINS extrinsicRotation/Translation)


# --------------------------------------------------------------------------------------
# Parsing
# --------------------------------------------------------------------------------------


def strictly_increasing(stamps: np.ndarray) -> np.ndarray:
    """Boolean mask keeping only samples whose stamp beats every earlier kept one.

    Some KITTI raw drives contain OXTS clock-resync glitches: the stamp jumps backwards by
    ~50 ms and then catches up. Both estimators require monotonic stamps, and ground truth
    has to stay on the same filtered clock as the bag, so the drop happens once, here.
    """
    keep = np.zeros(len(stamps), dtype=bool)
    highest = -np.inf
    for i, stamp in enumerate(stamps):
        if stamp > highest:
            keep[i] = True
            highest = stamp
    return keep


def read_calib_file(path: Path) -> dict[str, np.ndarray]:
    values: dict[str, np.ndarray] = {}
    for line in path.read_text().splitlines():
        if ":" not in line:
            continue
        key, payload = line.split(":", 1)
        try:
            values[key.strip()] = np.array([float(x) for x in payload.split()])
        except ValueError:
            continue  # non-numeric rows such as calib_time
    return values


def read_timestamps(path: Path) -> np.ndarray:
    """KITTI stamps look like '2011-09-26 13:02:44.332000000' — nanosecond precision.

    Interpreted as UTC so the resulting epoch seconds do not depend on the timezone of
    whichever machine runs the conversion.
    """
    stamps = []
    for line in path.read_text().splitlines():
        line = line.strip()
        if not line:
            continue
        whole = datetime.datetime.strptime(line[:26], "%Y-%m-%d %H:%M:%S.%f").replace(
            tzinfo=datetime.timezone.utc
        )
        trailing_nanos = float(line[26:29] or 0) * 1e-9
        stamps.append(whole.timestamp() + trailing_nanos)
    return np.array(stamps)


def load_oxts(drive_dir: Path) -> tuple[np.ndarray, np.ndarray]:
    """Return (timestamps, packets) where packets is (N, 30)."""
    files = sorted((drive_dir / "oxts" / "data").glob("*.txt"))
    if not files:
        raise FileNotFoundError(f"no OXTS files under {drive_dir / 'oxts' / 'data'}")
    packets = np.array([[float(x) for x in f.read_text().split()] for f in files])
    stamps = read_timestamps(drive_dir / "oxts" / "timestamps.txt")
    if len(stamps) != len(packets):
        raise ValueError(f"{len(stamps)} OXTS timestamps but {len(packets)} packets")
    return stamps, packets


def compose_extrinsics(calib_dir: Path, rectified: bool) -> np.ndarray:
    """Chain KITTI's three calibration files into T_imu_cam (camera -> IMU).

    KITTI only publishes imu->velo, velo->cam0 and the rectifying rotation, so the
    transform VIO actually needs has to be built by composition and then inverted.
    """

    def homogeneous(rotation: np.ndarray, translation: np.ndarray) -> np.ndarray:
        matrix = np.eye(4)
        matrix[:3, :3] = rotation.reshape(3, 3)
        matrix[:3, 3] = translation
        return matrix

    imu_to_velo = read_calib_file(calib_dir / "calib_imu_to_velo.txt")
    velo_to_cam = read_calib_file(calib_dir / "calib_velo_to_cam.txt")
    cam_to_cam = read_calib_file(calib_dir / "calib_cam_to_cam.txt")

    T_cam_imu = homogeneous(velo_to_cam["R"], velo_to_cam["T"]) @ homogeneous(
        imu_to_velo["R"], imu_to_velo["T"]
    )
    if rectified:
        rectify = np.eye(4)
        rectify[:3, :3] = cam_to_cam["R_rect_00"].reshape(3, 3)
        T_cam_imu = rectify @ T_cam_imu
    return np.linalg.inv(T_cam_imu)


def load_calibration(calib_dir: Path, rectified: bool) -> Calibration:
    cam_to_cam = read_calib_file(calib_dir / "calib_cam_to_cam.txt")
    if rectified:
        # P_rect_00 already encodes the rectified pinhole; distortion is zero by construction.
        projection = cam_to_cam["P_rect_00"].reshape(3, 4)
        fx, fy, cx, cy = projection[0, 0], projection[1, 1], projection[0, 2], projection[1, 2]
        width, height = cam_to_cam["S_rect_00"]
    else:
        intrinsics = cam_to_cam["K_00"]
        fx, fy, cx, cy = intrinsics[0], intrinsics[4], intrinsics[2], intrinsics[5]
        width, height = cam_to_cam["S_00"]
    return Calibration(
        fx=float(fx), fy=float(fy), cx=float(cx), cy=float(cy),
        width=int(width), height=int(height),
        T_imu_cam=compose_extrinsics(calib_dir, rectified),
    )


# --------------------------------------------------------------------------------------
# Conversion
# --------------------------------------------------------------------------------------


def oxts_to_imu_csv(stamps: np.ndarray, packets: np.ndarray, destination: Path) -> int:
    accel = packets[:, [OXTS_COLUMNS.index(name) for name in ACCEL_FIELDS]]
    gyro = packets[:, [OXTS_COLUMNS.index(name) for name in GYRO_FIELDS]]

    rows = (
        f"{t:.9f},{a[0]:.9f},{a[1]:.9f},{a[2]:.9f},{w[0]:.9f},{w[1]:.9f},{w[2]:.9f}"
        for t, a, w in zip(stamps, accel, gyro)
    )
    destination.write_text("stamp,ax,ay,az,gx,gy,gz\n" + "\n".join(rows) + "\n")
    return len(stamps)


def load_image_index(
    drive_dir: Path, camera: str, frames: tuple[int, int] | None = None
) -> tuple[np.ndarray, list[str]]:
    """Image stamps and filenames, optionally cut to an inclusive frame-index range.

    Frame indices are the `_sync` numbering, which is also what the KITTI odometry
    benchmark's start/end frames refer to.
    """
    stamps = read_timestamps(drive_dir / camera / "timestamps.txt")
    names = sorted(p.name for p in (drive_dir / camera / "data").glob("*.png"))
    if len(stamps) != len(names):
        raise ValueError(f"{len(stamps)} timestamps but {len(names)} images in {camera}")
    if frames is not None:
        first, last = frames
        if not 0 <= first <= last < len(names):
            raise ValueError(f"frame range {first}..{last} outside 0..{len(names) - 1}")
        stamps, names = stamps[first : last + 1], names[first : last + 1]
    return stamps, names


def images_to_csv(stamps: np.ndarray, names: Sequence[str], destination: Path) -> int:
    rows = (f"{t:.9f},{name}" for t, name in zip(stamps, names))
    destination.write_text("stamp,filename\n" + "\n".join(rows) + "\n")
    return len(names)


def oxts_to_tum(stamps: np.ndarray, packets: np.ndarray, destination: Path) -> int:
    """Ground truth from the OXTS GPS/INS fix, as a local ENU trajectory.

    KITTI's own convention: a Mercator projection anchored at the first latitude. Good to
    GPS accuracy (decimetres here), which is enough to score a 7 s drive but is not the
    centimetre-grade truth EuRoC provides.
    """
    from scipy.spatial.transform import Rotation

    lat = packets[:, OXTS_COLUMNS.index("lat")]
    lon = packets[:, OXTS_COLUMNS.index("lon")]
    alt = packets[:, OXTS_COLUMNS.index("alt")]
    rpy = packets[:, [OXTS_COLUMNS.index(n) for n in ("roll", "pitch", "yaw")]]

    scale = np.cos(np.radians(lat[0]))
    east = scale * EARTH_RADIUS_M * np.radians(lon)
    north = scale * EARTH_RADIUS_M * np.log(np.tan(np.radians(45 + lat / 2)))
    position = np.column_stack([east - east[0], north - north[0], alt - alt[0]])
    quat = Rotation.from_euler("xyz", rpy).as_quat()  # (x, y, z, w)

    rows = (
        " ".join([f"{t:.9f}", *(f"{v:.9f}" for v in p), *(f"{v:.9f}" for v in q)])
        for t, p, q in zip(stamps, position, quat)
    )
    destination.write_text("\n".join(rows) + "\n")
    return len(stamps)


def format_config_block(calib: Calibration, image_dir: Path) -> str:
    rotation = "\n".join(
        "          " + ", ".join(f"{v:.9f}" for v in row) + ","
        for row in calib.T_imu_cam[:3, :3]
    ).rstrip(",")
    translation = ", ".join(f"{v:.9f}" for v in calib.T_imu_cam[:3, 3])
    return f"""
# ---- paste into your VINS-Mono config ----
image_width: {calib.width}
image_height: {calib.height}
model_type: PINHOLE
camera_name: camera
distortion_parameters:
   k1: 0.0
   k2: 0.0
   p1: 0.0
   p2: 0.0
projection_parameters:
   fx: {calib.fx:.9f}
   fy: {calib.fy:.9f}
   cx: {calib.cx:.9f}
   cy: {calib.cy:.9f}
estimate_extrinsic: 0
extrinsicRotation: !!opencv-matrix
   rows: 3
   cols: 3
   dt: d
   data: [
{rotation}]
extrinsicTranslation: !!opencv-matrix
   rows: 3
   cols: 1
   dt: d
   data: [{translation}]
# images: {image_dir}
"""


# --------------------------------------------------------------------------------------
# CLI
# --------------------------------------------------------------------------------------


def parse_args(argv: Sequence[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    parser.add_argument(
        "root", type=Path, help="directory holding the *_sync / *_extract dirs and calib_*.txt"
    )
    parser.add_argument(
        "--date", default=None, help="e.g. 2011_09_26 (inferred from the drive folders when omitted)"
    )
    parser.add_argument("--drive", required=True, help="e.g. 0002")
    parser.add_argument("-o", "--output", type=Path, required=True, help="output directory")
    parser.add_argument("--camera", default="image_00", help="image_00/01 grayscale, 02/03 colour")
    parser.add_argument(
        "--images-from",
        choices=["sync", "extract"],
        default="sync",
        help="sync = rectified and distortion-free (default); extract = raw, needs distortion",
    )
    parser.add_argument(
        "--frames",
        type=int,
        nargs=2,
        metavar=("FIRST", "LAST"),
        default=None,
        help="keep only this inclusive range of image indices (e.g. an odometry sequence)",
    )
    parser.add_argument(
        "--imu-margin",
        type=float,
        default=1.0,
        help="with --frames, seconds of IMU/ground truth kept on either side of the images",
    )
    return parser.parse_args(argv)


DATE_PATTERN = re.compile(r"(\d{4}_\d{2}_\d{2})_drive_\d+_(?:sync|extract)$")


def infer_date(root: Path) -> str:
    """Recover the capture date from the drive folder names.

    The date cannot be assumed to be the parent folder's name: people rename the parent
    after download (to the odometry sequence number, say), but the drive folders keep
    KITTI's own naming.
    """
    dates = {m.group(1) for m in (DATE_PATTERN.match(p.name) for p in root.iterdir() if p.is_dir()) if m}
    if len(dates) != 1:
        raise ValueError(
            f"could not infer the capture date from {root} (found {sorted(dates) or 'nothing'}); "
            "pass --date explicitly"
        )
    return dates.pop()


def main(argv: Sequence[str] | None = None) -> int:
    args = parse_args(argv)
    date = args.date or infer_date(args.root)
    sync_dir = args.root / f"{date}_drive_{args.drive}_sync"
    extract_dir = args.root / f"{date}_drive_{args.drive}_extract"
    image_dir = sync_dir if args.images_from == "sync" else extract_dir

    if not extract_dir.is_dir():
        raise FileNotFoundError(
            f"{extract_dir} not found. The 100 Hz IMU only exists in the *_extract archive; "
            "the *_sync one is downsampled to the 10 Hz camera rate and is unusable for VIO."
        )
    if not image_dir.is_dir():
        raise FileNotFoundError(f"{image_dir} not found")

    args.output.mkdir(parents=True, exist_ok=True)

    # IMU always comes from _extract: that is the only place the 100 Hz stream survives.
    imu_stamps, packets = load_oxts(extract_dir)
    monotonic = strictly_increasing(imu_stamps)
    dropped = int((~monotonic).sum())
    if dropped:
        print(
            f"NOTE: dropped {dropped} OXTS samples whose timestamps ran backwards "
            "(KITTI clock-resync glitch); IMU and ground truth stay on one monotonic clock."
        )
    imu_stamps, packets = imu_stamps[monotonic], packets[monotonic]

    frames = tuple(args.frames) if args.frames else None
    image_stamps, image_names = load_image_index(image_dir, args.camera, frames)
    if frames is not None:
        # The IMU keeps a margin so it still brackets the first and last frame.
        window = (imu_stamps >= image_stamps[0] - args.imu_margin) & (
            imu_stamps <= image_stamps[-1] + args.imu_margin
        )
        imu_stamps, packets = imu_stamps[window], packets[window]
        print(f"NOTE: frames {frames[0]}..{frames[1]} only; IMU cut to a ±{args.imu_margin:g} s margin.")

    n_imu = oxts_to_imu_csv(imu_stamps, packets, args.output / "imu.csv")
    n_img = images_to_csv(image_stamps, image_names, args.output / "images.csv")
    n_gt = oxts_to_tum(imu_stamps, packets, args.output / "gt_tum.txt")

    rectified = args.images_from == "sync"
    calib = load_calibration(args.root, rectified)

    imu_gaps = np.diff(imu_stamps)
    if imu_gaps.size and imu_gaps.max() > 0.05:
        print(
            f"WARNING: {(imu_gaps > 0.05).sum()} IMU gaps over 50 ms, largest {imu_gaps.max():.3f} s "
            f"at t={imu_stamps[imu_gaps.argmax()]:.3f} (OXTS packet loss in _extract)"
        )
    print(f"imu    : {n_imu:5d} samples @ {1 / np.median(np.diff(imu_stamps)):6.2f} Hz  -> imu.csv")
    print(f"images : {n_img:5d} frames  @ {1 / np.median(np.diff(image_stamps)):6.2f} Hz  -> images.csv")
    print(f"gt     : {n_gt:5d} poses                 -> gt_tum.txt  (GPS/INS, decimetre-grade)")
    print(f"span   : imu {imu_stamps[0]:.3f}..{imu_stamps[-1]:.3f}  images {image_stamps[0]:.3f}..{image_stamps[-1]:.3f}")
    print(f"         IMU brackets images: start={imu_stamps[0] < image_stamps[0]} end={imu_stamps[-1] > image_stamps[-1]}")

    if not rectified:
        distortion = read_calib_file(args.root / "calib_cam_to_cam.txt")["D_00"]
        print(
            f"\nWARNING: raw images carry 5 distortion coefficients {distortion.round(5)};\n"
            "         VINS-Mono's PINHOLE model has only k1,k2,p1,p2 and silently drops k3."
        )

    print(format_config_block(calib, image_dir / args.camera / "data"))
    print("Next:")
    print(f"  python3 make_rosbag.py {args.output}/kitti.bag \\")
    print(f"      --images {image_dir / args.camera / 'data'} \\")
    print(f"      --image-csv {args.output}/images.csv --image-csv-columns stamp,filename \\")
    print(f"      --imu-csv {args.output}/imu.csv")
    return 0


if __name__ == "__main__":
    sys.exit(main())
