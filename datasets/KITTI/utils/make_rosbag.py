#!/usr/bin/env python3
"""Build a ROS1 bag from any dataset of sequential images plus an IMU CSV.

VINS-Mono and OpenVINS both consume `sensor_msgs/Image` + `sensor_msgs/Imu`; this turns a
plain folder of frames and a CSV of inertial samples into exactly that, with no ROS
installation required (uses the pure-Python `rosbags` package).

Timestamps may come from the image filenames or from a separate CSV, in seconds or
nanoseconds, which covers the usual layouts (EuRoC, TUM, Isaac Sim, ad-hoc dumps).

    python3 make_rosbag.py out.bag --images frames/ --imu-csv imu.csv

Run with --help for the full option list, or import `build_bag` to drive it from Python.
"""

from __future__ import annotations

import argparse
import heapq
import re
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Callable, Iterable, Protocol, Sequence, TypeVar

import cv2
import numpy as np
import pandas as pd

NANOSECONDS_PER_SECOND = 1_000_000_000

# Multiplier that converts a stamp in the given unit into seconds.
TIME_UNIT_TO_SECONDS = {"s": 1.0, "ms": 1e-3, "us": 1e-6, "ns": 1e-9}

IMAGE_SUFFIXES = (".png", ".jpg", ".jpeg", ".bmp", ".tif", ".tiff")


# --------------------------------------------------------------------------------------
# Domain types
# --------------------------------------------------------------------------------------


@dataclass(frozen=True)
class ImuSample:
    """One inertial measurement, already in the IMU/body frame and in SI units."""

    stamp: float  # seconds
    accel: np.ndarray  # (3,) m/s^2 specific force — reads +g on the up axis at rest
    gyro: np.ndarray  # (3,) rad/s


@dataclass(frozen=True)
class FrameRef:
    """An image on disk. Pixels stay unread until the writer asks for them."""

    stamp: float  # seconds
    path: Path


@dataclass(frozen=True)
class EncodedImage:
    height: int
    width: int
    encoding: str
    data: bytes


class MessageSink(Protocol):
    """Destination for the merged stream, so `build_bag` never sees a ROS type."""

    def write_imu(self, sample: ImuSample) -> None: ...

    def write_image(self, stamp: float, image: EncodedImage) -> None: ...


# --------------------------------------------------------------------------------------
# Loading — pure: paths and options in, records out
# --------------------------------------------------------------------------------------


def load_imu(
    csv_path: Path,
    columns: Sequence[str],
    time_unit: str = "s",
    gyro_first: bool = False,
) -> list[ImuSample]:
    """Read an IMU CSV.

    `columns` names the seven columns in the order
    `stamp, ax, ay, az, gx, gy, gz`. EuRoC stores gyro before accel, which `gyro_first`
    handles without having to restate the column names.
    """
    if len(columns) != 7:
        raise ValueError(f"expected 7 column names (stamp + 6 axes), got {len(columns)}")

    frame = pd.read_csv(csv_path, comment="#")
    missing = set(columns) - set(frame.columns)
    if missing:
        raise ValueError(
            f"{csv_path} is missing columns {sorted(missing)}; it has {list(frame.columns)}"
        )

    scale = TIME_UNIT_TO_SECONDS[time_unit]
    stamps = frame[columns[0]].to_numpy(dtype=float) * scale
    first = frame[list(columns[1:4])].to_numpy(dtype=float)
    second = frame[list(columns[4:7])].to_numpy(dtype=float)
    accels, gyros = (second, first) if gyro_first else (first, second)

    return [
        ImuSample(stamp=float(stamp), accel=accel, gyro=gyro)
        for stamp, accel, gyro in zip(stamps, accels, gyros)
    ]


def load_frames_from_filenames(images_dir: Path, pattern: str, time_unit: str = "s") -> list[FrameRef]:
    """Take each image's timestamp from its own filename via a regex with one group."""
    regex = re.compile(pattern)
    scale = TIME_UNIT_TO_SECONDS[time_unit]

    def parsed(path: Path) -> FrameRef | None:
        match = regex.search(path.name)
        return FrameRef(stamp=float(match.group(1)) * scale, path=path) if match else None

    candidates = sorted(p for p in images_dir.iterdir() if p.suffix.lower() in IMAGE_SUFFIXES)
    refs = [ref for ref in map(parsed, candidates) if ref is not None]
    if not refs:
        raise ValueError(
            f"no filenames under {images_dir} matched {pattern!r} "
            f"(found {len(candidates)} image files)"
        )
    return sorted(refs, key=lambda ref: ref.stamp)


def load_frames_from_csv(
    images_dir: Path, csv_path: Path, columns: Sequence[str], time_unit: str = "s"
) -> list[FrameRef]:
    """Take timestamps from a CSV of `stamp, filename` pairs (the EuRoC layout)."""
    frame = pd.read_csv(csv_path, comment="#")
    stamp_column, name_column = columns
    scale = TIME_UNIT_TO_SECONDS[time_unit]

    refs = [
        FrameRef(stamp=float(stamp) * scale, path=images_dir / str(name).strip())
        for stamp, name in zip(frame[stamp_column], frame[name_column])
    ]
    missing = [ref.path for ref in refs if not ref.path.exists()]
    if missing:
        raise ValueError(f"{len(missing)} listed images are absent, e.g. {missing[0]}")
    return sorted(refs, key=lambda ref: ref.stamp)


T = TypeVar("T")


def drop_non_increasing(records: Sequence[T], stamp_of: Callable[[T], float]) -> tuple[list[T], int]:
    """Keep only strictly-increasing timestamps.

    Duplicate or backwards stamps make VINS-Mono's measurement sync throw away data and
    OpenVINS reject the stream outright, so they are removed here rather than debugged later.
    """
    kept: list[T] = []
    for record in records:
        if not kept or stamp_of(record) > stamp_of(kept[-1]):
            kept.append(record)
    return kept, len(records) - len(kept)


# --------------------------------------------------------------------------------------
# Image decoding — the one unavoidable I/O step, isolated
# --------------------------------------------------------------------------------------


def read_image(path: Path, encoding: str) -> EncodedImage:
    """Decode one frame. `mono8` is what both front-ends ultimately use."""
    flag = cv2.IMREAD_GRAYSCALE if encoding == "mono8" else cv2.IMREAD_COLOR
    image = cv2.imread(str(path), flag)
    if image is None:
        raise ValueError(f"could not decode {path}")
    return EncodedImage(
        height=image.shape[0],
        width=image.shape[1],
        encoding=encoding,
        data=image.tobytes(),
    )


# --------------------------------------------------------------------------------------
# ROS1 bag sink
# --------------------------------------------------------------------------------------


class Ros1BagSink:
    """Writes the merged stream to a ROS1 bag. Owns every bag-format detail."""

    def __init__(
        self,
        path: Path,
        imu_topic: str,
        image_topic: str,
        imu_frame: str = "body",
        camera_frame: str = "cam0",
    ):
        from rosbags.rosbag1 import Writer
        from rosbags.typesys import Stores, get_typestore

        self._typestore = get_typestore(Stores.ROS1_NOETIC)
        types = self._typestore.types
        self._Image = types["sensor_msgs/msg/Image"]
        self._Imu = types["sensor_msgs/msg/Imu"]
        self._Header = types["std_msgs/msg/Header"]
        self._Time = types["builtin_interfaces/msg/Time"]
        self._Vector3 = types["geometry_msgs/msg/Vector3"]
        self._Quaternion = types["geometry_msgs/msg/Quaternion"]

        self._writer = Writer(path)
        self._writer.open()
        self._imu_conn = self._writer.add_connection(
            imu_topic, self._Imu.__msgtype__, typestore=self._typestore
        )
        self._image_conn = self._writer.add_connection(
            image_topic, self._Image.__msgtype__, typestore=self._typestore
        )
        self._imu_frame = imu_frame
        self._camera_frame = camera_frame
        self._counts = {"imu": 0, "image": 0}

    def __enter__(self) -> "Ros1BagSink":
        return self

    def __exit__(self, *_exc_info: object) -> None:
        self._writer.close()

    @property
    def counts(self) -> dict[str, int]:
        return dict(self._counts)

    def _header(self, stamp: float, frame_id: str):
        nanoseconds = int(round(stamp * NANOSECONDS_PER_SECOND))
        return self._Header(
            # ROS1 headers carry a sequence counter; playback renumbers it anyway.
            seq=0,
            stamp=self._Time(
                sec=nanoseconds // NANOSECONDS_PER_SECOND,
                nanosec=nanoseconds % NANOSECONDS_PER_SECOND,
            ),
            frame_id=frame_id,
        )

    def _emit(self, connection, stamp: float, message, msgtype: str) -> None:
        self._writer.write(
            connection,
            int(round(stamp * NANOSECONDS_PER_SECOND)),
            self._typestore.serialize_ros1(message, msgtype),
        )

    def write_imu(self, sample: ImuSample) -> None:
        no_covariance = np.zeros(9, dtype=np.float64)
        # REP 145: a leading -1 marks orientation as not supplied.
        unknown_orientation = np.array([-1.0] + [0.0] * 8, dtype=np.float64)

        message = self._Imu(
            header=self._header(sample.stamp, self._imu_frame),
            orientation=self._Quaternion(x=0.0, y=0.0, z=0.0, w=1.0),
            orientation_covariance=unknown_orientation,
            angular_velocity=self._Vector3(*map(float, sample.gyro)),
            angular_velocity_covariance=no_covariance,
            linear_acceleration=self._Vector3(*map(float, sample.accel)),
            linear_acceleration_covariance=no_covariance.copy(),
        )
        self._emit(self._imu_conn, sample.stamp, message, self._Imu.__msgtype__)
        self._counts["imu"] += 1

    def write_image(self, stamp: float, image: EncodedImage) -> None:
        channels = 1 if image.encoding == "mono8" else 3
        message = self._Image(
            header=self._header(stamp, self._camera_frame),
            height=image.height,
            width=image.width,
            encoding=image.encoding,
            is_bigendian=0,
            step=image.width * channels,
            data=np.frombuffer(image.data, dtype=np.uint8),
        )
        self._emit(self._image_conn, stamp, message, self._Image.__msgtype__)
        self._counts["image"] += 1


# --------------------------------------------------------------------------------------
# Merging and writing — orchestration only
# --------------------------------------------------------------------------------------


Measurement = ImuSample | FrameRef


def merged_stream(imu: Sequence[ImuSample], frames: Sequence[FrameRef]) -> Iterable[Measurement]:
    """Interleave both sensors by timestamp, as a live recording would have landed."""

    # On equal stamps IMU sorts first, so the estimator always holds inertial data
    # spanning the frame it is about to process.
    def ordering(record: Measurement) -> tuple[float, int]:
        return (record.stamp, 0 if isinstance(record, ImuSample) else 1)

    return heapq.merge(imu, frames, key=ordering)


def build_bag(
    imu: Sequence[ImuSample],
    frames: Sequence[FrameRef],
    sink: MessageSink,
    encoding: str = "mono8",
    progress: Callable[[int, int], None] | None = None,
) -> None:
    written = 0
    for record in merged_stream(imu, frames):
        if isinstance(record, ImuSample):
            sink.write_imu(record)
            continue
        sink.write_image(record.stamp, read_image(record.path, encoding))
        written += 1
        if progress is not None:
            progress(written, len(frames))


def report_coverage(imu: Sequence[ImuSample], frames: Sequence[FrameRef]) -> list[str]:
    """Flag the conditions that silently ruin a VIO run, before anyone waits on a result."""
    notes: list[str] = []
    if not imu or not frames:
        return ["one of the two streams is empty"]

    if imu[0].stamp > frames[0].stamp:
        notes.append(
            f"IMU starts {imu[0].stamp - frames[0].stamp:.3f} s AFTER the first image — "
            "leading frames will be discarded by the estimator"
        )
    if imu[-1].stamp < frames[-1].stamp:
        notes.append(
            f"IMU ends {frames[-1].stamp - imu[-1].stamp:.3f} s BEFORE the last image — "
            "trailing frames will be discarded"
        )

    imu_rate = 1.0 / np.median(np.diff([s.stamp for s in imu]))
    image_rate = 1.0 / np.median(np.diff([f.stamp for f in frames]))
    if imu_rate < 100:
        notes.append(f"IMU rate is {imu_rate:.1f} Hz; VIO normally wants 100-200 Hz")
    if image_rate < 10:
        notes.append(f"image rate is {image_rate:.2f} Hz; VINS-Mono's README asks for >= 10 Hz")

    gaps = np.diff([f.stamp for f in frames])
    if gaps.size and gaps.max() > 1.0:
        notes.append(
            f"largest image gap is {gaps.max():.2f} s; a gap over 1.0 s resets the feature "
            "tracker and restarts the estimator"
        )
    return notes


# --------------------------------------------------------------------------------------
# CLI
# --------------------------------------------------------------------------------------


def parse_args(argv: Sequence[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    parser.add_argument("output", type=Path, help="destination .bag")

    parser.add_argument("--images", type=Path, required=True, help="directory of image files")
    parser.add_argument(
        "--image-csv",
        type=Path,
        default=None,
        help="CSV of 'stamp,filename'; omit to read timestamps from the filenames",
    )
    parser.add_argument(
        "--image-csv-columns",
        default="#timestamp [ns],filename",
        help="stamp and filename column names in --image-csv",
    )
    parser.add_argument(
        "--image-pattern",
        default=r"(\d+\.?\d*)",
        help="regex with one capture group giving the stamp in the filename "
        r"(default grabs the first number; Isaac-style files need '_(\d+\.\d+)\.png$')",
    )
    parser.add_argument(
        "--image-time-unit", choices=sorted(TIME_UNIT_TO_SECONDS), default="s"
    )

    parser.add_argument("--imu-csv", type=Path, required=True)
    parser.add_argument(
        "--imu-columns",
        default="stamp,ax,ay,az,gx,gy,gz",
        help="seven column names in the order stamp,ax,ay,az,gx,gy,gz",
    )
    parser.add_argument(
        "--imu-gyro-first",
        action="store_true",
        help="the CSV stores gyro before accel (EuRoC does)",
    )
    parser.add_argument("--imu-time-unit", choices=sorted(TIME_UNIT_TO_SECONDS), default="s")

    parser.add_argument("--imu-topic", default="/imu0")
    parser.add_argument("--image-topic", default="/cam0/image_raw")
    parser.add_argument("--encoding", choices=["mono8", "bgr8"], default="mono8")
    parser.add_argument("--start-time", type=float, default=None, help="drop frames before this stamp")
    parser.add_argument("--end-time", type=float, default=None, help="drop frames after this stamp")
    return parser.parse_args(argv)


def main(argv: Sequence[str] | None = None) -> int:
    args = parse_args(argv)

    imu_raw = load_imu(
        args.imu_csv,
        [name.strip() for name in args.imu_columns.split(",")],
        args.imu_time_unit,
        args.imu_gyro_first,
    )
    if args.image_csv is not None:
        frames_raw = load_frames_from_csv(
            args.images,
            args.image_csv,
            [name.strip() for name in args.image_csv_columns.split(",")],
            args.image_time_unit,
        )
    else:
        frames_raw = load_frames_from_filenames(
            args.images, args.image_pattern, args.image_time_unit
        )

    imu, imu_dropped = drop_non_increasing(imu_raw, lambda s: s.stamp)
    frames, frames_dropped = drop_non_increasing(frames_raw, lambda f: f.stamp)
    if args.start_time is not None:
        frames = [f for f in frames if f.stamp >= args.start_time]
    if args.end_time is not None:
        frames = [f for f in frames if f.stamp <= args.end_time]

    print(f"imu    : {len(imu):6d} samples  ({imu_dropped} non-increasing dropped)")
    print(f"images : {len(frames):6d} frames   ({frames_dropped} non-increasing dropped)")
    print(f"imu    : {imu[0].stamp:.6f} -> {imu[-1].stamp:.6f} s")
    print(f"images : {frames[0].stamp:.6f} -> {frames[-1].stamp:.6f} s")

    for note in report_coverage(imu, frames):
        print(f"  WARNING: {note}")

    if args.output.exists():
        args.output.unlink()

    def progress(done: int, total: int) -> None:
        if done % 50 == 0 or done == total:
            print(f"  images {done}/{total}", end="\r", flush=True)

    with Ros1BagSink(args.output, args.imu_topic, args.image_topic) as sink:
        build_bag(imu, frames, sink, args.encoding, progress)
        counts = sink.counts

    print(f"\nwrote {args.output}  ({args.output.stat().st_size / 1e6:.1f} MB)")
    print(f"  {args.imu_topic:20s} {counts['imu']:6d} msgs")
    print(f"  {args.image_topic:20s} {counts['image']:6d} msgs")
    return 0


if __name__ == "__main__":
    sys.exit(main())
