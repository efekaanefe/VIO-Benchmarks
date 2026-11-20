#!/usr/bin/env python3
"""Turn a TUM ground-truth file into a small companion bag you can play alongside the data.

None of our bags carry ground truth, and the shipped rviz config looks for it on
`/benchmark_publisher/path` — a topic only the EuRoC-specific `benchmark_publisher` node
produces. So there is nothing to see in rviz on KITTI or Isaac.

Rather than rebuild a multi-gigabyte data bag just to add a few hundred poses, this writes
ground truth to its own bag. `rosbag play data.bag gt.bag` plays both together.

    python3 make_gt_bag.py gt_tum.txt gt.bag

Publishes:
    /ground_truth/path      nav_msgs/Path       cumulative trail, decimated (for rviz)
    /ground_truth/odometry  nav_msgs/Odometry   one per sample, decimated

By default the trajectory is re-anchored so its first pose sits at the origin with zero
yaw, which is the convention VINS-Mono pins its own world frame to. That makes the two
roughly overlay in rviz; it is a visual aid, not an alignment for metrics.
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path
from typing import Sequence

import numpy as np

NANOSECONDS_PER_SECOND = 1_000_000_000


def load_tum(path: Path) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    data = np.loadtxt(path)
    if data.ndim != 2 or data.shape[1] < 8:
        raise ValueError(f"{path} is not TUM format (t tx ty tz qx qy qz qw)")
    return data[:, 0], data[:, 1:4], data[:, 4:8]


def anchor_to_origin(positions: np.ndarray, quaternions: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """Re-express the trajectory with its first pose at the origin and yaw zero.

    VINS-Mono pins position and yaw at initialisation because both are unobservable, so
    matching that convention is what makes the two trajectories comparable by eye.
    """
    from scipy.spatial.transform import Rotation

    rotations = Rotation.from_quat(quaternions)
    yaw = rotations.as_euler("ZYX")[0, 0]
    undo_yaw = Rotation.from_euler("Z", -yaw)
    return (
        undo_yaw.apply(positions - positions[0]),
        (undo_yaw * rotations).as_quat(),
    )


class GroundTruthBagWriter:
    """Writes the two ground-truth topics. Owns every bag-format detail."""

    def __init__(self, path: Path, path_topic: str, odom_topic: str, frame_id: str = "world"):
        from rosbags.rosbag1 import Writer
        from rosbags.typesys import Stores, get_typestore

        self._ts = get_typestore(Stores.ROS1_NOETIC)
        types = self._ts.types
        self._Path = types["nav_msgs/msg/Path"]
        self._Odometry = types["nav_msgs/msg/Odometry"]
        self._PoseStamped = types["geometry_msgs/msg/PoseStamped"]
        self._Pose = types["geometry_msgs/msg/Pose"]
        self._PoseWithCovariance = types["geometry_msgs/msg/PoseWithCovariance"]
        self._TwistWithCovariance = types["geometry_msgs/msg/TwistWithCovariance"]
        self._Twist = types["geometry_msgs/msg/Twist"]
        self._Point = types["geometry_msgs/msg/Point"]
        self._Vector3 = types["geometry_msgs/msg/Vector3"]
        self._Quaternion = types["geometry_msgs/msg/Quaternion"]
        self._Header = types["std_msgs/msg/Header"]
        self._Time = types["builtin_interfaces/msg/Time"]

        self._writer = Writer(path)
        self._writer.open()
        self._path_conn = self._writer.add_connection(path_topic, self._Path.__msgtype__, typestore=self._ts)
        self._odom_conn = self._writer.add_connection(odom_topic, self._Odometry.__msgtype__, typestore=self._ts)
        self._frame_id = frame_id
        self._trail: list = []
        self.counts = {"path": 0, "odometry": 0}

    def __enter__(self) -> "GroundTruthBagWriter":
        return self

    def __exit__(self, *_exc: object) -> None:
        self._writer.close()

    def _header(self, stamp: float):
        nanos = int(round(stamp * NANOSECONDS_PER_SECOND))
        return self._Header(
            seq=0,
            stamp=self._Time(sec=nanos // NANOSECONDS_PER_SECOND, nanosec=nanos % NANOSECONDS_PER_SECOND),
            frame_id=self._frame_id,
        )

    def _pose(self, position: np.ndarray, quaternion: np.ndarray):
        return self._Pose(
            position=self._Point(*map(float, position)),
            orientation=self._Quaternion(*map(float, quaternion)),  # TUM order is x,y,z,w
        )

    def write(self, stamp: float, position: np.ndarray, quaternion: np.ndarray) -> None:
        header = self._header(stamp)
        pose = self._pose(position, quaternion)

        self._trail.append(self._PoseStamped(header=header, pose=pose))
        bag_time = int(round(stamp * NANOSECONDS_PER_SECOND))

        # The trail is cumulative, so each Path message carries every pose so far.
        self._writer.write(
            self._path_conn,
            bag_time,
            self._ts.serialize_ros1(self._Path(header=header, poses=list(self._trail)), self._Path.__msgtype__),
        )
        self.counts["path"] += 1

        zeros = np.zeros(36, dtype=np.float64)
        odometry = self._Odometry(
            header=header,
            child_frame_id="world",
            pose=self._PoseWithCovariance(pose=pose, covariance=zeros),
            twist=self._TwistWithCovariance(
                twist=self._Twist(linear=self._Vector3(0.0, 0.0, 0.0), angular=self._Vector3(0.0, 0.0, 0.0)),
                covariance=zeros.copy(),
            ),
        )
        self._writer.write(self._odom_conn, bag_time, self._ts.serialize_ros1(odometry, self._Odometry.__msgtype__))
        self.counts["odometry"] += 1


def main(argv: Sequence[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("gt_tum", type=Path, help="ground truth, TUM format")
    parser.add_argument("output", type=Path, help="destination .bag")
    parser.add_argument("--rate", type=float, default=10.0,
                        help="publish rate in Hz; a cumulative Path grows as O(n^2) in bag "
                             "size, so this is decimated (default 10)")
    parser.add_argument("--path-topic", default="/ground_truth/path")
    parser.add_argument("--odom-topic", default="/ground_truth/odometry")
    parser.add_argument("--no-anchor", action="store_true",
                        help="keep ground truth in its own frame instead of re-anchoring "
                             "to origin with zero yaw")
    args = parser.parse_args(argv)

    stamps, positions, quaternions = load_tum(args.gt_tum)
    if not args.no_anchor:
        positions, quaternions = anchor_to_origin(positions, quaternions)

    # Decimate to the requested rate by taking the first sample in each time bucket.
    step = max(1, int(round(1.0 / (args.rate * np.median(np.diff(stamps))))))
    keep = np.arange(0, len(stamps), step)

    if args.output.exists():
        args.output.unlink()
    with GroundTruthBagWriter(args.output, args.path_topic, args.odom_topic) as writer:
        for i in keep:
            writer.write(float(stamps[i]), positions[i], quaternions[i])
        counts = dict(writer.counts)

    print(f"source   : {len(stamps)} poses @ {1 / np.median(np.diff(stamps)):.1f} Hz")
    print(f"written  : {counts['path']} messages @ ~{args.rate:g} Hz (every {step}th pose)")
    print(f"anchored : {'no (own frame)' if args.no_anchor else 'yes (origin, zero yaw)'}")
    print(f"wrote {args.output} ({args.output.stat().st_size / 1e6:.1f} MB)")
    print(f"  {args.path_topic}\n  {args.odom_topic}")
    print(f"\nPlay alongside the data bag:\n  rosbag play <data>.bag {args.output}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
