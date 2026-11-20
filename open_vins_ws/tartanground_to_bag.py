#!/usr/bin/env python3
"""Convert TartanGround diff-drive trajectories (lcam_front + imu) to ROS1 bags for OpenVINS.

    python3 tartanground_to_bag.py [--bag-dir DIR] [Env_P1000 ...]   (default: every downloaded trajectory)
    python3 tartanground_to_bag.py --list                            (print the trajectory names)

Reads  /media/efe/Data/datasets/VIO/TartanGround/data/<Env>/Data_diff/<P>/
Writes /media/efe/Data/datasets/VIO/TartanGround/vins/<Env>_<P>/
    data.bag     /cam0/image_raw (sensor_msgs/Image mono8, 10 Hz), /imu0 (sensor_msgs/Imu, 100 Hz)
                 (or DIR/<Env>_<P>.bag with --bag-dir; run_tartanground.sh writes it to the SSD and
                 deletes it after the run: ~0.85 GB per 200 s, 100 GB for all trajectories)
    gt_tum.txt   pose_lcam_front.txt at the camera timestamps (TUM: t x y z qx qy qz qw)

TartanGround conventions (checked against the data): the IMU body frame is the lcam_front body
frame in NED (x forward, y right, z down), so the IMU sits at the camera; gyro is in the body
frame, acc.txt is the specific force in the body frame (gravity 9.8 along +z down).
The ground truth pose of lcam_front is therefore the IMU pose OpenVINS estimates.
Timestamps start at 0; T0 is added so no ROS time is 0. The last image has no IMU after it
and is dropped.
"""
import argparse
from pathlib import Path

import cv2
import numpy as np
from rosbags.rosbag1 import Writer
from rosbags.typesys import Stores, get_typestore

DATA = Path("/media/efe/Data/datasets/VIO/TartanGround")
T0 = 1000.0  # s
ts = get_typestore(Stores.ROS1_NOETIC)
Image, Imu, Header = ts.types["sensor_msgs/msg/Image"], ts.types["sensor_msgs/msg/Imu"], ts.types["std_msgs/msg/Header"]
Time, Quat, Vec3 = ts.types["builtin_interfaces/msg/Time"], ts.types["geometry_msgs/msg/Quaternion"], ts.types["geometry_msgs/msg/Vector3"]
ZERO9 = np.zeros(9)


def sequences():
    return sorted(f"{p.parents[1].name}_{p.name}" for p in (DATA / "data").glob("*/Data_diff/P*")
                  if (p / "imu" / "imu_time.txt").exists() and (p / "image_lcam_front").is_dir())


def source(seq):
    env, traj = seq.rsplit("_", 1)
    return DATA / "data" / env / "Data_diff" / traj


def stamp(t):
    ns = int(round((T0 + t) * 1e9))
    return Time(sec=ns // 10**9, nanosec=ns % 10**9), ns


def convert(seq, bag_dir=None):
    src, out = source(seq), DATA / "vins" / seq
    out.mkdir(parents=True, exist_ok=True)
    bag = Path(bag_dir) / f"{seq}.bag" if bag_dir else out / "data.bag"
    t_imu = np.loadtxt(src / "imu" / "imu_time.txt")
    acc, gyro = np.loadtxt(src / "imu" / "acc.txt"), np.loadtxt(src / "imu" / "gyro.txt")
    t_cam = np.loadtxt(src / "imu" / "cam_time.txt")
    pose = np.loadtxt(src / "pose_lcam_front.txt")
    images = sorted((src / "image_lcam_front").glob("*_lcam_front.png"))
    n = min(len(t_cam), len(pose), len(images))
    keep = np.flatnonzero(t_cam[:n] <= t_imu[-1] + 1e-6)
    np.savetxt(out / "gt_tum.txt", np.column_stack([T0 + t_cam[keep], pose[keep]]), fmt="%.9f")

    bag.unlink(missing_ok=True)
    with Writer(bag) as w:
        c_imu = w.add_connection("/imu0", Imu.__msgtype__, typestore=ts)
        c_cam = w.add_connection("/cam0/image_raw", Image.__msgtype__, typestore=ts)
        events = [(t, 0, i) for i, t in enumerate(t_imu)] + [(t_cam[i], 1, i) for i in keep]
        for seq_nr, (t, kind, i) in enumerate(sorted(events)):
            st, ns = stamp(t)
            hdr = Header(seq=seq_nr, stamp=st, frame_id="imu0" if kind == 0 else "cam0")
            if kind == 0:
                msg = Imu(header=hdr, orientation=Quat(x=0.0, y=0.0, z=0.0, w=1.0), orientation_covariance=ZERO9,
                          angular_velocity=Vec3(x=gyro[i, 0], y=gyro[i, 1], z=gyro[i, 2]), angular_velocity_covariance=ZERO9,
                          linear_acceleration=Vec3(x=acc[i, 0], y=acc[i, 1], z=acc[i, 2]), linear_acceleration_covariance=ZERO9)
                w.write(c_imu, ns, ts.serialize_ros1(msg, Imu.__msgtype__))
            else:
                img = cv2.imread(str(images[i]), cv2.IMREAD_GRAYSCALE)
                msg = Image(header=hdr, height=img.shape[0], width=img.shape[1], encoding="mono8", is_bigendian=0,
                            step=img.shape[1], data=img.reshape(-1))
                w.write(c_cam, ns, ts.serialize_ros1(msg, Image.__msgtype__))
    print(f"{seq}: {len(keep)} images, {len(t_imu)} imu -> {bag}", flush=True)


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("sequences", nargs="*")
    parser.add_argument("--bag-dir")
    parser.add_argument("--list", action="store_true")
    args = parser.parse_args()
    if args.list:
        print("\n".join(sequences()))
    else:
        for s in args.sequences or sequences():
            convert(s, args.bag_dir)
