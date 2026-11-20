#!/usr/bin/env python3
"""Convert extracted M3ED sequences (see extract_ovc.py) into ROS1 bags.

For every <input_dir>/<seq>/ this writes into <output_dir>/<seq>/:
  data.bag    /cam0/image_raw (sensor_msgs/Image, mono8), /imu0 (sensor_msgs/Imu)
  gt.bag      /ground_truth/odometry (nav_msgs/Odometry),
              /ground_truth/path (nav_msgs/Path, one message with the full path)
  calib.yaml  camera intrinsics and IMU-camera extrinsics for the VINS config

M3ED timestamps are microseconds from the start of the recording; TIME_OFFSET
is added to all of them (ROS time 0 is invalid), identically in both bags.
Ground truth is the pose of the IMU in the IMU frame at the trajectory origin.
"""
import argparse
import os
import sys
from concurrent.futures import ProcessPoolExecutor

import h5py
import numpy as np
from rosbags.rosbag1 import Writer
from rosbags.typesys import Stores, get_typestore

TS = get_typestore(Stores.ROS1_NOETIC)
Time = TS.types["builtin_interfaces/msg/Time"]
Header = TS.types["std_msgs/msg/Header"]
Image = TS.types["sensor_msgs/msg/Image"]
Imu = TS.types["sensor_msgs/msg/Imu"]
Vector3 = TS.types["geometry_msgs/msg/Vector3"]
Point = TS.types["geometry_msgs/msg/Point"]
Quaternion = TS.types["geometry_msgs/msg/Quaternion"]
Pose = TS.types["geometry_msgs/msg/Pose"]
PoseStamped = TS.types["geometry_msgs/msg/PoseStamped"]
PoseWithCovariance = TS.types["geometry_msgs/msg/PoseWithCovariance"]
Twist = TS.types["geometry_msgs/msg/Twist"]
TwistWithCovariance = TS.types["geometry_msgs/msg/TwistWithCovariance"]
Odometry = TS.types["nav_msgs/msg/Odometry"]
Path = TS.types["nav_msgs/msg/Path"]

TIME_OFFSET_S = 1_000_000_000
CAM_FRAME = "cam0"
IMU_FRAME = "imu0"
WORLD_FRAME = "world"
FRAME_BATCH = 64


def stamp_ns(t_us, offset_ns):
    return int(t_us) * 1000 + offset_ns


def header(seq, t_ns, frame_id):
    return Header(seq=seq,
                  stamp=Time(sec=t_ns // 10**9, nanosec=t_ns % 10**9),
                  frame_id=frame_id)


def rot_to_quat(R):
    """Rotation matrix -> (x, y, z, w)."""
    t = np.trace(R)
    if t > 0:
        s = np.sqrt(t + 1.0) * 2
        q = [(R[2, 1] - R[1, 2]) / s, (R[0, 2] - R[2, 0]) / s,
             (R[1, 0] - R[0, 1]) / s, 0.25 * s]
    else:
        i = int(np.argmax(np.diag(R)))
        j, k = (i + 1) % 3, (i + 2) % 3
        s = np.sqrt(1.0 + R[i, i] - R[j, j] - R[k, k]) * 2
        q = [0.0, 0.0, 0.0, (R[k, j] - R[j, k]) / s]
        q[i] = 0.25 * s
        q[j] = (R[j, i] + R[i, j]) / s
        q[k] = (R[k, i] + R[i, k]) / s
    q = np.array(q)
    return q / np.linalg.norm(q)


def write_data_bag(h5, out_path, offset_ns):
    cam_ts = h5["ovc/ts"][:]
    frames = h5["ovc/left/data"]
    height, width = frames.shape[1:3]
    imu_ts = h5["ovc/imu/ts"][:]
    accel = h5["ovc/imu/accel"][:]
    omega = h5["ovc/imu/omega"][:]
    # Dropped IMU packets are stored as NaN readings with a garbage timestamp
    ok = (np.isfinite(accel).all(1) & np.isfinite(omega).all(1)
          & (imu_ts >= 0) & (imu_ts <= max(imu_ts[-1], cam_ts[-1])))
    dropped = int((~ok).sum())
    # IMU samples sit on a regular grid (slot = index * period). A few samples
    # carry the timestamp of the next slot; put those back on their own slot.
    slots = np.flatnonzero(ok)
    imu_ts, accel, omega = imu_ts[ok], accel[ok], omega[ok]
    period = int(np.median(np.diff(imu_ts)))
    for j in np.flatnonzero(np.diff(imu_ts) <= 0):
        imu_ts[j] = imu_ts[j + 1] - period * (slots[j + 1] - slots[j])
    if (np.diff(imu_ts) <= 0).any() or (np.diff(cam_ts) <= 0).any():
        raise ValueError("timestamps are not strictly increasing")

    no_orientation = np.array([-1.0] + [0.0] * 8)
    unknown_cov = np.zeros(9)
    identity = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)

    tmp = out_path + ".part"
    if os.path.exists(tmp):
        os.remove(tmp)
    with Writer(tmp) as bag:
        imu_conn = bag.add_connection("/imu0", Imu.__msgtype__, typestore=TS)
        cam_conn = bag.add_connection("/cam0/image_raw", Image.__msgtype__,
                                      typestore=TS)
        i = 0  # next IMU sample to write
        for start in range(0, len(cam_ts), FRAME_BATCH):
            batch = frames[start:start + FRAME_BATCH]
            for k, img in enumerate(batch):
                n = start + k
                # Keep the bag in time order: IMU up to this frame first
                while i < len(imu_ts) and imu_ts[i] <= cam_ts[n]:
                    t = stamp_ns(imu_ts[i], offset_ns)
                    msg = Imu(
                        header=header(i, t, IMU_FRAME),
                        orientation=identity,
                        orientation_covariance=no_orientation,
                        angular_velocity=Vector3(*map(float, omega[i])),
                        angular_velocity_covariance=unknown_cov,
                        linear_acceleration=Vector3(*map(float, accel[i])),
                        linear_acceleration_covariance=unknown_cov)
                    bag.write(imu_conn, t,
                              TS.serialize_ros1(msg, Imu.__msgtype__))
                    i += 1
                t = stamp_ns(cam_ts[n], offset_ns)
                msg = Image(header=header(n, t, CAM_FRAME),
                            height=height, width=width, encoding="mono8",
                            is_bigendian=0, step=width,
                            data=np.ascontiguousarray(img).reshape(-1))
                bag.write(cam_conn, t,
                          TS.serialize_ros1(msg, Image.__msgtype__))
        while i < len(imu_ts):
            t = stamp_ns(imu_ts[i], offset_ns)
            msg = Imu(
                header=header(i, t, IMU_FRAME),
                orientation=identity,
                orientation_covariance=no_orientation,
                angular_velocity=Vector3(*map(float, omega[i])),
                angular_velocity_covariance=unknown_cov,
                linear_acceleration=Vector3(*map(float, accel[i])),
                linear_acceleration_covariance=unknown_cov)
            bag.write(imu_conn, t, TS.serialize_ros1(msg, Imu.__msgtype__))
            i += 1
    os.replace(tmp, out_path)
    return len(cam_ts), len(imu_ts), dropped


def write_gt_bag(h5, gt, out_path, offset_ns):
    gt_ts = gt["ts"][:]
    Cn_T_C0 = gt["Cn_T_C0"][:]
    # Ground truth is for the left event camera; move it to the IMU
    T_c_i = h5["ovc/imu/calib/T_to_prophesee_left"][:]
    T_i_c = np.linalg.inv(T_c_i)
    T_w_i = T_i_c @ np.linalg.inv(Cn_T_C0) @ T_c_i

    zero = Vector3(x=0.0, y=0.0, z=0.0)
    cov = np.zeros(36)
    poses = []
    tmp = out_path + ".part"
    if os.path.exists(tmp):
        os.remove(tmp)
    with Writer(tmp) as bag:
        odom_conn = bag.add_connection("/ground_truth/odometry",
                                       Odometry.__msgtype__, typestore=TS)
        path_conn = bag.add_connection("/ground_truth/path",
                                       Path.__msgtype__, typestore=TS)
        for n, (t_us, T) in enumerate(zip(gt_ts, T_w_i)):
            t = stamp_ns(t_us, offset_ns)
            q = rot_to_quat(T[:3, :3])
            pose = Pose(position=Point(*map(float, T[:3, 3])),
                        orientation=Quaternion(*map(float, q)))
            hdr = header(n, t, WORLD_FRAME)
            poses.append(PoseStamped(header=hdr, pose=pose))
            msg = Odometry(
                header=hdr, child_frame_id=IMU_FRAME,
                pose=PoseWithCovariance(pose=pose, covariance=cov),
                twist=TwistWithCovariance(
                    twist=Twist(linear=zero, angular=zero), covariance=cov))
            bag.write(odom_conn, t,
                      TS.serialize_ros1(msg, Odometry.__msgtype__))
        t0 = stamp_ns(gt_ts[0], offset_ns)
        path = Path(header=header(0, t0, WORLD_FRAME), poses=poses)
        bag.write(path_conn, t0, TS.serialize_ros1(path, Path.__msgtype__))
    os.replace(tmp, out_path)
    return len(gt_ts)


def write_calib(h5, out_path, offset_ns):
    calib = h5["ovc/left/calib"]
    fx, fy, cx, cy = calib["intrinsics"][:]
    k1, k2, p1, p2 = calib["distortion_coeffs"][:]
    width, height = calib["resolution"][:]
    T_imu_cam = (np.linalg.inv(h5["ovc/imu/calib/T_to_prophesee_left"][:])
                 @ calib["T_to_prophesee_left"][:])
    rows = "\n".join("  - [" + ", ".join(f"{v:.9f}" for v in r) + "]"
                     for r in T_imu_cam)
    with open(out_path, "w") as f:
        f.write(
            f"# M3ED left grayscale camera (cam0) + IMU (imu0)\n"
            f"image_topic: /cam0/image_raw\n"
            f"imu_topic: /imu0\n"
            f"camera_model: {calib['camera_model'][()].decode()}\n"
            f"distortion_model: {calib['distortion_model'][()].decode()}\n"
            f"image_width: {width}\n"
            f"image_height: {height}\n"
            f"intrinsics: {{fx: {fx:.6f}, fy: {fy:.6f}, "
            f"cx: {cx:.6f}, cy: {cy:.6f}}}\n"
            f"distortion: {{k1: {k1:.9f}, k2: {k2:.9f}, "
            f"p1: {p1:.9f}, p2: {p2:.9f}}}\n"
            f"# Transform from cam0 to imu0 (VINS body_T_cam0)\n"
            f"T_imu_cam:\n{rows}\n"
            f"# Seconds added to the M3ED timestamps in data.bag and gt.bag\n"
            f"time_offset_s: {offset_ns // 10**9}\n")


def convert(seq, input_dir, output_dir, offset_ns):
    src = os.path.join(input_dir, seq)
    dst = os.path.join(output_dir, seq)
    os.makedirs(dst, exist_ok=True)
    data_bag = os.path.join(dst, "data.bag")
    gt_bag = os.path.join(dst, "gt.bag")
    out = [seq]
    with h5py.File(os.path.join(src, f"{seq}_data.h5"), "r") as h5:
        write_calib(h5, os.path.join(dst, "calib.yaml"), offset_ns)
        pose_path = os.path.join(src, f"{seq}_pose_gt.h5")
        if os.path.exists(gt_bag):
            out.append("gt.bag exists")
        elif not os.path.exists(pose_path):
            out.append("no pose_gt, gt.bag skipped")
        else:
            with h5py.File(pose_path, "r") as gt:
                out.append(f"gt.bag {write_gt_bag(h5, gt, gt_bag, offset_ns)} poses")
        if os.path.exists(data_bag):
            out.append("data.bag exists")
        else:
            n_img, n_imu, dropped = write_data_bag(h5, data_bag, offset_ns)
            out.append(f"data.bag {n_img} images, {n_imu} imu "
                       f"({dropped} dropped imu samples skipped), "
                       f"{os.path.getsize(data_bag) / 1e9:.1f} GB")
    return ": ".join([out[0], ", ".join(out[1:])])


if __name__ == "__main__":
    p = argparse.ArgumentParser()
    p.add_argument("--input_dir", default="data_car")
    p.add_argument("--output_dir", default="vins")
    p.add_argument("--sequences", nargs="+",
                   help="Default: every sequence in input_dir")
    p.add_argument("--time_offset", type=int, default=TIME_OFFSET_S,
                   help="Seconds added to all timestamps")
    p.add_argument("--jobs", type=int, default=3)
    args = p.parse_args()

    seqs = args.sequences or sorted(
        d for d in os.listdir(args.input_dir)
        if os.path.exists(os.path.join(args.input_dir, d, f"{d}_data.h5")))
    print(f"Converting {len(seqs)} sequences into {args.output_dir}/")
    offset_ns = args.time_offset * 10**9
    failed = []
    with ProcessPoolExecutor(args.jobs) as ex:
        futures = {s: ex.submit(convert, s, args.input_dir, args.output_dir,
                                offset_ns) for s in seqs}
        for s, fut in futures.items():
            try:
                print(fut.result(), flush=True)
            except Exception as e:
                print(f"{s}: FAILED: {e!r}", flush=True)
                failed.append(s)
    if failed:
        sys.exit(f"Failed: {', '.join(failed)}")
    print("Done.")
