# Datasets

The raw data stays outside the repository. The scripts in this folder download the data and convert it for the three methods. In this document, `$DATA` is the dataset root. On the original machine, `$DATA` is `/media/efe/Data/datasets` (543 GB).

Dataset preparation does not need ROS or Docker. The Python package `rosbags` writes ROS1 bags.

```bash
pip install -r datasets/requirements.txt
```

## 1. Summary

| Dataset | Subset | Sequences | Length | Disk | Camera (mono) | IMU | Ground truth |
|---|---|---|---|---|---|---|---|
| KITTI raw | Odometry 00-02, 04-10 (full raw drives) | 10 | 22.8 km, 41 min | 21 GB | `image_00` rectified grayscale, 1241x376, 9.6 Hz | OXTS, 100 Hz | OXTS GPS/INS |
| M3ED | Car, train split: 4 forest, 6 urban day, 6 urban night | 16 | 17.4 km, 72 min | 181 GB | OVC left grayscale, 1280x800, 25 Hz | 400 Hz | `pose_gt.h5`, 10 Hz |
| TartanGround | Differential drive, P1000-P1002, 42 environments | 117 | 44.1 km, 7.0 h | 153 GB | `lcam_front`, 640x640, 10 Hz | 100 Hz, no noise | `pose_lcam_front.txt` (NED) |
| TartanAir v2 | 74 environments, easy and hard | 250 | not measured | 173 GB | `lcam_front`, 640x640, 10 Hz | 100 Hz, no noise | `pose_lcam_front.txt` |
| EuRoC MAV | Vicon Room 1: V1_01, V1_02, V1_03 | 3 | not measured | 15 GB | 752x480, 20 Hz | 200 Hz | Vicon |

TartanAir v2 is downloaded but not benchmarked. EuRoC is a sanity check only.

All converted bags use the EuRoC topic names:

| Topic | Type | Content |
|---|---|---|
| `/cam0/image_raw` | `sensor_msgs/Image` | mono8 image |
| `/imu0` | `sensor_msgs/Imu` | accelerometer and gyroscope |

Data flow:

```
download -> raw files -> convert -> $DATA/VIO/<DATASET>/.../vins/   (bags, gt_tum.txt)
                                       |-- VINS-Mono, OpenVINS: read the bags
                                       |-- DPVO: reads the PNG or HDF5 files directly
```

## 2. KITTI raw

Steps:

```bash
mkdir -p $DATA/VIO/KITTI && cd $DATA/VIO/KITTI
bash <repo>/datasets/KITTI/download_kitti.sh       # <drive>_sync.zip and <drive>_extract.zip
wget https://s3.eu-central-1.amazonaws.com/avg-kitti/raw_data/2011_09_30_calib.zip
wget https://s3.eu-central-1.amazonaws.com/avg-kitti/raw_data/2011_10_03_calib.zip
python3 <repo>/datasets/KITTI/utils/kitti_batch.py . --keep-images
```

- `download_kitti.sh` does not download the calibration archives. Download them with the two `wget` commands.
- `kitti_batch.py` extracts only `image_00` from `_sync` and `oxts/` from `_extract`. This is approximately 3 % of each archive.
- For each drive, the script runs `kitti_to_csv.py`, then `make_rosbag.py`, then `make_gt_bag.py`. Keep the four scripts in the same folder.
- The script skips a drive if its bag exists. Thus you can stop it and start it again.
- Use `--keep-images`. DPVO reads the PNG files.
- Other options: `--sequences 00 05`, `--odometry-range` (cut each bag to the odometry frames), `--delete-zips`.

Odometry sequence to raw drive:

| Seq | Drive | Seq | Drive |
|---|---|---|---|
| 00 | 2011_10_03_drive_0027 | 06 | 2011_09_30_drive_0020 |
| 01 | 2011_10_03_drive_0042 | 07 | 2011_09_30_drive_0027 |
| 02 | 2011_10_03_drive_0034 | 08 | 2011_09_30_drive_0028 |
| 04 | 2011_09_30_drive_0016 | 09 | 2011_09_30_drive_0033 |
| 05 | 2011_09_30_drive_0018 | 10 | 2011_09_30_drive_0034 |

Layout for one drive:

```
2011_09_30_drive_0028/                         # seq 08
|-- calib_{cam_to_cam,imu_to_velo,velo_to_cam}.txt
|-- 2011_09_30_drive_0028_sync/image_00/{data/*.png, timestamps.txt}
|-- 2011_09_30_drive_0028_extract/oxts/{data/*.txt, timestamps.txt}
`-- vins/
    |-- 2011_09_30_drive_0028.bag              # /cam0/image_raw, /imu0
    |-- 2011_09_30_drive_0028_gt.bag           # /ground_truth/path, /ground_truth/odometry (RViz only)
    |-- 2011_09_30_drive_0028.txt              # conversion log, VINS-Mono calibration block
    |-- gt_tum.txt                             # t x y z qx qy qz qw, 100 Hz, local ENU
    |-- imu.csv
    `-- images.csv
```

Data problems:

- Sequence 03 (`2011_09_26_drive_0067`) is not on the server (HTTP 404). The script still has it in its list.
- The bags contain the full raw drive. For example, the seq 08 bag has frames 0-5176. The odometry benchmark uses frames 1100-5170.
- OXTS timestamps go backwards for 6-36 samples in each drive. The script removes these samples. Thus the IMU and the ground truth use one monotonic clock.
- Packet loss causes IMU gaps of 1.6-1.9 s in seqs 00, 02, 05, 06 and 08. Each `<drive>.txt` lists the gaps.
- The ground truth is the OXTS GPS/INS pose of the IMU (decimeter accuracy). It is not the camera pose of the odometry benchmark.
- `_gt.bag` is large (1.1 GB for seq 08). Use it for RViz only. Use `gt_tum.txt` for the evaluation.

## 3. M3ED

Steps:

```bash
mkdir -p $DATA/VIO/M3ED && cd $DATA/VIO/M3ED
python3 <repo>/datasets/M3ED/extract_ovc.py --vehicle car --cameras left \
    --yaml <repo>/datasets/M3ED/dataset_lists.yaml --output_dir data_car
python3 <repo>/datasets/M3ED/make_vins_bags.py --input_dir data_car --output_dir vins --jobs 3
```

- `extract_ovc.py` uses HTTP range requests. It copies only `/ovc/{left,imu,ts}` and the calibration from each remote `data.h5`. It also downloads `pose_gt.h5`.
- Do not download the full files. Car and falcon together are approximately 2.1 TB.
- The script skips the test sequences, because they have no public ground truth.

Layout:

```
data_car/<seq>/{<seq>_data.h5, <seq>_pose_gt.h5}
vins/<seq>/
|-- data.bag       # /cam0/image_raw mono8 1280x800 25 Hz, /imu0 400 Hz
|-- gt.bag         # /ground_truth/odometry (10 Hz), /ground_truth/path
`-- calib.yaml     # pinhole-radtan intrinsics, T_imu_cam, time_offset_s
```

Data problems:

- M3ED time starts at 0. The bags add 1e9 s to all timestamps. `calib.yaml` records the offset.
- The ground truth is the IMU pose. Its world frame is the IMU frame at t0, which is not gravity-aligned. Align the trajectories before you calculate the error.
- The twist in the odometry messages is zero.
- Approximately 0.3 % of the IMU samples are NaN. The script skips them. Thus there are gaps of 5-12.5 ms.
- `car_forest_tree_tunnel` and `car_forest_into_ponds_long` do not come back to their start point.

## 4. TartanGround

Steps:

```bash
python3 <repo>/datasets/TartanGround/download_tartanground.py \
    --output_dir $DATA/VIO/TartanGround/data --max-per-env 3        # --dry-run, --envs X
```

The script downloads `image_lcam_front.zip`, `imu.zip` and `metadata.zip` of `Data_diff`. Then it extracts them and deletes the archives.

Layout:

```
data/<Env>/Data_diff/P100x/
|-- image_lcam_front/NNNNNN_lcam_front.png
|-- imu/{imu_time,cam_time,acc,gyro}.txt, parameter.yaml      # 10 Hz images, 100 Hz IMU
|-- pose_lcam_front.txt                                        # x y z qx qy qz qw, NED
`-- P100x_metadata.json
```

The disk does not keep TartanGround bags. Each method makes its input during the run:

| Method | Converter | Output |
|---|---|---|
| VINS-Mono | `VINS-Mono/run_tartanground.py` | Temporary bag, deleted after the runs |
| OpenVINS | `open_vins_ws/tartanground_to_bag.py` | Temporary bag, deleted after the runs |
| DPVO | `DPVO/run_multiseed.py` | Reads the PNG files directly |

A bag is approximately 0.85 GB for each 200 s.

Data problems:

- The IMU has no noise.
- The IMU frame is the `lcam_front` camera frame (NED). Thus the camera-IMU transform is a rotation only.
- The last image has no IMU sample after it. The converters do not use it.
- The converters add 1000 s to all timestamps.

## 5. TartanAir v2

```bash
cp <repo>/datasets/TartanAir/traj_index.json $DATA/VIO/TartanAir/
python3 <repo>/datasets/TartanAir/download_tartanair.py --output_dir $DATA/VIO/TartanAir/data --budget_gb 180
```

- The script reads the HuggingFace archives `theairlabcmu/tartanair2` with range requests. It extracts only the selected trajectories.
- Each round adds one trajectory to each (environment, difficulty) group. The script selects the trajectory that is nearest to the median length of the group.
- `traj_index.json` keeps the trajectory sizes. With this file, the selection is the same as on the original machine.
- Layout: `data/<Env>/Data_{easy,hard}/P0xx/{image_lcam_front/, imu/, pose_lcam_front.txt}`.

## 6. EuRoC MAV

Download Vicon Room 1 (V1_01_easy, V1_02_medium, V1_03_difficult) as ROS bags from the EuRoC MAV dataset page. Put them in `$DATA/VIO/EuRoC-MAV/vicon_room1/<seq>/<seq>.bag`. The official bags already use `/cam0/image_raw` and `/imu0`. No conversion is necessary.
