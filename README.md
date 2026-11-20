# VIO-Benchmarks

This repository compares three monocular visual(-inertial) odometry methods on public datasets. It contains the setup, the dataset scripts, the run and evaluation scripts, the results and the report.

| Method | Estimator type | Sensors | Runtime | Folder |
|---|---|---|---|---|
| VINS-Mono | Sliding-window nonlinear optimization (Ceres) | Camera, IMU | ROS Kinetic in Docker, CPU | `VINS-Mono/` |
| DPVO | Deep patch visual odometry, differentiable bundle adjustment | Camera | PyTorch, CUDA GPU | `DPVO/` |
| OpenVINS | Multi-state constraint Kalman filter (MSCKF) | Camera, IMU | ROS Noetic in Docker, CPU | `open_vins_ws/` |

| Dataset | Sequences | Runs for each sequence | Length |
|---|---|---|---|
| KITTI raw (odometry 00-10 without 03) | 10 | 10 | 22.8 km |
| M3ED (car) | 16 | 10 | 17.4 km |
| TartanGround (differential drive) | 117 | 5 | 44.1 km |

## 1. Structure

```
datasets/*  --download, convert-->  $DATA/VIO/<DATASET>/   (rosbags, PNG, HDF5, gt_tum.txt)
                                          |
          +-------------------------------+------------------------------+
          |                               |                              |
   VINS-Mono (Docker)               DPVO (conda, GPU)              OpenVINS (Docker)
   rosbag, real-time play           images, every frame            rosbag, frame by frame
          |                               |                              |
          +----------- X sequences x N runs -> traj_estimate_tum.txt ----+
                                          |
                     eval_multiseed.py (evo) -> runs.csv, REPORT.md
                     make_table.py           -> results/table.tex
                                          |
                     report/file.tex, report/executive_summary.pdf
```

Each method has its own copy of the evaluation scripts. The three copies use the same metric definitions (Section 5).

## 2. Repository layout

| Path | Content | Type |
|---|---|---|
| `VINS-Mono/` | Fork of HKUST-Aerial-Robotics/VINS-Mono, branch `vio-benchmarks`. Guide: `VINS-Mono/BENCHMARK.md`. | Submodule |
| `DPVO/` | Fork of princeton-vl/DPVO, branch `vio-benchmarks`. Guide: `DPVO/BENCHMARK.md`. | Submodule |
| `open_vins_ws/` | Catkin workspace, run and evaluation scripts. Guide: `open_vins_ws/README.md`. | Folder |
| `open_vins_ws/src/open_vins/` | Fork of rpng/open_vins, branch `vio-benchmarks` (dataset configurations). | Submodule |
| `datasets/` | Download and conversion scripts. Guide: `datasets/README.md`. | Folder |
| `report/` | Slide tables (`file.tex`), executive summary (PDF and its generator). | Folder |

Each fork adds files only. The estimator source code is not changed.

## 3. Install

1. Clone the repository with all submodules:

   ```bash
   git clone --recurse-submodules https://github.com/efekaanefe/VIO-Benchmarks.git
   cd VIO-Benchmarks
   ```

2. Prepare the datasets. Refer to [datasets/README.md](datasets/README.md).

3. Install and build each method:

   | Method | Commands | Guide |
   |---|---|---|
   | VINS-Mono | `cd VINS-Mono && export DATASETS_DIR=/abs/path && docker/vins.sh build` | [VINS-Mono/BENCHMARK.md](https://github.com/efekaanefe/VINS-Mono/blob/vio-benchmarks/BENCHMARK.md) |
   | DPVO | `cd DPVO && conda env create -f environment.yml`, then the build steps in the guide | [DPVO/BENCHMARK.md](https://github.com/efekaanefe/DPVO/blob/vio-benchmarks/BENCHMARK.md) |
   | OpenVINS | `cd open_vins_ws && ./ov_docker.sh image && DATASETS_DIR=/abs/path ./ov_docker.sh build` | [open_vins_ws/README.md](open_vins_ws/README.md) |

4. Change the hard-coded dataset paths. Each guide has a table of these paths. The original root is `/media/efe/Data/datasets`.

Requirements:

- Linux with Docker (VINS-Mono, OpenVINS).
- An NVIDIA GPU with CUDA 12 and conda (DPVO).
- Python 3 on the host with `numpy pandas scipy matplotlib evo` (evaluation).

## 4. Run

| Method | One dataset, all runs | Output |
|---|---|---|
| VINS-Mono | `python3 run_multiseed.py m3ed --runs 10` | `VINS-Mono/results/multi_seed/<dataset>/` |
| DPVO | `DATASETS=m3ed RUNS=10 ./run_multiseed_all.sh` | `DPVO/results/multi_seed/<dataset>/` |
| OpenVINS | `DATASETS=m3ed RUNS=10 ./run_multiseed_all.sh` | `open_vins_ws/results/multi_seed/<dataset>/` |

Rules for all runs:

- Only one run operates at a time when the FPS is measured. Parallel runs share the CPU or GPU and change the FPS.
- A finished run has a `run.log`. The scripts skip finished runs. Thus a stopped job can continue.
- Loop closure is off for all methods. The results show odometry drift only.

## 5. Evaluation

All metrics come from [evo](https://github.com/MichaelGrupp/evo). The ground truth is interpolated at the timestamps of the estimate.

Alignment (Umeyama). Let $p_i$ be the estimated positions and $g_i$ the ground-truth positions:

$$(s, R, t) = \arg\min_{s,R,t} \sum_i \lVert g_i - (s R p_i + t) \rVert^2$$

- $Sim(3)$ alignment: $s$ is free. This removes the scale error of monocular methods.
- $SE(3)$ alignment: $s = 1$. This keeps the scale error.

Absolute trajectory error, in % of the ground-truth path length $L$:

$$\mathrm{ATE} = \frac{100}{L} \sqrt{\frac{1}{N} \sum_{i=1}^{N} \lVert g_i - (s R p_i + t) \rVert^2}$$

Relative translation error over segments of $d = 100$ m of ground-truth path. Let $P_i$ be the estimated pose and $Q_i$ the ground-truth pose. For all pairs $(i, j)$ with a ground-truth path distance of $d$:

$$E_{ij} = (Q_i^{-1} Q_j)^{-1} (P_i^{-1} P_j), \qquad \mathrm{RPE\text{-}t} = \frac{100}{d} \sqrt{\frac{1}{M} \sum_{(i,j)} \lVert \mathrm{trans}(E_{ij}) \rVert^2}$$

- $SE(3)$ column: RPE of the raw estimate.
- $Sim(3)$ column: RPE of the estimate multiplied by $s$.
- A sequence shorter than 100 m has no RPE value.

Failure criteria for a run:

| Outcome | Condition |
|---|---|
| `reinit` | The estimator rebooted or initialized again (VINS-Mono). The pose goes back to the origin. |
| `diverged` | $Sim(3)$ ATE > 10 % of $L$, or $s \notin [0.5, 2]$ (VIO methods). Non-finite poses (DPVO). |
| `no_init`, `crashed` | No initialization, or the process stopped. |

Aggregation. Each table cell is the mean over the $X$ sequences of the mean over the successful runs of each sequence:

$$\bar{e} = \frac{1}{X} \sum_{k=1}^{X} \frac{1}{|O_k|} \sum_{r \in O_k} e_{k,r}, \qquad \mathrm{Failed} = 100 \cdot \frac{\text{failed runs}}{X \cdot N}$$

$O_k$ is the set of successful runs of sequence $k$. Failed runs are not in the error columns.

FPS. The three methods use different modes. Thus compare the FPS values with care:

| Method | Mode | FPS meaning |
|---|---|---|
| VINS-Mono | Bag plays at real time. | Pose output rate. On M3ED and TartanGround, the camera/tracker rate (10 Hz) limits it. |
| DPVO | The reader waits for the estimator. | Frames / processing time, one run on the GPU. |
| OpenVINS | `serial.launch` reads the bag frame by frame. | Processing rate. |

## 6. Results

| Dataset | Method | Runs X x N | Failed % | ATE SE(3) % | ATE Sim(3) % | RPE-t SE(3) % | RPE-t Sim(3) % | FPS |
|---|---|---|---|---|---|---|---|---|
| KITTI | VINS-Mono * | 10 x 10 | 30 | 6.7 | 2.03 | 33.3 | 18.8 | 9.63 |
| KITTI | DPVO | 10 x 10 | 0 | 12.3 | 2.94 | 82.7 | 31.7 | 4.76 |
| KITTI | OpenVINS * | 10 x 10 | 30 | 3.69 | 3.00 | 20.0 | 17.6 | 81.5 |
| M3ED | VINS-Mono | 16 x 10 | 12.5 | 0.40 | 0.31 | 2.11 | 1.89 | 10.0 |
| M3ED | DPVO | 16 x 10 | 0 | 12.7 | 0.91 | 75.1 | 6.0 | 5.10 |
| M3ED | OpenVINS | 16 x 10 | 0 | 0.72 | 0.51 | 4.46 | 3.43 | 48.0 |
| TartanGround | VINS-Mono | 117 x 5 | 0.3 | 0.31 | 0.24 | 1.15 | 1.18 | 10.0 |
| TartanGround | DPVO | 117 x 5 | 0.2 | 9.1 | 2.89 | 37.7 | 14.3 | 4.78 |
| TartanGround | OpenVINS | 117 x 5 | 7.5 | 0.31 | 0.21 | 1.58 | 1.32 | 45.8 |

Hardware: Intel i7-5930K and NVIDIA GTX 1080 Ti.

\* These KITTI rows come from 10 x 10 runs on a second PC (AMD Ryzen 5 5500U). Those runs are not in this repository. The KITTI runs on the first PC (10 x 1) are in `VINS-Mono/results/table.tex` and `open_vins_ws/results/table.tex`.

Sources of the other rows: `<method>/results/table.tex`, made by `make_table.py` from `results/multi_seed/`.

Main points:

- DPVO has no IMU. Its scale is different in each run. Thus its $SE(3)$ ATE is 3-14 times its $Sim(3)$ ATE.
- On M3ED and TartanGround, the two VIO methods have a $Sim(3)$ ATE of 0.2-0.5 % of the path.
- KITTI is the most difficult dataset for the VIO methods. The OXTS IMU has timestamp errors and gaps of up to 1.9 s, and long straight drives make the scale and the IMU biases difficult to observe.
- VINS-Mono: the wall-clock solver limit (`max_solver_time`) causes run-to-run differences. With a limit of 1000 s, runs are bit-identical.
- OpenVINS: 8 TartanGround sequences diverge in all 5 runs. The scale goes to 0-0.05.
- On the i7-5930K and GTX 1080 Ti, DPVO (all datasets) and VINS-Mono (KITTI) are slower than the camera. OpenVINS processes 2-8 times faster than the camera rate.

## 7. Report

| File | Content |
|---|---|
| `report/file.tex` | Beamer slides. The result tables use the values of Section 6. |
| `report/executive_summary.pdf` | One-page summary of the project. |
| `report/make_summary.py` | Makes the PDF. Requires `pip install reportlab`. |

## 8. Licenses

- VINS-Mono and OpenVINS: GPLv3. DPVO: MIT. Refer to the license file in each submodule.
- The ground-truth files in `results/` come from the datasets. The dataset licenses apply: KITTI (CC BY-NC-SA 3.0), M3ED and TartanGround (refer to their web pages).
