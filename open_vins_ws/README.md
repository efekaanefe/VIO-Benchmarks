# OpenVINS workspace

This folder is the catkin workspace for OpenVINS. It contains the run, evaluation and plot scripts. The OpenVINS source is the submodule `src/open_vins` (branch `vio-benchmarks` of [efekaanefe/open_vins](https://github.com/efekaanefe/open_vins)). That branch adds the dataset configurations. Its `BENCHMARK.md` describes them.

- Mode: monocular camera and IMU, `ros1_serial_msckf` through `serial.launch`.
- Environment: ROS Noetic, Ubuntu 20.04, in Docker.
- The scripts in this folder run on the host. `ov_docker.sh` sends commands into the container.

## 1. Components

```
host                                          container "open_vins" (ROS Noetic)
----                                          ----------------------------------
run_{euroc,kitti,m3ed,tartanground}.sh  --+
run_multiseed.py / run_multiseed_all.sh --+-> ov_docker.sh roslaunch ov_msckf serial.launch
tartanground_to_bag.py (temporary bag)        reads the bag frame by frame -> traj_estimate.txt
eval_{euroc,kitti,m3ed,tartanground}.py   -> evo_results.csv, *_tum.txt
eval_multiseed.py                         -> runs.csv, summary_*.csv, REPORT.md
make_table.py                             -> results/table.tex, results/table_runs.csv
plot_traj.py, plot_m3ed.py                -> trajectory plots (SE(3) and Sim(3))
```

Container mounts:

| Container | Host |
|---|---|
| `/catkin_ws` | `open_vins_ws/` (thus `build/`, `devel/`, `logs/` stay on the host) |
| `$HOME` | `open_vins_ws/.home` |
| `/datasets` | `$DATASETS_DIR` |

## 2. Requirements

- Linux with Docker. Your user must be in the `docker` group.
- Python 3 on the host, with `evo rosbags opencv-python numpy pandas scipy matplotlib`. The container has no evo.
- The dataset layout of [../datasets/README.md](../datasets/README.md) under `$DATASETS_DIR/VIO/`.

## 3. Build

```bash
cd VIO-Benchmarks/open_vins_ws
./ov_docker.sh image                                   # build the image ov_ros1_20_04
DATASETS_DIR=/abs/path/to/datasets ./ov_docker.sh true  # create and start the container
./ov_docker.sh build                                   # catkin build, Release
```

| Command | Function |
|---|---|
| `./ov_docker.sh` | Open a shell in the container. |
| `./ov_docker.sh <cmd...>` | Run a command in the container, with ROS sourced. |
| `./ov_docker.sh build` | `catkin build`. `JOBS=n` sets the jobs (default 4, approximately 1.2 GB RAM each). |
| `./ov_docker.sh stop` | Remove the container. Do this before you change `DATASETS_DIR`. |

## 4. Set the paths

| File | Value | Action |
|---|---|---|
| `ov_docker.sh` | `DATASETS_DIR=/media/efe/Data/datasets` | Export `DATASETS_DIR`. |
| `run_kitti.sh`, `run_m3ed.sh`, `eval_kitti.py`, `eval_m3ed.py`, `eval_tartanground.py` | `BAG_HOST=/media/efe/Data/datasets/VIO/...` | Export `BAG_HOST`. |
| `run_euroc.sh`, `run_multiseed.py` (`DATA_HOST`), `tartanground_to_bag.py` (`DATA`), `plot_traj.py` | `/media/efe/Data/datasets/...` | Edit the value. |

The scripts use ROS master ports 11411, 11420-11422, 11430-11432, 11450 and higher. These ports must be free.

## 5. Run one sequence

```bash
./run_euroc.sh V1_01_easy                        # config euroc_mav
./run_m3ed.sh car_urban_day_horse                # config from the name: m3ed_car_{urban_day,urban_night,forest}
JOBS=1 ./run_kitti.sh 2011_09_30_drive_0018      # config kitti_<date>_tuned
./run_tartanground.sh AbandonedCable_P1000       # convert, run, delete the bag
```

- Without an argument, a script runs all sequences that it finds.
- Output: `results/<dataset>/<seq>/{traj_estimate.txt, traj_timing.txt, run.log}`.
- `run_kitti.sh` runs 3 jobs in parallel by default. Use `JOBS=1` for a correct FPS.
- `run_tartanground.sh` skips a sequence that has a `run.log`. `KEEP_BAGS=1` keeps the bag.

TartanGround bags:

```bash
python3 tartanground_to_bag.py --list
python3 tartanground_to_bag.py --bag-dir tmp_bags AbandonedCable_P1000   # -> tmp_bags/<seq>.bag
```

The bag has `/cam0/image_raw` (mono8, 10 Hz) and `/imu0` (100 Hz). The converter also writes `TartanGround/vins/<seq>/gt_tum.txt`.

## 6. Repeated runs

```bash
./run_multiseed_all.sh                                 # M3ED: 16 x 10 runs (7.6 h, then 50 min evaluation)
DATASETS=tartanground RUNS=5 ./run_multiseed_all.sh    # TartanGround: 117 x 5 runs
python3 run_multiseed.py {m3ed|kitti|tartanground} --runs N [--jobs N] [--sequences ...] [--port 11450]
```

- The runs operate one at a time (`JOBS=1`). Thus the FPS of a run is its own FPS.
- A `run_XX/run.log` marks a finished run. Thus you can stop the job and start it again.
- Progress: `results/multi_seed/logs/progress.log`. The last line is `ALL DONE`.
- `run_multiseed_all.sh` runs `eval_multiseed.py` after the runs.
- Output: `results/multi_seed/<dataset>/<seq>/run_XX/{traj_estimate_tum.txt, run.log}`.

The TartanGround results in this repository came from these steps:

1. One pass: `./run_tartanground.sh > results/tartanground/logs/runs.log 2>&1`.
2. `python3 import_tartanground_run00.py` copies that pass as `run_00`.
3. `DATASETS=tartanground RUNS=5 ./run_multiseed_all.sh` adds `run_01` to `run_04` (9.3 h).

KITTI repeated runs are possible, but they were not done on this machine. The KITTI row comes from one `run_kitti.sh` pass.

## 7. Evaluate

Do the steps in this sequence:

```bash
python3 eval_kitti.py; python3 eval_m3ed.py; python3 eval_tartanground.py   # single runs, evo_results.csv
python3 eval_multiseed.py m3ed tartanground                               # repeated runs, REPORT.md
python3 make_table.py                                                     # results/table.tex
python3 plot_traj.py kitti m3ed tartanground                              # 6000x3000 px plots
python3 plot_m3ed.py                                                      # results/m3ed/trajectories_{se3,sim3}.png
```

A run is `diverged` if its Sim(3) ATE is more than 10 % of the path, or if its Sim(3) scale is outside [0.5, 2]. A run with no trajectory also fails.

Note: `eval_kitti.py` and `eval_m3ed.py` select the RPE pairs along the estimate. The other scripts select them along the ground truth.

## 8. Results

Refer to `src/open_vins/BENCHMARK.md` and the project README.
