#!/bin/bash
# Run OpenVINS (ros1_serial_msckf, monocular) on the KITTI raw bags and save the estimated trajectories.
# Run from the host:  [JOBS=3] ./run_kitti.sh [sequence ...]
#
# Results (host):  results/kitti/<sequence>/
#   traj_estimate.txt   IMU pose in global: timestamp(s) tx ty tz qx qy qz qw + covariance
#   traj_timing.txt     per-frame timing
#   run.log             estimator console output
WS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
BAG_HOST="${BAG_HOST:-/media/efe/Data/datasets/VIO/KITTI}"
BAG_ROOT="${BAG_ROOT:-/datasets/VIO/KITTI}"   # same directory inside the container
OUT_ROOT="/catkin_ws/results/kitti"                # path inside the container
PORT="${PORT:-11430}"                             # first ROS master port, one per parallel job
JOBS="${JOBS:-3}"

run_seq() {
  local seq="$1" port="$2"
  # one calibration per capture date, the sequence name starts with it (2011_09_30_drive_0016)
  local config="kitti_${seq%_drive_*}_tuned"
  if [ ! -d "$WS_DIR/src/open_vins/config/$config" ]; then echo "[$seq] no config $config, skipping"; return; fi
  if [ ! -f "$BAG_HOST/$seq/vins/$seq.bag" ]; then echo "[$seq] bag not found, skipping"; return; fi

  local res="$WS_DIR/results/kitti/$seq" out="$OUT_ROOT/$seq"
  mkdir -p "$res"
  rm -f "$res"/traj_*.txt
  local t0=$(date +%s)
  "$WS_DIR/ov_docker.sh" roslaunch -p "$port" ov_msckf serial.launch \
    config:="$config" dataset:="$seq" bag:="$BAG_ROOT/$seq/vins/$seq.bag" bag_start:=0 \
    max_cameras:=1 use_stereo:=false \
    dosave:=true path_est:="$out/traj_estimate.txt" \
    dotime:=true path_time:="$out/traj_timing.txt" \
    > "$res/run.log" 2>&1 < /dev/null
  echo "[$seq] exit=$? in $(( $(date +%s) - t0 ))s, poses: $(grep -vc '^#' "$res/traj_estimate.txt" 2>/dev/null)"
}

if [ $# -gt 0 ]; then
  SEQS=("$@")
else
  SEQS=($(cd "$BAG_HOST" && ls -d *_drive_*/ | tr -d /))
fi

"$WS_DIR/ov_docker.sh" true  # make sure the container is up before the parallel jobs
slot=0
for seq in "${SEQS[@]}"; do
  run_seq "$seq" $(( PORT + slot % JOBS )) &
  slot=$(( slot + 1 ))
  # each slot keeps its own ROS master port, so wait for the whole batch
  if [ $(( slot % JOBS )) -eq 0 ]; then wait; fi
done
wait
