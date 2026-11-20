#!/bin/bash
# Run OpenVINS (ros1_serial_msckf, monocular) on the M3ED car bags and save the estimated trajectories.
# Run from the host:  [JOBS=3] ./run_m3ed.sh [sequence ...]
#
# Results (host):  results/m3ed/<sequence>/
#   traj_estimate.txt   IMU pose in global: timestamp(s) tx ty tz qx qy qz qw + covariance
#   traj_timing.txt     per-frame timing
#   run.log             estimator console output
WS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
BAG_HOST="${BAG_HOST:-/media/efe/Data/datasets/VIO/M3ED/vins}"
BAG_ROOT="${BAG_ROOT:-/datasets/VIO/M3ED/vins}"   # same directory inside the container
OUT_ROOT="/catkin_ws/results/m3ed"                # path inside the container
PORT="${PORT:-11420}"                             # first ROS master port, one per parallel job
JOBS="${JOBS:-3}"

run_seq() {
  local seq="$1" port="$2"
  # camera calibration differs between the three sequence groups
  local config
  case "$seq" in
    car_forest_*)      config=m3ed_car_forest ;;
    car_urban_day_*)   config=m3ed_car_urban_day ;;
    car_urban_night_*) config=m3ed_car_urban_night ;;
    *) echo "[$seq] no config for this sequence, skipping"; return ;;
  esac
  if [ ! -f "$BAG_HOST/$seq/data.bag" ]; then echo "[$seq] bag not found, skipping"; return; fi

  local res="$WS_DIR/results/m3ed/$seq" out="$OUT_ROOT/$seq"
  mkdir -p "$res"
  rm -f "$res"/traj_*.txt
  local t0=$(date +%s)
  "$WS_DIR/ov_docker.sh" roslaunch -p "$port" ov_msckf serial.launch \
    config:="$config" dataset:="$seq" bag:="$BAG_ROOT/$seq/data.bag" bag_start:=0 \
    max_cameras:=1 use_stereo:=false \
    dosave:=true path_est:="$out/traj_estimate.txt" \
    dotime:=true path_time:="$out/traj_timing.txt" \
    > "$res/run.log" 2>&1 < /dev/null
  echo "[$seq] exit=$? in $(( $(date +%s) - t0 ))s, poses: $(grep -vc '^#' "$res/traj_estimate.txt" 2>/dev/null)"
}

if [ $# -gt 0 ]; then
  SEQS=("$@")
else
  SEQS=($(cd "$BAG_HOST" && ls -d */ | tr -d /))
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
