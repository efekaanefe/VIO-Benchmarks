#!/bin/bash
# Run OpenVINS (ros1_serial_msckf, stereo) on the EuRoC MAV bags and save the estimated trajectories.
# Run from the host:  ./run_euroc.sh [sequence ...]
#
# Results (host):  results/euroc_mav/<sequence>/
#   traj_estimate.txt   IMU pose in global: timestamp(s) tx ty tz qx qy qz qw + covariance
#   traj_timing.txt     per-frame timing
#   run.log             estimator console output
WS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
BAG_ROOT="${BAG_ROOT:-/datasets/VIO/EuRoC-MAV}"   # path inside the container
OUT_ROOT="/catkin_ws/results/euroc_mav"           # path inside the container
PORT="${PORT:-11411}"                             # own ROS master port (container uses host network)

if [ $# -gt 0 ]; then
  SEQS=("$@")
else
  SEQS=()
  for b in /media/efe/Data/datasets/VIO/EuRoC-MAV/*/*/*.bag; do
    [ -f "$b" ] && SEQS+=("$(basename "$b" .bag)")
  done
  SEQS=($(printf '%s\n' "${SEQS[@]}" | sort -u))
fi

for seq in "${SEQS[@]}"; do
  bag_host=$(ls /media/efe/Data/datasets/VIO/EuRoC-MAV/*/"$seq"/"$seq".bag 2>/dev/null | head -1)
  if [ -z "$bag_host" ]; then echo "[$seq] bag not found, skipping"; continue; fi
  bag="$BAG_ROOT/${bag_host#/media/efe/Data/datasets/VIO/EuRoC-MAV/}"

  # recommended bag start offsets (seconds) from serial.launch
  case "$seq" in
    MH_01*) start=40 ;; MH_02*) start=35 ;; MH_03*) start=17.5 ;; MH_04*|MH_05*) start=15 ;; *) start=0 ;;
  esac

  out="$OUT_ROOT/$seq"
  mkdir -p "$WS_DIR/results/euroc_mav/$seq"
  rm -f "$WS_DIR/results/euroc_mav/$seq"/traj_*.txt
  echo "[$seq] running (bag_start=$start) ..."
  t0=$(date +%s)
  "$WS_DIR/ov_docker.sh" roslaunch -p "$PORT" ov_msckf serial.launch \
    config:=euroc_mav dataset:="$seq" bag:="$bag" bag_start:="$start" \
    max_cameras:=2 use_stereo:=true \
    dosave:=true path_est:="$out/traj_estimate.txt" \
    dotime:=true path_time:="$out/traj_timing.txt" \
    > "$WS_DIR/results/euroc_mav/$seq/run.log" 2>&1 < /dev/null
  echo "[$seq] exit=$? in $(( $(date +%s) - t0 ))s, poses: $(grep -vc '^#' "$WS_DIR/results/euroc_mav/$seq/traj_estimate.txt" 2>/dev/null)"
done
