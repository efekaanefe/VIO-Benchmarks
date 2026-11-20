#!/bin/bash
# Run OpenVINS (ros1_serial_msckf, monocular) once on every TartanGround trajectory and save the estimated trajectories.
# Run from the host:  ./run_tartanground.sh [sequence ...]      (default: every downloaded trajectory)
#
# Per sequence, strictly one at a time (so the per-frame timing / fps is not distorted):
#   1. tartanground_to_bag.py writes the bag to tmp_bags/ in the workspace (SSD; the dataset HDD is
#      too slow to read the PNGs and write the bags at the same time) and gt_tum.txt next to the data
#   2. OpenVINS runs on the bag
#   3. the bag is deleted again (KEEP_BAGS=1 keeps it)
# Sequences that already have a run.log are skipped, so the script can be re-run to resume.
#
# Results (host):  results/tartanground/<sequence>/
#   traj_estimate.txt   IMU pose in global: timestamp(s) tx ty tz qx qy qz qw + covariance
#   traj_timing.txt     per-frame timing
#   run.log             estimator console output
WS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
BAG_HOST="$WS_DIR/tmp_bags"
BAG_ROOT="/catkin_ws/tmp_bags"             # same directory inside the container
OUT_ROOT="/catkin_ws/results/tartanground" # path inside the container
CONFIG="tartanground_diff"
PORT="${PORT:-11460}"

run_seq() {
  local seq="$1"
  local res="$WS_DIR/results/tartanground/$seq" out="$OUT_ROOT/$seq"
  if [ -f "$res/run.log" ]; then echo "[$seq] done before, skipping"; return; fi
  mkdir -p "$BAG_HOST"
  local t0=$(date +%s)
  if ! python3 "$WS_DIR/tartanground_to_bag.py" --bag-dir "$BAG_HOST" "$seq"; then
    echo "[$(date '+%F %T')] [$seq] bag conversion failed"; rm -f "$BAG_HOST/$seq.bag"; return
  fi
  local t1=$(date +%s)

  mkdir -p "$res"
  rm -f "$res"/traj_*.txt
  "$WS_DIR/ov_docker.sh" roslaunch -p "$PORT" ov_msckf serial.launch \
    config:="$CONFIG" dataset:="$seq" bag:="$BAG_ROOT/$seq.bag" bag_start:=0 \
    max_cameras:=1 use_stereo:=false \
    dosave:=true path_est:="$out/traj_estimate.txt" \
    dotime:=true path_time:="$out/traj_timing.txt" \
    > "$res/run.log.tmp" 2>&1 < /dev/null
  local rc=$?
  mv "$res/run.log.tmp" "$res/run.log"
  [ "${KEEP_BAGS:-0}" = 1 ] || rm -f "$BAG_HOST/$seq.bag"
  echo "[$(date '+%F %T')] [$seq] exit=$rc, bag $(( t1 - t0 ))s, run $(( $(date +%s) - t1 ))s, poses: $(grep -vc '^#' "$res/traj_estimate.txt" 2>/dev/null)"
}

if [ $# -gt 0 ]; then
  SEQS=("$@")
else
  SEQS=($(python3 "$WS_DIR/tartanground_to_bag.py" --list))
fi

"$WS_DIR/ov_docker.sh" true  # make sure the container is up
for seq in "${SEQS[@]}"; do
  run_seq "$seq"
done
echo "[$(date '+%F %T')] ALL DONE"
