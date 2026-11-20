#!/bin/bash
# 10 OpenVINS runs per sequence, each dataset followed by its evaluation.
#   [DATASETS="m3ed kitti"] [RUNS=10] [JOBS=1] ./run_multiseed_all.sh
# Resumable: finished runs are skipped. Logs: results/multi_seed/logs/
cd "$(dirname "$0")"
RUNS=${RUNS:-10}
JOBS=${JOBS:-1}   # one run at a time, so that the run time and fps of a run are its own
LOGS=results/multi_seed/logs
mkdir -p $LOGS

for dataset in ${DATASETS:-m3ed}; do
  echo "[$(date '+%F %T')] PHASE $dataset runs" >> $LOGS/progress.log
  python3 run_multiseed.py $dataset --runs $RUNS --jobs $JOBS >> $LOGS/${dataset}_runs.log 2>&1
  echo "[$(date '+%F %T')] PHASE $dataset eval" >> $LOGS/progress.log
  python3 eval_multiseed.py $dataset > $LOGS/${dataset}_eval.log 2>&1
  echo "[$(date '+%F %T')] PHASE $dataset done (eval rc=$?)" >> $LOGS/progress.log
done
echo "[$(date '+%F %T')] ALL DONE" >> $LOGS/progress.log
