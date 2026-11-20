#!/bin/bash
# Run WITHOUT sudo: sudo drops the conda env (-> missing tartanair).
# Needs: pip install tartanair
# Extra args are passed through, e.g. --dry-run, --max-per-env 2, --envs SeasideTown

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
OUTPUT_DIR="$SCRIPT_DIR/data"
PYTHON="${PYTHON:-python3}"
export PYTHONUNBUFFERED=1  # live progress when logging to a file

# Monocular VIO subset: diff-drive, lcam_front + imu + poses, 3 trajectories per env
"$PYTHON" "$SCRIPT_DIR/download_tartanground.py" \
    --output_dir "$OUTPUT_DIR" \
    --max-per-env 3 \
    "$@" || { echo "TartanGround download failed."; exit 1; }

echo "Batch download process terminated successfully."
