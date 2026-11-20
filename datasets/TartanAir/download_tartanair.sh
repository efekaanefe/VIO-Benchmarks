#!/bin/bash
# Run WITHOUT sudo: sudo drops the conda env (-> missing tartanair/requests).
# Needs: pip install tartanair
# Extra args are passed through, e.g. --dry-run, --budget_gb 150, --envs Hospital

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
OUTPUT_DIR="$SCRIPT_DIR/data"
PYTHON="${PYTHON:-python3}"
export PYTHONUNBUFFERED=1  # live progress when logging to a file

# Monocular VIO subset: all envs, easy + hard, lcam_front + imu + poses, ~180 GB
"$PYTHON" "$SCRIPT_DIR/download_tartanair.py" \
    --output_dir "$OUTPUT_DIR" \
    --budget_gb 180 \
    "$@" || { echo "TartanAir download failed."; exit 1; }

echo "Batch download process terminated successfully."
