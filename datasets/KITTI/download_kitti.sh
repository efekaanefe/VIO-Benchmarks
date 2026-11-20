#!/bin/bash

# KITTI Odometry Sequences 00 to 10 mapped to their Raw Data Drives
DRIVES=(
  "2011_10_03_drive_0027" # Seq 00
  "2011_10_03_drive_0042" # Seq 01
  "2011_10_03_drive_0034" # Seq 02
  "2011_09_26_drive_0067" # Seq 03
  "2011_09_30_drive_0016" # Seq 04
  "2011_09_30_drive_0018" # Seq 05
  "2011_09_30_drive_0020" # Seq 06
  "2011_09_30_drive_0027" # Seq 07
  "2011_09_30_drive_0028" # Seq 08
  "2011_09_30_drive_0033" # Seq 09
  "2011_09_30_drive_0034" # Seq 10
)

BASE_URL="https://s3.eu-central-1.amazonaws.com/avg-kitti/raw_data"

for DRIVE in "${DRIVES[@]}"; do
  echo "Downloading synced and unsynced data for $DRIVE..."

  # Synced version
  wget -c --show-progress "$BASE_URL/$DRIVE/${DRIVE}_sync.zip"

  # Unsynced (extract) version
  wget -c --show-progress "$BASE_URL/$DRIVE/${DRIVE}_extract.zip"
done
