#!/usr/bin/env bash
set -o pipefail

LOG_DIR="$1"
LOG_NAME="$2"
shift 2

mkdir -p "$LOG_DIR"
export ROS_LOG_DIR="$LOG_DIR/ros"
mkdir -p "$ROS_LOG_DIR"

{
  echo "===== $(date '+%F %T') START $LOG_NAME ====="
  echo "PWD=$PWD"
  echo "CMD=$*"
  "$@"
  rc=$?
  echo "===== $(date '+%F %T') END $LOG_NAME rc=$rc ====="
  exit "$rc"
} 2>&1 | tee -a "$LOG_DIR/$LOG_NAME.log"
