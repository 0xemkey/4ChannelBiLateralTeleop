#!/usr/bin/env bash
set -euo pipefail
VIS_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
export ROS_DOMAIN_ID=42
export ROS_LOCALHOST_ONLY=1
export ROS_LOG_DIR="$VIS_DIR/artifacts/ros_logs"
export MPLCONFIGDIR="$VIS_DIR/artifacts/matplotlib"
source_mode=synthetic
dry_run=false
rviz=false
args=()
while (($#)); do
  case "$1" in
    --source) source_mode="$2"; shift 2 ;;
    --dry-run) dry_run=true; shift ;;
    --rviz) rviz=true; shift ;;
    *) args+=("$1"); shift ;;
  esac
done
case "$source_mode" in synthetic|simulation|replay) ;; *) echo 'Unsupported source' >&2; exit 2 ;; esac
dashboard=(python3 "$VIS_DIR/teleop_4ch_dashboard.py" --source "$source_mode" "${args[@]}")
markers=(python3 "$VIS_DIR/teleop_4ch_visualizer.py" --source "$source_mode" "${args[@]}")
if "$dry_run"; then
  printf 'ROS_DOMAIN_ID=42 ROS_LOCALHOST_ONLY=1\n'
  printf '%q ' "${dashboard[@]}"; printf '\n'
  if "$rviz"; then
    printf '%q ' "${markers[@]}"; printf '\n'
    printf 'rviz2 -d %q\n' "$VIS_DIR/config/teleop_4ch.rviz"
  fi
  exit 0
fi
if ! "$rviz"; then exec "${dashboard[@]}"; fi
pids=()
cleanup() { for pid in "${pids[@]}"; do kill "$pid" 2>/dev/null || true; done; wait || true; }
trap cleanup EXIT INT TERM
"${markers[@]}" & pids+=("$!")
rviz2 -d "$VIS_DIR/config/teleop_4ch.rviz" & pids+=("$!")
"${dashboard[@]}" & pids+=("$!")
wait -n "${pids[@]}"
