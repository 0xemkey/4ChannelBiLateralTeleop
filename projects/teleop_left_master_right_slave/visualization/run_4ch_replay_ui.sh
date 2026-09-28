#!/usr/bin/env bash
set -euo pipefail
VIS_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
if (($# < 1)); then echo 'Usage: run_4ch_replay_ui.sh FILE [--rviz] [--dry-run]' >&2; exit 2; fi
input_file="$1"
shift
exec bash "$VIS_DIR/run_4ch_sim_ui.sh" --source replay --file "$input_file" "$@"
