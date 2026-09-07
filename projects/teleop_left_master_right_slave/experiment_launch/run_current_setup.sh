#!/usr/bin/env bash
set -euo pipefail
HERE="$(cd "$(dirname "$0")" && pwd)"
args=("$@")
"${HERE}/00_preflight.sh" "${args[@]}"
"${HERE}/01_start_master.sh" "${args[@]}"
"${HERE}/02_start_slave.sh" "${args[@]}"
echo '[setup] ZERO-FORCE preparation complete. Position bridge was NOT started.'
echo '[setup] After status checks and operator confirmation, run 03_start_position_bridge.sh with --arm-position.'

