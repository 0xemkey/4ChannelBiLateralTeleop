#!/usr/bin/env bash
set -euo pipefail
EXTRA_ARGS=(); source "$(cd "$(dirname "$0")" && pwd)/common.sh"; parse_common_args "$@"
[[ ${#EXTRA_ARGS[@]} -eq 0 ]] || { echo "[master-c2-diag][error] unknown argument: ${EXTRA_ARGS[*]}" >&2; exit 2; }

# DIAGNOSTIC MODE C (task section 3): "Slave deliberately commands master
# position with low authority solely to verify the reverse communication
# and controller path." A DELIBERATE, EXPLICIT alternative to
# 01_start_master.sh, not a flag on it -- per the task's own instruction
# not to silently change normal free-space master behavior. Run this
# INSTEAD of 01_start_master.sh for a C2 diagnostic session, never both.
#
# Gains, deliberately far below production C2 values (LEFT_STIFFNESS_N_PER_M
# =15.0, LEFT_DAMPING_N_S_PER_M=0.10, confirmed live 2026-09-03) and
# MAX_FORCE_N (2.0, the normal ceiling): this is "can the reverse path move
# the master's arm perceptibly and safely at all", not a felt-quality pass.
# 0.5N sits safely under the Touch's ~0.88N continuous rating even during
# a long diagnostic session.
reject_live_component 'touch_slave.*(__node:=teleop_master_left|device:=Left)' 'left master'
cmd=(env ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}" ENABLE_C2=true JOINT_SPACE_CONTROL=false \
  LEFT_STIFFNESS_N_PER_M=5.0 LEFT_DAMPING_N_S_PER_M=0.05 MAX_FORCE_N=0.5 \
  "${PROJECT_ROOT}/scripts/run_master_left.sh")
echo '[master-c2-diag][warning] DIAGNOSTIC MODE: C2 reverse-position path armed at LOW gain'
echo '  stiffness=5.0 N/m (vs 15.0 production) damping=0.05 Ns/m (vs 0.10) max_force=0.5N (vs 2.0)'
echo '  This deliberately does NOT match normal X-lateral free-motion behavior (tau_m=0).'
echo '  Bridge must ALSO be started with 03b_start_bridge_c2_diagnostic.sh, not 03_start_position_bridge.sh.'
start_component master "${cmd[@]}"
