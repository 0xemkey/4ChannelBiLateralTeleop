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
#
# 2026-09-10, Candidate A (C2 force-budget fix): real-hardware C2 Y-trial
# showed the master never moving -- spring output was architecturally
# capped at K*max_position_error_m = 5.0*0.03 = 0.15N regardless of
# max_force_n, and 0.15N + LEFT_GRAVITY_COMP_N=0.55N (0.70N total, measured)
# was insufficient. K and LEFT_GRAVITY_COMP_N are UNCHANGED here (per the
# analysis, raising max_force_n alone cannot add spring authority -- only
# max_position_error_m or K can). max_position_error_m raised 0.03->0.04m
# (spring authority 0.15N->0.20N), and MAX_FORCE_N raised 0.8->0.85N to
# match the new required budget (0.55 aux + 0.20 spring + 0.10 reserve =
# 0.85N exactly -- not a margin-padded guess). 0.85N stays under the
# Touch's ~0.88N continuous-rating warning threshold in run_master_left.sh,
# so that warning stays silent at this value.
# 2026-09-10: MAX_FORCE_N/LEFT_MAX_POSITION_ERROR_M made OVERRIDABLE (default
# unchanged at the Candidate A values above) -- needed so
# run_hardware_test.sh can run a single explicitly-justified, bounded
# breakaway-force step without a new near-duplicate launcher file per step.
# K (LEFT_STIFFNESS_N_PER_M) and damping stay hardcoded, not overridable --
# only max_force/max_position_error (the two levers that set spring
# authority and the budget ceiling) are meant to move for a breakaway test.
MAX_FORCE_N="${MAX_FORCE_N:-0.85}"
LEFT_MAX_POSITION_ERROR_M="${LEFT_MAX_POSITION_ERROR_M:-0.04}"
reject_live_component 'touch_slave.*(__node:=teleop_master_left|device:=Left)' 'left master'
cmd=(env ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}" ENABLE_C2=true JOINT_SPACE_CONTROL=false \
  LEFT_STIFFNESS_N_PER_M=5.0 LEFT_DAMPING_N_S_PER_M=0.05 MAX_FORCE_N="${MAX_FORCE_N}" \
  LEFT_MAX_POSITION_ERROR_M="${LEFT_MAX_POSITION_ERROR_M}" \
  "${PROJECT_ROOT}/scripts/run_master_left.sh")
echo '[master-c2-diag][warning] DIAGNOSTIC MODE: C2 reverse-position path armed at LOW gain'
echo "  stiffness=5.0 N/m (vs 15.0 production) damping=0.05 Ns/m (vs 0.10) max_force=${MAX_FORCE_N}N (vs 2.0)" \
     "max_position_error=${LEFT_MAX_POSITION_ERROR_M}m (vs 0.03 production)"
echo '  This deliberately does NOT match normal X-lateral free-motion behavior (tau_m=0).'
echo '  Bridge must ALSO be started with 03b_start_bridge_c2_diagnostic.sh, not 03_start_position_bridge.sh.'
start_component master "${cmd[@]}"
