# Live state of the 4-channel bilateral controller.
#
# This message exists to answer one question directly: "is this actually
# 4-channel, or 2-channel, or unilateral?" That question came up repeatedly
# during bring-up and could previously only be answered by reading source.
#
# Channels, Lawrence numbering:
#   C1  slave force  -> master     C2  slave position -> master
#   C3  master position -> slave   C4  master force   -> slave
# Array index 0..3 corresponds to C1..C4.

std_msgs/Header header

# "4ch" | "4ch_safe" | "pp" | "pf" | "fp" | "unilateral" | "custom"
string architecture

# Commanded weights after ramping. Weights ramp rather than step because a step
# in w1 is a step force into the operator's hand.
float64[4] channel_weight

# The direct answer. True only when weight > 0 AND the source is fresh AND the
# estimate is valid. A channel can be weighted on and still be inactive.
bool[4] channel_active

# Why a channel is inactive, so "why is C1 dark" does not need a debugger.
# 0 OK, 1 ZERO_WEIGHT, 2 STALE, 3 INVALID, 4 BORROWED_REFUSED, 5 TANK_EMPTY,
# 6 TRIPPED, 7 CLUTCHED
uint8[4] channel_reason

# Estimated external joint torques crossing the channels, N*m.
float64[3] tau_e_hat_nm      # from slave  -> used by C1
float64[3] tau_h_hat_nm      # from master -> used by C4
float64 tau_e_age_s
float64 tau_h_age_s

# What was actually commanded this tick, N*m.
float64[3] master_torque_cmd_nm
float64[3] slave_torque_cmd_nm

# Fraction of each arm's per-joint torque budget in use, 0..1, worst joint.
# Published because the position channel and the force channel share one budget:
# as tracking error grows the PD term eats it and the force channel silently
# weakens, which the operator would otherwise experience as "the wall gets
# softer the harder I push."
float64 master_budget_used
float64 slave_budget_used

# Passivity energy tank on the master force channel, joules. Drained by
# positive work done BY the master ON the operator. A pumped oscillation is
# exactly sustained positive work, so this empties within a couple of cycles of
# the documented 0.007 -> 0.65 N failure and shuts the channel off before the
# operator feels anything worse than a soft wall.
float64 passivity_energy_j

# Damping coefficient handed to the nodes, N*m*s/rad. The COEFFICIENT crosses
# at 200 Hz; the product alpha*qdot is formed at 1 kHz inside the servo loop.
# Never send a velocity-derived product across the ROS path -- that lag is what
# turned damping into pumping before.
float64 damping_injected_nms_per_rad

# 60 s running mean of commanded torque magnitude, N*m. A thermal surrogate:
# HD_MOTOR_TEMPERATURE is dead on this hardware, so there is no real feedback.
float64 duty_cycle_nm

bool clutch_active
bool grip_detected

# 0 NOMINAL, 1 DERATED, 2 TRIPPED. TRIPPED latches and needs an explicit
# re-arm; it is set by the oscillation detector or an emptied tank.
uint8 safety_state

# Combined deadband, N*m, added 2026-09-04 (Phase 1.2 of the fine-tuning
# task). This is the two independent floors from touch_slave.cpp's
# computeForceEstimate() (per-joint threshold_nm) and this node's own
# kMinRelayTorqueNormNm, expressed as ONE number per joint per channel so
# "will a push of size X actually be felt" can be answered without doing
# the arithmetic by hand every time:
#
#     effective_threshold[j] = threshold_nm[j] + kMinRelayTorqueNormNm / w
#
# At w=0 the channel can never open regardless of push size; reported as
# +inf (not a large finite number) so a consumer cannot mistake it for a
# real, if strict, floor. THRESHOLD_NM in bilateral_4ch.py is a hand-kept
# copy of touch_slave.cpp's kThreshold array (that value lives in C++ and
# is not currently exposed over ROS) -- see the comment at its definition
# for the same caveat.
float64[3] effective_threshold_c1_nm
float64[3] effective_threshold_c4_nm

# Oscillation-guard diagnostics, added 2026-09-04 (Phase 1.3). The
# zero-crossing rate of the C1 torque vector over the last completed 1s
# window, Hz -- the secondary check that must ALSO be satisfied (on top of
# the existing pp-growing heuristic) before the "growing" trip condition
# fires, specifically to stop a human's deliberately-escalating push
# (near-zero crossing rate) from being misclassified as a developing
# oscillation. See analyze_window() in bilateral_4ch.py.
float64 c1_zero_crossing_hz
