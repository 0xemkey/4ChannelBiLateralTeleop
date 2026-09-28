# touch_master_slave_bridge's own internal state, published for logging.
# Added 2026-09-08 for the C2+C3 position-only diagnostic task -- several
# requested quantities (relative/mapped positions, the startup alignment
# state and error, per-tick C2/C3 tracking error, travel-limit flags) only
# ever existed as local variables inside bridge_master_slave.py's _tick(),
# with no ROS-visible trace. This does not change any control math -- it
# is read-only telemetry alongside the existing (unmodified) publishes.

std_msgs/Header header

# Anchor-relative positions (see startup_alignment.py): pos - anchor, in
# each device's own native frame, BEFORE the axis_map/scale used to cross
# from one frame to the other.
float64[3] master_relative_m
float64[3] slave_relative_m

# master_relative run through apply_axis_map+axis_scale (what C3 adds to
# right_anchor); slave_relative run through the INVERSE map+scale (what C2
# adds to left_anchor). Same math bridge_master_slave.py's own _tick()/
# _update_c2() already compute -- this is that same value, exposed.
float64[3] mapped_master_relative_m
float64[3] mapped_slave_relative_m

# Real-time per-channel tracking error, NOT the same quantity as
# follow_error below (which is the scalar norm already used in reporting).
# c3_position_error = slave_target - slave_actual (does C3 place the slave
# where it was told to). c2_position_error = master_target - master_actual
# (does C2 place the master where it was told to) -- all-zero when C2 is
# disabled, since master_target is undefined/unused in that case.
float64[3] c2_position_error_m
float64[3] c3_position_error_m

# From startup_alignment.py, constant for the whole session once past
# VALIDATE_ALIGNMENT -- see that module's docstring for the precise
# geometric meaning (a sanity bound, not a "must be zero" precision gate).
float64[3] alignment_error_xyz_m
float64 alignment_error_norm_m

# scalar target-vs-actual distance for the C3 (master->slave) direction --
# the same value bridge_master_slave.py's own _maybe_report() already
# logs as text, now also machine-readable.
float64 follow_error_m

string bridge_state          # WAIT_FOR_POSES / HOLD_STILL / .../ ACTIVE / FAULT
# bridge_state's full range as of the 2026-09-09 staged arm/disarm
# transition: WAIT_FOR_POSES, HOLD_STILL, CAPTURE_ANCHORS,
# VALIDATE_ALIGNMENT, READY, GRAVITY_RAMP_UP, GRAVITY_SETTLE,
# COUPLING_RAMP_UP, ACTIVE, COUPLING_RAMP_DOWN, GRAVITY_RAMP_DOWN, FAULT --
# see startup_alignment.py's module docstring for why gravity compensation
# and bilateral position coupling are staged as separate phases rather than
# ramped simultaneously (root-cause finding: doing both at once let a large
# gravity-sagged slave/master separation combine with an unsupported-weight
# transient, driving real force/position clamps during the old single-stage
# RAMPING state).
float64 ramp_alpha
# Legacy alias, ALWAYS exactly equal to coupling_ramp_alpha below -- kept so
# existing consumers gating on `bridge_state=="ACTIVE" and ramp_alpha>=0.999`
# (e.g. the scientific recorder) continue to work unchanged; state only
# ever reaches ACTIVE once gravity_ramp_alpha has already reached 1.0, so
# this remains an unambiguous "is bilateral coupling fully engaged" signal.

float64 gravity_ramp_alpha    # 0->1 in GRAVITY_RAMP_UP, 1 through GRAVITY_SETTLE/COUPLING_RAMP_UP/ACTIVE/COUPLING_RAMP_DOWN/SELF_FOLLOW settle, 1->0 in GRAVITY_RAMP_DOWN
float64 coupling_ramp_alpha   # 0 through READY/GRAVITY_RAMP_UP/GRAVITY_SETTLE, 0->1 in COUPLING_RAMP_UP, 1 in ACTIVE, 1->0 in COUPLING_RAMP_DOWN

bool c2_enabled
bool c3_enabled               # 2026-09-10: genuinely gateable now, --c3-enable (default true)
bool c1_enabled                # 2026-09-10: declares a co-running C1 relay is expected (see --c1-enable)
bool c4_enabled                # 2026-09-10: declares a co-running C4 relay is expected (see --c4-enable)

bool master_travel_limit_flag  # C2's left_max_offset_m clamp engaged this tick
bool slave_travel_limit_flag   # C3's max_offset_xyz clamp engaged this tick (any axis)

# Bilateral force-budget-aware coupling planner (2026-09-09, production-
# integration task) -- see coupling_force_budget.py. Only meaningful while
# bridge_state==COUPLING_RAMP_UP; zero/false otherwise.
#
# 2026-09-09 Part 1 audit fix: the single master/slave_{force,position}
# _margin_* pair this used to be conflated TWO different quantities
# depending on whether the tick made progress (see coupling_force_budget.py's
# AdaptiveCouplingRamp docstring for the full explanation) -- a HELD tick's
# margin could be legitimately negative (that is WHY the candidate was
# rejected) without the actual PUBLISHED target ever being unsafe, which
# read as a false safety violation in diagnostics. Split into two explicit,
# always-separately-populated pairs:
#   accepted_*  -- margin of the CURRENTLY PUBLISHED target, recomputed
#       fresh every tick against live pose/velocity. This is the ONLY one
#       any hard safety assertion should check -- it must never be negative.
#   candidate_* -- margin of the last bisection trial attempted this tick
#       (diagnostic/explanatory only: why did/didn't alpha advance). CAN be
#       legitimately negative on a held tick; that is expected, not a fault.
float64 master_accepted_force_budget_margin_n
float64 slave_accepted_force_budget_margin_n
float64 master_accepted_position_error_margin_m
float64 slave_accepted_position_error_margin_m
float64 master_candidate_force_budget_margin_n
float64 slave_candidate_force_budget_margin_n
float64 master_candidate_position_error_margin_m
float64 slave_candidate_position_error_margin_m
bool coupling_ramp_held                # planner held alpha this tick (force/position budget binding)
float64 coupling_ramp_stall_time_s     # elapsed since last accepted alpha progress
bool force_budget_telemetry_fresh_left
bool force_budget_telemetry_fresh_right

# Planner's predicted pre-primary-clamp force for the candidate target it
# accepted this tick -- comparable directly against each arm's own
# CartesianControllerTelemetry.force_before_primary_clamp_n, which is how
# predictor-vs-controller equivalence is measured (2026-09-09).
float64[3] master_predicted_force_n
float64[3] slave_predicted_force_n

# Whether the bridge's DECLARED planner parameters (CLI args) agree with the
# controller's own telemetry-reported stiffness/damping/max_force/
# max_position_error. The planner USES the telemetry values (they are
# authoritative); these flags exist so a launch-config/rig disagreement is
# loud rather than silent, and a mismatch faults with
# COUPLING_RAMP_PARAMETER_MISMATCH rather than proceeding.
bool planner_parameter_match_left
bool planner_parameter_match_right

# GRAVITY_SETTLE observability (2026-09-09 Part 3/5, validation-
# infrastructure follow-up) -- see startup_alignment.py's GRAVITY_SETTLE
# tick() branch. Only meaningful while bridge_state==GRAVITY_SETTLE;
# 0/false otherwise. Purpose: let the recorder/live tests verify the
# velocity-gated quiet-hold is not passing on a pathological oscillation
# zero-crossing (near-zero velocity for an instant while still swinging
# through a large-amplitude motion) -- a velocity-only check cannot
# distinguish that from genuine rest without also watching pose drift.
float64 gravity_settle_master_speed_mps
float64 gravity_settle_slave_speed_mps
float64 gravity_settle_master_pose_drift_m   # drift over the CURRENT quiet window so far
float64 gravity_settle_slave_pose_drift_m
float64 gravity_settle_quiet_window_accumulated_s
# Frozen at the moment the gate passes (the "final accepted quiet window"),
# NOT reset afterward -- so a consumer reading this once bridge_state has
# moved past GRAVITY_SETTLE still sees what the settle decision was based on.
float64 gravity_settle_accepted_master_pose_drift_m
float64 gravity_settle_accepted_slave_pose_drift_m
