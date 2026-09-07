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
float64 ramp_alpha
bool c2_enabled
bool c3_enabled               # always true whenever this bridge runs at all

bool master_travel_limit_flag  # C2's left_max_offset_m clamp engaged this tick
bool slave_travel_limit_flag   # C3's max_offset_xyz clamp engaged this tick (any axis)
