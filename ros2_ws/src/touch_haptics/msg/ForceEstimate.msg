# External force/torque estimated by the disturbance observer.
#
# SIGN CONVENTION, derived rather than assumed:
#   tau_dis_hat ~ Q(s) * [ tau_fric - tau_ext ]      (F_fric = 0 in the model)
#   =>  tau_ext_hat = -tau_dis_hat
# On the MASTER this is the operator's hand torque; on the SLAVE it is the
# environment's. Because the observer's tau_cmd already includes everything the
# arm renders, the estimate is independent of the arm's own output -- which is
# what makes 4-channel possible on this rig with no force sensors.
#
# Joint order is [waist, shoulder, elbow] (J1, J2, J3) throughout.

std_msgs/Header header

# External JOINT torque, N*m. This is the primary quantity -- the observer is a
# joint-space estimator, and the bilateral coupling is joint-space, so nothing
# needs a Jacobian to consume it.
float64[3] tau_ext_nm

# The raw observer output before sign inversion and deadband, for debugging.
float64[3] tau_dis_hat_nm

# Deadbanded in JOINT space, where threshold_nm was actually calibrated. The
# shrinking form sign*max(0,|tau|-db) is used rather than a hard gate: a hard
# gate produces a step, and a step into a human hand is an energy injection.
float64[3] tau_ext_deadbanded_nm

# Per-joint confidence, 0..1. Folds deadband margin and estimate validity.
# A consumer should scale or refuse the channel on low confidence rather than
# treating every sample as equally trustworthy.
float64[3] confidence

# True only when the observer is configured AND its parameters were loaded from
# an identified fit for THIS arm. False when parameters are borrowed from the
# other arm during bring-up -- see borrowed_params.
bool valid

# Set when the loaded M_n/G(q) came from a different physical arm. The estimate
# is then larger and pose-dependent but not wrong in sign: usable to feel a
# wall, not to quote a number. Exists so a borrowed-parameter floor is never
# mistaken for a hardware fault.
bool borrowed_params
