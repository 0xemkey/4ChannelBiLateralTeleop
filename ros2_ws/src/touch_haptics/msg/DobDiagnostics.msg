# Disturbance-observer diagnostics, one sample per publish tick.
#
# OBSERVATION ONLY. Nothing here feeds the command path: while
# TOUCH_DOB_ENABLE_TORQUE_WRITE is 0 the observer's correct() output is never
# written to the device, and this message exists so the estimate can be watched
# and recorded without arming anything.
#
# Joint order throughout is [waist, shoulder, elbow] (J1, J2, J3). The gimbal
# (J4-J6) is passive and has no observer.

std_msgs/Header header

# Disturbance estimate, N*m. With gravity modelled this is friction, residual
# gravity-model error, off-diagonal inertial coupling, and noise -- NOT a
# contact force. It sits at a nonzero floor even in free motion; the offline
# replay at g_dis = 2*pi*5 predicted roughly 0.065 / 0.046 / 0.037 N*m.
float64[3] tau_dis_hat_nm

# Gate state: |tau_dis_hat| above threshold_nm. Only as trustworthy as that
# threshold, which was derived from the floor rather than from known contact.
bool[3] contact_suspected

# Single backward difference of joint angles, rad/s. Quantisation-noisy by
# construction; the observer's own low-pass is what suppresses it.
float64[3] qdot_rad_s

# Driving torque handed to the observer, N*m. Reconstructed as J^T(q)*F_cmd
# from the PREVIOUS frame, not read from HD_LAST_JOINT_TORQUE -- see the note
# in touch_slave.cpp on why the raw register cannot be used uncalibrated.
float64[3] tau_cmd_nm

# Joint angles this frame, rad. Recorded so a bag can be re-analysed offline
# against the same gravity map the observer used.
float64[3] q_rad

# True once the bank has been configured from the identified JSON. When false
# every field above is zero and the observer is inert.
bool configured

# Joint-space PD breakdown, added 2026-09-03 to diagnose C4's tracking gap
# (see touch_slave.cpp's joint_space_control_ branch and the // TODO: [C4]
# comment there). Populated only when the joint-space PD branch actually
# runs this frame (joint_space_control_ && dob_bank_.configured() &&
# !sing.near_singular) -- zero otherwise, same convention as `configured`
# above. Lets a bag or a live subscriber separate gravity feedforward from
# the error/damping term and from the post-clamp value actually written,
# instead of only ever seeing their sum.
bool joint_pd_active

# gravityTorqueNm(q2, q3) alone, before the error/damping/Coulomb terms are
# added -- this is the value the C4 TODO's coupling hypothesis is about.
float64[3] joint_pd_gravity_ff_nm

# The Kp*err - Kd*qdot_filt + Coulomb block's contribution only, i.e. what
# gets added when joint_target_fresh. Zero if joint_target was stale this
# frame even though joint_pd_active is true.
float64[3] joint_pd_error_term_nm

# tau_cmd AFTER prepareJointTorqueCommand's clamp, i.e. what was actually
# handed to hdSetDoublev(HD_CURRENT_JOINT_TORQUE, ...) this frame (subject
# to joint_pd_write_ok below -- if that is false this was NOT written).
float64[3] joint_pd_cmd_nm

# Whether prepareJointTorqueCommand returned true this frame. False means
# joint_pd_cmd_nm was computed but refused (uncalibrated/unconfigured/
# gate closed) and nothing was actually written this frame -- distinguishes
# "refused" from "clamped to the ceiling".
bool joint_pd_write_ok
