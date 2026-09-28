#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <geometry_msgs/msg/wrench_stamped.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/int32.hpp>

#include <HD/hd.h>

#include "touch_haptics/touch_dob.hpp"
#include "touch_haptics/touch_force_estimate.hpp"
#include "touch_haptics/dob_json_loader.hpp"

#include "touch_haptics/msg/dob_diagnostics.hpp"
#include "touch_haptics/msg/force_estimate.hpp"
#include "touch_haptics/msg/cartesian_controller_telemetry.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <mutex>
#include <stdexcept>
#include <string>
#include <vector>

using namespace std::chrono_literals;

// HD_CURRENT_JOINT_TORQUE unit fix, found and fixed 2026-09-03. Every torque
// value in this file's joint-space path (tau_cmd, gravityTorqueNm,
// dob_torque_f_max_n, fric_coulomb_nm, ...) is N*m throughout -- matches the
// dynamics literature this project's own math follows, and matches the sim's
// own hd_sim_api.cpp (its hdSetDoublev/hdGetDoublev for this parameter store
// and return the value as-is, into a variable literally named cmd_joint_nm,
// consumed as N*m by its own physics). The REAL OpenHaptics HDAPI does not
// share that convention: the vendor reference guide (OpenHaptics_ProgGuide.pdf,
// Table 6-2, "New Joint Torque Parameters in OpenHaptics 3.4.0") documents
// HD_CURRENT_JOINT_TORQUE's units as mNm (milli-Newton-meters), not Nm. This
// was never caught by any sim test, because the sim was only ever checked for
// self-consistency with this file's own N*m assumption, never against the real
// API's actual documented unit contract -- there was no real-hardware joint-
// torque write to catch it against until 2026-09-03's first live C1 test,
// where a torque comparable in magnitude to something clearly felt via the
// Cartesian HD_CURRENT_FORCE path (which IS in N, matching this file's own
// units, and has worked correctly all along) produced literally nothing --
// because every value this file ever wrote was received by the device as that
// many milli-newton-meters, roughly 1000x too small to ever be felt.
// TOUCH_HAPTICS_REAL_HD_HARDWARE is defined only for real-hardware targets
// (CMakeLists.txt's add_touch_node, not add_touch_sim_node) specifically so
// this conversion applies to the real device and NEVER to the sim, which must
// keep receiving N*m to stay self-consistent with its own physics.
#if TOUCH_HAPTICS_REAL_HD_HARDWARE
constexpr double kJointTorqueNmToDeviceUnits = 1000.0;  // N*m -> mN*m
#else
constexpr double kJointTorqueNmToDeviceUnits = 1.0;  // sim's own contract is N*m
#endif

namespace
{

struct DeviceState
{
  std::array<double, 3> pos_m{0.0, 0.0, 0.0};
  std::array<double, 3> vel_mps{0.0, 0.0, 0.0};
  std::array<double, 4> quat_xyzw{0.0, 0.0, 0.0, 1.0};
  std::array<double, 3> rpy_rad{0.0, 0.0, 0.0};
  std::array<double, 3> applied_force_n{0.0, 0.0, 0.0};
  int buttons{0};
  bool clutch{false};
  rclcpp::Time stamp{0, 0, RCL_ROS_TIME};

  // Telemetry. These are COMMANDED values read back from the driver, not
  // measurements: the Touch/Omni has optical encoders on its joints and no
  // force or torque sensor anywhere. HD_CURRENT_JOINT_TORQUE is listed by
  // OpenHaptics as both a get and a set parameter precisely because it is the
  // command channel to the three base motors. On a passive-gimbal device such
  // as the Omni/Touch, gimbal_torque stays zero.
  // HD_LAST_FORCE: the previous frame's force AFTER the driver's own pipeline
  // (max-force clamping, ramping, one-frame limiting). Comparing it with what we
  // set is the only way to see the driver modifying our command.
  std::array<double, 3> last_force_n{0.0, 0.0, 0.0};
  std::array<double, 3> joint_torque_nm{0.0, 0.0, 0.0};
  std::array<double, 3> gimbal_torque_nm{0.0, 0.0, 0.0};
  std::array<double, 3> joint_angles_rad{0.0, 0.0, 0.0};
  std::array<double, 3> motor_temperature{0.0, 0.0, 0.0};
  // target - actual, unclamped. Zero unless a fresh target_pose is present.
  // With a known spring gain this is the only usable proxy for external load:
  // if the tip cannot reach its commanded target, something is resisting it.
  std::array<double, 3> tracking_error_m{0.0, 0.0, 0.0};
  bool tracking_valid{false};

  // Observer snapshot. Written in the servo callback under mtx_ and published
  // from publishTick, deliberately NOT published from the callback itself:
  // rclcpp publish can allocate and take locks, and a stalled servo frame is
  // the one failure this file consistently designs against. The estimate
  // updates at the servo rate; the topic carries it at publish_hz, which is
  // ample for watching a floor and for a bag.
  std::array<double, 3> dob_tau_dis_hat_nm{0.0, 0.0, 0.0};
  std::array<double, 3> dob_qdot_rad_s{0.0, 0.0, 0.0};
  std::array<double, 3> dob_tau_cmd_nm{0.0, 0.0, 0.0};
  std::array<bool, 3> dob_contact{false, false, false};
  // Kept separately from joint_angles_rad, which is only populated when
  // publish_telemetry_ is on. The observer publishes independently of that
  // flag, so it cannot borrow a field that may never be written.
  std::array<double, 3> dob_q_rad{0.0, 0.0, 0.0};
  bool dob_configured{false};

  // ForceEstimate snapshot, computed once per hdCallback tick via
  // touch_force_estimate.hpp and published from publishTick like the rest of
  // this struct -- see the comment above dob_tau_dis_hat_nm for why nothing
  // publishes directly from the servo callback.
  std::array<double, 3> fe_tau_ext_nm{0.0, 0.0, 0.0};
  std::array<double, 3> fe_tau_dis_hat_nm{0.0, 0.0, 0.0};
  std::array<double, 3> fe_tau_ext_deadbanded_nm{0.0, 0.0, 0.0};
  std::array<double, 3> fe_confidence{0.0, 0.0, 0.0};
  bool fe_valid{false};
  bool fe_borrowed_params{false};

  // Whether the joint-space PD path actually commanded the device this frame
  // (vs. the Cartesian path). Diagnostic only.
  bool joint_path_commanded{false};

  // Joint-space PD breakdown, added 2026-09-03 to diagnose C4's tracking
  // gap -- see the // TODO: [C4] comment at this file's joint_space_control_
  // branch and DobDiagnostics.msg's matching fields for what each captures.
  bool joint_pd_active{false};
  touch_haptics::Vec3 joint_pd_gravity_ff_nm{0.0, 0.0, 0.0};
  touch_haptics::Vec3 joint_pd_error_term_nm{0.0, 0.0, 0.0};
  touch_haptics::Vec3 joint_pd_cmd_nm{0.0, 0.0, 0.0};
  bool joint_pd_write_ok{false};

  // Exact servo-frame Cartesian decomposition. Telemetry only: these fields
  // snapshot values already used by the command path and never feed it.
  std::array<double, 3> target_position_m{0.0, 0.0, 0.0};
  std::array<double, 3> position_error_clamped_m{0.0, 0.0, 0.0};
  std::array<double, 3> spring_force_n{0.0, 0.0, 0.0};
  std::array<double, 3> damping_force_n{0.0, 0.0, 0.0};
  std::array<double, 3> auxiliary_force_n{0.0, 0.0, 0.0};
  std::array<double, 3> force_before_primary_clamp_n{0.0, 0.0, 0.0};
  std::array<double, 3> force_after_primary_clamp_n{0.0, 0.0, 0.0};
  std::array<double, 3> relay_torque_received_nm{0.0, 0.0, 0.0};
  std::array<double, 3> relay_force_n{0.0, 0.0, 0.0};
  std::array<double, 3> force_before_final_clamp_n{0.0, 0.0, 0.0};
  bool enabled{false}, deadman_ok{false}, target_valid{false}, target_fresh{false};
  bool force_valid{false}, force_fresh{false}, relay_valid{false}, relay_fresh{false};
  bool relay_applied{false}, position_error_clamped{false};
  bool primary_force_clamped{false}, final_force_clamped{false}, near_singular{false};
};

struct CommandState
{
  bool enabled{false};
  bool target_valid{false};
  bool force_valid{false};
  std::array<double, 3> target_pos_m{0.0, 0.0, 0.0};
  std::array<double, 3> force_cmd_n{0.0, 0.0, 0.0};
  std::chrono::steady_clock::time_point last_target_time{};
  std::chrono::steady_clock::time_point last_force_time{};

  // Joint-space bilateral inputs (0.5). Independent staleness tracking from
  // target_pos_m/force_cmd_n above -- these feed the joint-space PD path,
  // not the Cartesian spring/damper.
  bool joint_target_valid{false};
  std::array<double, 3> joint_target_rad{0.0, 0.0, 0.0};
  std::chrono::steady_clock::time_point last_joint_target_time{};

  bool joint_torque_ext_valid{false};
  std::array<double, 3> joint_torque_ext_deadbanded_nm{0.0, 0.0, 0.0};
  std::chrono::steady_clock::time_point last_joint_torque_ext_time{};
};

static inline double clampScalar(double value, double lo, double hi)
{
  return std::min(std::max(value, lo), hi);
}

static inline void clampNorm(std::array<double, 3> &v, double max_norm)
{
  const double norm = std::sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
  if (norm > max_norm && norm > 1e-12) {
    const double scale = max_norm / norm;
    v[0] *= scale;
    v[1] *= scale;
    v[2] *= scale;
  }
}

static inline void quatFromRotMat(
  double r00, double r01, double r02,
  double r10, double r11, double r12,
  double r20, double r21, double r22,
  double &qx, double &qy, double &qz, double &qw)
{
  double tr = r00 + r11 + r22;
  if (tr > 0.0) {
    double s = std::sqrt(tr + 1.0) * 2.0;
    qw = 0.25 * s;
    qx = (r21 - r12) / s;
    qy = (r02 - r20) / s;
    qz = (r10 - r01) / s;
  } else if ((r00 > r11) && (r00 > r22)) {
    double s = std::sqrt(1.0 + r00 - r11 - r22) * 2.0;
    qw = (r21 - r12) / s;
    qx = 0.25 * s;
    qy = (r01 + r10) / s;
    qz = (r02 + r20) / s;
  } else if (r11 > r22) {
    double s = std::sqrt(1.0 + r11 - r00 - r22) * 2.0;
    qw = (r02 - r20) / s;
    qx = (r01 + r10) / s;
    qy = 0.25 * s;
    qz = (r12 + r21) / s;
  } else {
    double s = std::sqrt(1.0 + r22 - r00 - r11) * 2.0;
    qw = (r10 - r01) / s;
    qx = (r02 + r20) / s;
    qy = (r12 + r21) / s;
    qz = 0.25 * s;
  }

  double n = std::sqrt(qx * qx + qy * qy + qz * qz + qw * qw);
  if (n > 0.0) {
    qx /= n;
    qy /= n;
    qz /= n;
    qw /= n;
  } else {
    qx = 0.0;
    qy = 0.0;
    qz = 0.0;
    qw = 1.0;
  }
}

static inline void rpyFromQuat(double qx, double qy, double qz, double qw, double &roll, double &pitch, double &yaw)
{
  const double sinr_cosp = 2.0 * (qw * qx + qy * qz);
  const double cosr_cosp = 1.0 - 2.0 * (qx * qx + qy * qy);
  roll = std::atan2(sinr_cosp, cosr_cosp);

  const double sinp = 2.0 * (qw * qy - qz * qx);
  if (std::abs(sinp) >= 1.0) {
    pitch = std::copysign(M_PI / 2.0, sinp);
  } else {
    pitch = std::asin(sinp);
  }

  const double siny_cosp = 2.0 * (qw * qz + qx * qy);
  const double cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz);
  yaw = std::atan2(siny_cosp, cosy_cosp);
}

}  // namespace

class TouchSlave : public rclcpp::Node
{
public:
  TouchSlave() : Node("touch_slave")
  {
    device_name_ = declare_parameter<std::string>("device", "DEFAULT");
    frame_id_ = declare_parameter<std::string>("frame_id", "touch_base");
    publish_hz_ = declare_parameter<int>("publish_hz", 200);

    pos_scale_ = declare_parameter<double>("pos_scale", 0.001);
    invert_x_ = declare_parameter<bool>("invert_x", false);
    invert_y_ = declare_parameter<bool>("invert_y", false);
    invert_z_ = declare_parameter<bool>("invert_z", false);
    swap_xy_ = declare_parameter<bool>("swap_xy", false);

    stiffness_n_per_m_ = declare_parameter<double>("stiffness_n_per_m", 8.0);
    damping_n_s_per_m_ = declare_parameter<double>("damping_n_s_per_m", 0.08);
    max_force_n_ = declare_parameter<double>("max_force_n", 0.45);
    // Separate from max_force_n_ above (the Cartesian spring/damper's own
    // clamp): this drives derivePerJointTorqueLimits() for the joint-space
    // PD path's per-joint torque ceiling. Was 0.6 N (reproducing the exact
    // values, 0.162/0.081/0.081 N*m, that were hardcoded in configureDob()
    // before 2026-09-02) until 2026-09-03, when C1's torque-headroom test
    // (sim/tests/test_c1_torque_headroom.py) found elbow's gravity
    // feedforward alone consuming 75-94% of that ceiling at 8 of the 10
    // calibrated poses in gravity_left_minimal.json (worst case 94%,
    // ~5.6% headroom) -- silently swallowing any C1 signal added on top,
    // the same failure mode independently found live on real hardware for
    // shoulder that same day. Raised to 0.9 N: elbow's worst-case headroom
    // at that value is 37.1% (computed directly from the same calibration
    // data), clearing the test's 30% bar with real margin, not just barely.
    // Widens every joint's ceiling (waist/shoulder too, not just elbow --
    // there is no cheaper way to change one joint's clamp without a
    // separate per-joint ceiling, which was considered and deferred, see
    // FOUR_CHANNEL_TODO.md's 2026-09-03 section) -- a deliberate,
    // real-hardware-affecting decision, not a silent tuning bump. Exists as
    // its own parameter (not reusing max_force_n_) because the two clamps
    // protect different things at different times (Cartesian spring output
    // vs. joint-space PD+gravity+friction output) and conflating them would
    // silently change one when tuning the other.
    dob_torque_f_max_n_ = declare_parameter<double>("dob_torque_f_max_n", 0.9);
    // Scientific-controller Phase 4B sweep hook, added 2026-09-08: overrides
    // the g_dis values below (default -1.0 = "no override, use the shipped
    // per-joint kGDis array unchanged"). Simulation-only use so far
    // (analysis/scientific_4ch/phase4b_*); does not change default behavior
    // on any existing launch (all defaults are -1.0). NOT a claim that any
    // override value is better -- purely a sweep capability, see
    // docs/SCIENTIFIC_DOB_VALIDATION.md Phase 4B.
    dob_g_dis_override_rad_s_ = declare_parameter<double>("dob_g_dis_override_rad_s", -1.0);
    // Preferred scientific sweep mechanism as of 2026-09-10: a per-joint
    // BANDWIDTH SCALE rather than a uniform absolute override -- scale=1.5
    // on the {10,10,5} baseline gives {15,15,7.5}, preserving the paper's
    // per-joint ratio (waist=shoulder=2x elbow) instead of collapsing it to
    // one value the way dob_g_dis_override_rad_s_ does. Default 1.0 =
    // shipped kGDis array unchanged. Kept independent of the override param
    // for backward compatibility; see configureDob() for precedence (the
    // override, if set, still wins -- it is a different, coarser sweep tool,
    // not superseded).
    dob_g_dis_scale_ = declare_parameter<double>("dob_g_dis_scale", 1.0);
    max_position_error_m_ = declare_parameter<double>("max_position_error_m", 0.04);
    stale_timeout_s_ = declare_parameter<double>("stale_timeout_s", 0.05);
    require_deadman_ = declare_parameter<bool>("require_deadman", true);
    deadman_button_mask_ = declare_parameter<int>("deadman_button_mask", HD_DEVICE_BUTTON_1);
    clutch_button_mask_ = declare_parameter<int>("clutch_button_mask", HD_DEVICE_BUTTON_2);
    enabled_on_start_ = declare_parameter<bool>("enabled_on_start", false);
    publish_telemetry_ = declare_parameter<bool>("publish_telemetry", false);

    // Observer parameter files. Defaults point at what the identification
    // pipeline writes; overridable so a second arm can use its own fits.
    dob_inertia_path_ = declare_parameter<std::string>(
      "dob_inertia_json", std::string(getenv("HOME") ? getenv("HOME") : ".") +
                          "/dob_logs/results/inertia_left.json");
    dob_gravity_path_ = declare_parameter<std::string>(
      "dob_gravity_json", std::string(getenv("HOME") ? getenv("HOME") : ".") +
                          "/dob_logs/results/gravity_left_minimal.json");
    dob_enable_ = declare_parameter<bool>("dob_enable", true);

    // Joint-space PD + C1-C4 (0.5). Off by default: sim behaviour is
    // unchanged unless a launch file explicitly asks for it. On a normal
    // hardware build this parameter can be set true and nothing will happen
    // anyway -- TOUCH_DOB_ENABLE_TORQUE_WRITE is 0 there, so writeAuthorized()
    // always refuses regardless of any runtime flag configureBilateral()
    // sets. Only a build configured with -DTOUCH_HAPTICS_ENABLE_REAL_TORQUE_
    // WRITE=ON (see CMakeLists.txt, off by default, a deliberate opt-in)
    // compiles the write-armed branch into the real touch_slave binary.
    joint_space_control_ = declare_parameter<bool>("joint_space_control", false);
    // Kp retuned 2026-08-27: the old {6.0, 3.0, 2.0} saturated
    // clampJointTorque()'s per-joint ceiling (derivePerJointTorqueLimits() at
    // f_max=0.6N: waist=0.162, shoulder/elbow=0.081 N*m) at only ~1.5 degrees
    // of error, making this a permanently-saturated bang-bang controller
    // rather than a PD one for any realistic target offset. Confirmed live in
    // sim/tests/test_joint_space_write_gated.py: with the old gains the arm
    // overshot a small commanded offset by 5.33x and drove the shoulder onto
    // its hard limit (-0.583 rad against a -0.6 rad stop) instead of settling
    // near the target. New Kp saturates at ~15-17 degrees instead (sized as
    // max_torque_nm / 0.3 rad per joint), which is the actual, dominant fix;
    // Kd is left as-is because at the new, much smaller Kp it already sits
    // above critical damping (Kd_crit = 2*sqrt(Kp*M_n)), which is the safe
    // direction to be wrong in. CORRECTED 2026-09-04: this was previously
    // claimed as "~2x overdamped" here; verified against the real identified
    // M_n (inertia_left.json) it is actually 6-8x:
    //
    //     joint      M_n (kg*m^2)   Kd_crit    Kd     Kd/Kd_crit
    //     waist      0.003369       0.0636     0.50   7.86x
    //     shoulder   0.002945       0.0485     0.30   6.18x
    //     elbow      0.000233       0.0159     0.10   6.31x
    //
    // Still safely overdamped (direction unchanged), just far more sluggish
    // than "~2x" implied. NOT the cause of the historical C4 test failure --
    // that was a test-methodology bug (see the RESOLVED [C4] note below,
    // joint_target going stale during the test's own spin_for() windows),
    // confirmed 2026-09-04 by re-running test_joint_space_write_gated.py
    // clean at 85.2% closed_fraction with these same Kd values unchanged.
    // This ratio describes the decoupled nominal joint-space model only
    // (M_n*qddot = Kp*err - Kd*qdot, gravity/friction/Coulomb/DOB terms enter
    // as additive feedforward/disturbance, not as cross-terms here) and does
    // not by itself say anything about the production Cartesian-relay C1/C4
    // path, which does not use kp_nm_per_rad_/kd_nms_per_rad_ at all.
    kp_nm_per_rad_ = declare_parameter<std::vector<double>>(
      "kp_nm_per_rad", std::vector<double>{0.30, 0.20, 0.27});
    kd_nms_per_rad_ = declare_parameter<std::vector<double>>(
      "kd_nms_per_rad", std::vector<double>{0.50, 0.30, 0.10});
    if (kp_nm_per_rad_.size() != 3 || kd_nms_per_rad_.size() != 3) {
      RCLCPP_FATAL(get_logger(),
                   "kp_nm_per_rad/kd_nms_per_rad must have exactly 3 elements; "
                   "joint_space_control forced off");
      joint_space_control_ = false;
      kp_nm_per_rad_ = {0.30, 0.20, 0.27};
      kd_nms_per_rad_ = {0.50, 0.30, 0.10};
    }

    {
      std::lock_guard<std::mutex> lock(mtx_);
      command_.enabled = enabled_on_start_;
    }

    pose_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>("pose", 10);
    rpy_pub_ = create_publisher<geometry_msgs::msg::Vector3Stamped>("rpy", 10);
    buttons_pub_ = create_publisher<std_msgs::msg::Int32>("buttons", 10);
    clutch_pub_ = create_publisher<std_msgs::msg::Bool>("clutch", 10);
    applied_force_pub_ = create_publisher<geometry_msgs::msg::WrenchStamped>("applied_force", 10);

    if (publish_telemetry_) {
      last_force_pub_ = create_publisher<geometry_msgs::msg::WrenchStamped>("last_force", 10);
      joint_torque_pub_ = create_publisher<geometry_msgs::msg::Vector3Stamped>("joint_torque", 10);
      gimbal_torque_pub_ = create_publisher<geometry_msgs::msg::Vector3Stamped>("gimbal_torque", 10);
      joint_angles_pub_ = create_publisher<geometry_msgs::msg::Vector3Stamped>("joint_angles", 10);
      motor_temperature_pub_ = create_publisher<geometry_msgs::msg::Vector3Stamped>("motor_temperature", 10);
      tracking_error_pub_ = create_publisher<geometry_msgs::msg::Vector3Stamped>("tracking_error", 10);
    }

    target_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      "target_pose", 10, std::bind(&TouchSlave::targetPoseCallback, this, std::placeholders::_1));
    force_sub_ = create_subscription<geometry_msgs::msg::WrenchStamped>(
      "force_cmd", 10, std::bind(&TouchSlave::forceCommandCallback, this, std::placeholders::_1));
    enable_sub_ = create_subscription<std_msgs::msg::Bool>(
      "enable", 10, std::bind(&TouchSlave::enableCallback, this, std::placeholders::_1));

    // C2/C3: the peer arm's joint angles, crossing Jacobian-free because both
    // arms share identical kinematics -- see Part 3.1 of the bilateral plan.
    joint_target_sub_ = create_subscription<geometry_msgs::msg::Vector3Stamped>(
      "joint_target", 10,
      std::bind(&TouchSlave::jointTargetCallback, this, std::placeholders::_1));
    // C1/C4: the peer arm's weighted external-torque contribution, relayed by
    // bilateral_4ch.py from that arm's own force_ext_hat.
    joint_torque_ext_sub_ = create_subscription<touch_haptics::msg::ForceEstimate>(
      "joint_torque_ext_cmd", 10,
      std::bind(&TouchSlave::jointTorqueExtCallback, this, std::placeholders::_1));

    dob_diag_pub_ =
      create_publisher<touch_haptics::msg::DobDiagnostics>("dob_diagnostics", 10);
    force_ext_hat_pub_ =
      create_publisher<touch_haptics::msg::ForceEstimate>("force_ext_hat", 10);
    cartesian_telemetry_pub_ =
      create_publisher<touch_haptics::msg::CartesianControllerTelemetry>(
        "cartesian_controller_telemetry", 10);
    initDevice();

    // AFTER initDevice(): configureDob() asks the driver for HD_UPDATE_RATE to
    // get the servo period, and that query needs an open device. Configuring
    // first silently fell back to an assumed rate -- small in effect, but it
    // would have made the live filter differ from the offline replay for a
    // reason that had nothing to do with the hardware.
    configureDob();
    // AFTER configureDob(): dob_cfg_ is populated there (M_n, g_dis,
    // threshold, max_torque). configureBilateral() only flips the three
    // commissioning flags on top of that already-loaded config.
    configureBilateral();

    auto period = std::chrono::duration<double>(1.0 / std::max(1, publish_hz_));
    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      std::bind(&TouchSlave::publishTick, this));

    RCLCPP_WARN(
      get_logger(),
      "touch_slave started with max_force=%.3f N stiffness=%.3f N/m damping=%.3f Ns/m require_deadman=%s",
      max_force_n_, stiffness_n_per_m_, damping_n_s_per_m_, require_deadman_ ? "true" : "false");
  }

  ~TouchSlave() override
  {
    shutdownDevice();
  }

private:
  // ---------------------------------------------------------------------
  // Load identified parameters and configure the observer. OBSERVATION ONLY:
  // this arms nothing. TOUCH_DOB_ENABLE_TORQUE_WRITE remains 0, so
  // prepareJointTorqueCommand() still refuses and the Cartesian path stays
  // the live command. Configuring the bank only makes tau_dis_hat available
  // to watch.
  //
  // A failure here is logged and the node continues: an unconfigured bank is
  // inert (update() is a no-op, estimates stay zero), so the arm still
  // teleoperates normally. Refusing to start would trade a working robot for
  // a diagnostic, which is the wrong way round.
  // ---------------------------------------------------------------------
  void configureDob()
  {
    if (!dob_enable_) {
      RCLCPP_WARN(get_logger(), "DOB disabled by parameter dob_enable:=false");
      return;
    }

    touch_haptics::DobConfig cfg{};
    std::string err;
    if (!touch_haptics::dob_json::loadIdentifiedParams(
          dob_inertia_path_, dob_gravity_path_, &cfg, &err)) {
      RCLCPP_FATAL(get_logger(), "DOB parameter load FAILED: %s", err.c_str());
      RCLCPP_FATAL(get_logger(),
                   "observer stays unconfigured; teleoperation is unaffected");
      return;
    }

    // Cutoff and thresholds, updated 2026-08-25 to the reference paper's band.
    //
    // g_dis = {10, 10, 5} rad/s is the paper's per-joint choice on this exact
    // hardware. An earlier draft here used a uniform 2*pi*5 rad/s (5 Hz), which
    // was ~3x faster and sat almost exactly on the slave's 4.4 Hz mechanical
    // resonance -- so the observer passed that peak essentially unattenuated,
    // and feeding it to a master force channel would close a loop through it.
    //
    // Measured floors from tools/dob_replay_cpp.cpp over ~/dob_logs/dyn_Left.csv
    // (p99 |tau_dis_hat|, N*m, gravity compensated):
    //
    //     g_dis        waist    shoulder    elbow
    //     2pi*5.0 Hz   0.0654    0.0458     0.0373     <- previous setting
    //     10 rad/s     0.0382    0.0320     0.0305
    //      5 rad/s     0.0255    0.0298     0.0250
    //
    // As a fraction of each joint's torque authority at f_max = 0.6 N that is
    // 40/57/46 % before, 24/40/31 % now. The elbow is given the lower cutoff
    // because its floor keeps falling below 1.6 Hz (18 % from 5 Hz to 1.59 Hz,
    // another 18 % to 0.8 Hz) -- an earlier analysis here claimed that joint was
    // insensitive to g_dis, which a linear extrapolation had got wrong.
    //
    // COST: the elbow's time constant is 200 ms and the other two are 100 ms,
    // against 32 ms before. Contact onset is perceived correspondingly later.
    // The paper offsets this with a neural-network contact detector; we have no
    // equivalent, so the lag is simply accepted for now.
    double kGDis[3]      = {10.0, 10.0, 5.0};             // rad/s
    const double kThreshold[3] = {0.07632, 0.06400, 0.05000};   // 2 x p99, independent of f_max
    // Was a hardcoded {0.162, 0.081, 0.081} array (f_max=0.6N) until
    // 2026-09-02; now derived at runtime from dob_torque_f_max_n_ (default
    // 0.6N, so this reproduces the exact same numbers unless overridden).
    const touch_haptics::Vec3 kMaxTorque =
      touch_haptics::derivePerJointTorqueLimits(dob_torque_f_max_n_);
    if (dob_g_dis_override_rad_s_ > 0.0) {
      // Phase 4B sweep hook: uniform override across all 3 joints, matching
      // the historical single-value hardware comparison table this session
      // cites (touch_slave.cpp's own g_dis comment above). threshold_nm is
      // intentionally NOT re-derived here (still the shipped per-joint
      // values) -- Phase 4B measures raw tau_dis_hat_nm magnitude directly,
      // which does not depend on threshold_nm; only the contact GATE would,
      // and this sweep does not use the gate. Takes precedence over
      // dob_g_dis_scale_ below -- it is a coarser, uniform-value sweep tool
      // that predates the scale and is kept working unchanged; a caller
      // that sets both is explicitly asking for the absolute override.
      for (int i = 0; i < 3; ++i) kGDis[i] = dob_g_dis_override_rad_s_;
    } else if (dob_g_dis_scale_ != 1.0) {
      // Preferred sweep mechanism (2026-09-10): scales the shipped per-joint
      // baseline rather than collapsing it to one value, so scale=1.5 gives
      // {15,15,7.5} -- same threshold_nm caveat as the override above (not
      // re-derived; this sweep does not use the contact gate either).
      for (int i = 0; i < 3; ++i) kGDis[i] *= dob_g_dis_scale_;
    }
    for (int i = 0; i < touch_haptics::kNumActiveJoints; ++i) {
      cfg.joints[i].g_dis_rad_s = kGDis[i];
      cfg.joints[i].threshold_nm = kThreshold[i];
      cfg.joints[i].max_torque_nm = kMaxTorque[i];
      cfg.joints[i].gate_release_ratio = 1.0;
    }

    // tau_cmd is handed to the bank already in N*m (reconstructed as
    // J^T(q)*F_cmd), so the raw-to-N*m conversion factor is exactly 1. See
    // the long note at the update() call site for why the raw
    // HD_LAST_JOINT_TORQUE register is NOT used here.
    cfg.last_joint_torque_scale_nm_per_unit = 1.0;

    // The servo loop runs at the HDAPI scheduler rate, not publish_hz_. Ask
    // the driver rather than assuming: measured 1019 Hz on this unit, and a
    // wrong period would mis-tune the filter's discretisation.
    HDint rate = 0;
    hdGetIntegerv(HD_UPDATE_RATE, &rate);
    const double servo_hz = (rate > 0) ? static_cast<double>(rate) : 1000.0;
    if (rate <= 0) {
      RCLCPP_WARN(get_logger(),
                  "HD_UPDATE_RATE unavailable; assuming %.0f Hz for the DOB period",
                  servo_hz);
    }

    if (!dob_bank_.configure(cfg, 1.0 / servo_hz)) {
      const char *why = cfg.joints[0].invalidReason();
      RCLCPP_FATAL(get_logger(), "DOB bank refused to configure: %s",
                   why ? why : "unknown");
      return;
    }
    dob_frames_since_configure_ = 0;
    if (std::getenv("TOUCH_DIAG_DOB_RESTART_TRANSIENT") != nullptr) {
      // 2026-09-04 restart-transient investigation: reset() (called
      // unconditionally inside configure()) guarantees lpf_/u_prev_ are
      // exactly 0.0 here by construction -- logged anyway per this
      // project's own "verified empirically, not assumed" convention
      // rather than trusted from reading the header alone.
      for (int i = 0; i < touch_haptics::kNumActiveJoints; ++i) {
        std::fprintf(stderr,
                      "[diag_dob_restart] joint=%d AT CONFIGURE lpf=%.6f u_prev=%.6f "
                      "tau_dis_hat=%.6f\n",
                      i, dob_bank_.joint(i).lpfState(), dob_bank_.joint(i).uPrevState(),
                      dob_bank_.joint(i).tauDisHatNm());
      }
    }
    // Same measured period dob_bank_ was configured with -- reused below for
    // the joint-space PD path's OWN filtered-velocity estimate (see
    // qdot_filt_rad_s_), which is a separate consumer from the observer's
    // internal qdot and does not touch dob_bank_'s math at all.
    dob_servo_period_s_ = 1.0 / servo_hz;

    dob_cfg_ = cfg;
    // Report the ACTUAL per-joint cutoffs. This line previously hardcoded a
    // single "5.0 Hz" and would have kept printing it after the values changed
    // -- a log that lies about configuration is worse than no log, because it
    // is what gets pasted into a session record.
    RCLCPP_INFO(get_logger(),
                "DOB bank configured: servo=%.0f Hz  "
                "g_dis(w/s/e)=%.1f/%.1f/%.1f rad/s = %.2f/%.2f/%.2f Hz",
                servo_hz,
                cfg.joints[0].g_dis_rad_s, cfg.joints[1].g_dis_rad_s,
                cfg.joints[2].g_dis_rad_s,
                cfg.joints[0].g_dis_rad_s / (2.0 * M_PI),
                cfg.joints[1].g_dis_rad_s / (2.0 * M_PI),
                cfg.joints[2].g_dis_rad_s / (2.0 * M_PI));
    RCLCPP_INFO(get_logger(),
                "  threshold(w/s/e)=%.4f/%.4f/%.4f N*m  max_torque=%.4f/%.4f/%.4f N*m",
                cfg.joints[0].threshold_nm, cfg.joints[1].threshold_nm,
                cfg.joints[2].threshold_nm, cfg.joints[0].max_torque_nm,
                cfg.joints[1].max_torque_nm, cfg.joints[2].max_torque_nm);
    RCLCPP_INFO(get_logger(),
                "  M_n(w/s/e)=%.4e/%.4e/%.4e kg*m^2  gravity model loaded",
                cfg.joints[0].m_n_kg_m2, cfg.joints[1].m_n_kg_m2,
                cfg.joints[2].m_n_kg_m2);
    RCLCPP_INFO(get_logger(),
                "  friction compensation (w/s/e): %s/%s/%s "
                "(c_coulomb=%.4f/%.4f/%.4f N*m  b_visc=%.4f/%.4f/%.4f N*m*s/rad)",
                cfg.joints[0].frictionModelLoaded() ? "on" : "off",
                cfg.joints[1].frictionModelLoaded() ? "on" : "off",
                cfg.joints[2].frictionModelLoaded() ? "on" : "off",
                cfg.joints[0].fric_coulomb_nm, cfg.joints[1].fric_coulomb_nm,
                cfg.joints[2].fric_coulomb_nm, cfg.joints[0].b_visc_nm_s_per_rad,
                cfg.joints[1].b_visc_nm_s_per_rad, cfg.joints[2].b_visc_nm_s_per_rad);
    RCLCPP_WARN(get_logger(),
                "  OBSERVATION ONLY -- torque write remains gated off; "
                "tau_dis_hat is published, never applied");
  }

  // ---------------------------------------------------------------------
  // Arms the joint-space PD + C1-C4 write path (0.5). Only ever takes effect
  // where TOUCH_DOB_ENABLE_TORQUE_WRITE=1 was set at compile time -- on
  // touch_slave (hardware) that define does not exist, so writeAuthorized()
  // returns false regardless of what this function sets, and the code below
  // is dead weight there by construction, not by this flag. See touch_dob.hpp
  // and the CMakeLists.txt comment next to touch_slave_sim's
  // target_compile_definitions.
  //
  // Called only when joint_space_control_ is true, so the sim's default
  // behaviour (Cartesian, exactly as today) is unchanged unless a launch file
  // explicitly opts in.
  // ---------------------------------------------------------------------
  void configureBilateral()
  {
    if (!joint_space_control_) {
      return;
    }
    if (!dob_bank_.configured()) {
      RCLCPP_ERROR(get_logger(),
                   "joint_space_control requested but the DOB bank is not "
                   "configured (see the DOB parameter load log above); "
                   "joint-space write stays gated off");
      return;
    }
    // last_joint_torque_scale_nm_per_unit is already 1.0 (set in
    // configureDob()): tau_cmd here is always reconstructed as J^T(q)*F or
    // Kp*(q_target-q)-Kd*qdot, never read from the raw register, so
    // torqueScaleCalibrated() is satisfied without a real calibration sweep.
    dob_cfg_.calibration_a_complete = true;
    dob_cfg_.clamp_b_complete = true;
    dob_cfg_.enable_torque_write = true;
    RCLCPP_WARN(get_logger(),
                "joint_space_control enabled -- joint-torque write path armed "
                "(effective only on a TOUCH_DOB_ENABLE_TORQUE_WRITE=1 build, "
                "i.e. touch_slave_sim)");
  }

  void initDevice()
  {
    if (device_name_ == "DEFAULT") {
      hHD_ = hdInitDevice(HD_DEFAULT_DEVICE);
    } else {
      hHD_ = hdInitDevice(device_name_.c_str());
    }

    HDErrorInfo error = hdGetError();
    if (HD_DEVICE_ERROR(error)) {
      throw std::runtime_error(std::string("hdInitDevice failed: ") + hdGetErrorString(error.errorCode));
    }

    hdEnable(HD_FORCE_OUTPUT);
    hdStartScheduler();

    error = hdGetError();
    if (HD_DEVICE_ERROR(error)) {
      throw std::runtime_error(std::string("hdStartScheduler failed: ") + hdGetErrorString(error.errorCode));
    }

    callback_handle_ = hdScheduleAsynchronous(&TouchSlave::hdCallback, this, HD_MAX_SCHEDULER_PRIORITY);
  }

  void shutdownDevice()
  {
    {
      std::lock_guard<std::mutex> lock(mtx_);
      command_.enabled = false;
      state_.applied_force_n = {0.0, 0.0, 0.0};
    }

    if (callback_handle_ != HD_INVALID_HANDLE) {
      hdUnschedule(callback_handle_);
      callback_handle_ = HD_INVALID_HANDLE;
    }
    hdStopScheduler();
    if (hHD_ != HD_INVALID_HANDLE) {
      hdDisableDevice(hHD_);
      hHD_ = HD_INVALID_HANDLE;
    }
  }

  void targetPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(mtx_);
    command_.target_pos_m = {
      msg->pose.position.x,
      msg->pose.position.y,
      msg->pose.position.z,
    };
    command_.target_valid = true;
    command_.last_target_time = std::chrono::steady_clock::now();
  }

  void forceCommandCallback(const geometry_msgs::msg::WrenchStamped::SharedPtr msg)
  {
    std::array<double, 3> force{
      msg->wrench.force.x,
      msg->wrench.force.y,
      msg->wrench.force.z,
    };
    clampNorm(force, max_force_n_);

    std::lock_guard<std::mutex> lock(mtx_);
    command_.force_cmd_n = force;
    command_.force_valid = true;
    command_.last_force_time = std::chrono::steady_clock::now();
  }

  void enableCallback(const std_msgs::msg::Bool::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(mtx_);
    command_.enabled = msg->data;
  }

  // C2/C3 input: the peer arm's joint angles, crossing as raw radians per the
  // Jacobian-free architecture (both arms share kinematics).
  void jointTargetCallback(const geometry_msgs::msg::Vector3Stamped::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(mtx_);
    command_.joint_target_rad = {msg->vector.x, msg->vector.y, msg->vector.z};
    command_.joint_target_valid = true;
    command_.last_joint_target_time = std::chrono::steady_clock::now();
  }

  // C1/C4 input: the peer arm's weighted, deadbanded external torque, relayed
  // by bilateral_4ch.py from that arm's own force_ext_hat. Only
  // tau_ext_deadbanded_nm is consumed -- the rest of ForceEstimate exists so
  // the wire format needs no duplication, not because every field feeds the
  // controller.
  void jointTorqueExtCallback(const touch_haptics::msg::ForceEstimate::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(mtx_);
    command_.joint_torque_ext_deadbanded_nm = {
      msg->tau_ext_deadbanded_nm[0],
      msg->tau_ext_deadbanded_nm[1],
      msg->tau_ext_deadbanded_nm[2],
    };
    command_.joint_torque_ext_valid = true;
    command_.last_joint_torque_ext_time = std::chrono::steady_clock::now();
  }

  static HDCallbackCode HDCALLBACK hdCallback(void *user_data)
  {
    auto *self = static_cast<TouchSlave *>(user_data);

    hdBeginFrame(self->hHD_);

    // Report the driver's own force pipeline once, from inside a frame where
    // these queries are valid. Anything enabled here can change our command
    // between hdSetDoublev and the motors.
    if (!self->capabilities_logged_) {
      self->capabilities_logged_ = true;
      double nominal_max = 0.0, nominal_cont = 0.0, ramp_rate = 0.0;
      hdGetDoublev(HD_NOMINAL_MAX_FORCE, &nominal_max);
      hdGetDoublev(HD_NOMINAL_MAX_CONTINUOUS_FORCE, &nominal_cont);
      hdGetDoublev(HD_FORCE_RAMPING_RATE, &ramp_rate);
      RCLCPP_WARN(
        self->get_logger(),
        "device force pipeline: nominal_max=%.3f N nominal_continuous=%.3f N ramping_rate=%.3f",
        nominal_max, nominal_cont, ramp_rate);
      RCLCPP_WARN(
        self->get_logger(),
        "driver capabilities: MAX_FORCE_CLAMPING=%s FORCE_RAMPING=%s "
        "SOFTWARE_FORCE_LIMIT=%s ONE_FRAME_LIMIT=%s",
        hdIsEnabled(HD_MAX_FORCE_CLAMPING) ? "ON" : "off",
        hdIsEnabled(HD_FORCE_RAMPING) ? "ON" : "off",
        hdIsEnabled(HD_SOFTWARE_FORCE_LIMIT) ? "ON" : "off",
        hdIsEnabled(HD_ONE_FRAME_LIMIT) ? "ON" : "off");
      if (self->max_force_n_ > nominal_max && nominal_max > 0.0) {
        RCLCPP_ERROR(
          self->get_logger(),
          "max_force_n=%.3f N exceeds the device nominal max of %.3f N; commands will be clipped",
          self->max_force_n_, nominal_max);
      }
    }

    double transform[16];
    double velocity_mm_s[3];
    int buttons = 0;
    hdGetDoublev(HD_CURRENT_TRANSFORM, transform);
    hdGetDoublev(HD_CURRENT_VELOCITY, velocity_mm_s);
    hdGetIntegerv(HD_CURRENT_BUTTONS, &buttons);

    const double raw_x_m = transform[12] * self->pos_scale_;
    const double raw_y_m = transform[13] * self->pos_scale_;
    const double raw_z_m = transform[14] * self->pos_scale_;
    const double raw_vx_mps = velocity_mm_s[0] * self->pos_scale_;
    const double raw_vy_mps = velocity_mm_s[1] * self->pos_scale_;
    const double raw_vz_mps = velocity_mm_s[2] * self->pos_scale_;

    std::array<double, 3> pos_m = self->mapDeviceVectorToRos({raw_x_m, raw_y_m, raw_z_m});
    std::array<double, 3> vel_mps = self->mapDeviceVectorToRos({raw_vx_mps, raw_vy_mps, raw_vz_mps});

    double qx, qy, qz, qw;
    quatFromRotMat(
      transform[0], transform[1], transform[2],
      transform[4], transform[5], transform[6],
      transform[8], transform[9], transform[10],
      qx, qy, qz, qw);

    double roll, pitch, yaw;
    rpyFromQuat(qx, qy, qz, qw, roll, pitch, yaw);

    CommandState command;
    {
      std::lock_guard<std::mutex> lock(self->mtx_);
      command = self->command_;
    }

    const auto now = std::chrono::steady_clock::now();
    const bool target_fresh =
      command.target_valid &&
      std::chrono::duration<double>(now - command.last_target_time).count() <= self->stale_timeout_s_;
    const bool force_fresh =
      command.force_valid &&
      std::chrono::duration<double>(now - command.last_force_time).count() <= self->stale_timeout_s_;
    const bool deadman_ok =
      !self->require_deadman_ || ((buttons & self->deadman_button_mask_) != 0);

    std::array<double, 3> tracking_error{0.0, 0.0, 0.0};
    std::array<double, 3> clamped_error{0.0, 0.0, 0.0};
    std::array<double, 3> spring_force{0.0, 0.0, 0.0};
    std::array<double, 3> damping_force{0.0, 0.0, 0.0};
    std::array<double, 3> auxiliary_force{0.0, 0.0, 0.0};
    bool tracking_valid = false;
    bool position_error_clamped = false;
    bool primary_force_clamped = false;

    std::array<double, 3> force_ros{0.0, 0.0, 0.0};
    if (command.enabled && deadman_ok) {
      if (target_fresh) {
        std::array<double, 3> err{
          command.target_pos_m[0] - pos_m[0],
          command.target_pos_m[1] - pos_m[1],
          command.target_pos_m[2] - pos_m[2],
        };
        tracking_error = err;
        tracking_valid = true;
        for (double &v : err) {
          position_error_clamped |= std::abs(v) > self->max_position_error_m_;
          v = clampScalar(v, -self->max_position_error_m_, self->max_position_error_m_);
        }
        clamped_error = err;
        for (int i = 0; i < 3; ++i) {
          spring_force[i] = self->stiffness_n_per_m_ * err[i];
          damping_force[i] = -self->damping_n_s_per_m_ * vel_mps[i];
          force_ros[i] += spring_force[i] + damping_force[i];
        }
      }
      if (force_fresh) {
        auxiliary_force = command.force_cmd_n;
        for (int i = 0; i < 3; ++i) force_ros[i] += auxiliary_force[i];
      }
      primary_force_clamped = std::sqrt(force_ros[0]*force_ros[0] + force_ros[1]*force_ros[1] + force_ros[2]*force_ros[2]) > self->max_force_n_;
      clampNorm(force_ros, self->max_force_n_);
    }
    const std::array<double, 3> force_before_primary_clamp{
      spring_force[0] + damping_force[0] + auxiliary_force[0],
      spring_force[1] + damping_force[1] + auxiliary_force[1],
      spring_force[2] + damping_force[2] + auxiliary_force[2]};
    const auto force_after_primary_clamp = force_ros;

    // Not const: the C1-onto-Cartesian relay below (added 2026-09-03) adds to
    // this after q_rad/sing become available, further down the function.
    std::array<double, 3> force_device = self->mapRosForceToDevice(force_ros);

    // ---------------------------------------------------------------------
    // Joint-space path (DOB). GATED OFF -- see touch_dob.hpp prerequisites.
    // ---------------------------------------------------------------------
    // While the gate is closed this block only observes and logs; the Cartesian
    // write below remains the live command path. That ordering is deliberate:
    // replacing the Cartesian write with a gated joint write would leave the
    // device receiving no force at all whenever the gate is closed, i.e. the
    // slave would go limp. Exactly one of the two paths commands the device on
    // any given frame, and the Cartesian one is the default.
    //
    // q and tau_cmd are read here, unconditionally, BEFORE the force write.
    // Two reasons they are not in the publish_telemetry_ block below:
    //   - the observer needs them every frame regardless of telemetry state;
    //   - HD_LAST_JOINT_TORQUE must be sampled before this frame's write so it
    //     still refers to the previous frame's command, which is the frame that
    //     JointDobBank's one-sample velocity buffer aligns it against.
    double dob_q[3] = {0.0, 0.0, 0.0};
    double dob_tau_raw[3] = {0.0, 0.0, 0.0};
    hdGetDoublev(HD_CURRENT_JOINT_ANGLES, dob_q);
    hdGetDoublev(HD_LAST_JOINT_TORQUE, dob_tau_raw);

    const touch_haptics::Vec3 q_rad{dob_q[0], dob_q[1], dob_q[2]};

    // WHY THE RAW REGISTER IS READ BUT NOT USED AS tau_cmd
    // ---------------------------------------------------------------------
    // HD_LAST_JOINT_TORQUE is in raw device units whose conversion to N*m has
    // never been calibrated -- hdapi_torque_calib.cpp has not been run, and
    // last_joint_torque_scale_nm_per_unit has no measured value. Comparing the
    // logged raw column against J^T(q)*F_cmd over the dynamic captures puts the
    // factor around 6e-4 to 1e-3 N*m per raw unit, i.e. raw is roughly a
    // thousand times larger. Feeding raw with a scale of 1.0 would inflate
    // tau_cmd by that factor and tau_dis_hat with it -- the predicted 0.065
    // N*m floor would read in the tens of N*m, and nothing could be compared
    // against the offline replay.
    //
    // So tau_cmd is reconstructed the way the whole identification pipeline
    // reconstructs it, from a force we chose and therefore know exactly:
    //
    //     tau = J^T(q) * F_commanded,   F in newtons  ->  real N*m
    //
    // and the scale is set to exactly 1.0 because the value handed over is
    // already in N*m. This is the same construction tools/dob_replay_cpp.cpp
    // used to produce the numbers this run is meant to reproduce.
    //
    // THE ONE-FRAME LAG: the bank pairs its tau_cmd argument with q_prev_,
    // because on hardware HD_LAST_JOINT_TORQUE reports the command issued one
    // frame earlier. The reconstruction has to carry the same lag or the
    // alignment the bank was built around would be defeated -- hence
    // dob_prev_tau_nm_, computed last frame at last frame's pose. The raw
    // register is still read above and republished for comparison, so the
    // scale can be recovered from a bag once hdapi_torque_calib has run.
    self->dob_bank_.update(q_rad, self->dob_prev_tau_nm_);
    ++self->dob_frames_since_configure_;

    // 2026-09-04 restart-transient investigation: the leading hypothesis is
    // NOT the filter state (reset() guarantees lpf_/u_prev_ start at 0.0,
    // confirmed at the configure()-time trace above) but a backward-
    // difference qdot spike on the first primed frame right after a device
    // re-init, if HD_CURRENT_JOINT_ANGLES returns a stale/garbage first
    // reading before the driver's internal buffer has actually refreshed --
    // that would alias directly into tau_dis_hat via g*M_n*qdot (see the
    // Q-filter derivation in touch_dob.hpp). Traces the first 10 frames only,
    // env-gated, zero cost when unset -- matches TOUCH_DIAG_C4_ELBOW's
    // existing pattern in this file.
    constexpr uint64_t kRestartTraceFrames = 10;
    if (self->dob_frames_since_configure_ <= kRestartTraceFrames &&
        std::getenv("TOUCH_DIAG_DOB_RESTART_TRANSIENT") != nullptr) {
      const auto &qdot = self->dob_bank_.qdotRadS();
      std::fprintf(stderr,
                    "[diag_dob_restart] frame=%lu q_rad=(%.5f,%.5f,%.5f) "
                    "qdot=(%.5f,%.5f,%.5f) tau_cmd_in=(%.5f,%.5f,%.5f) "
                    "tau_dis_hat=(%.5f,%.5f,%.5f) "
                    "lpf=(%.5f,%.5f,%.5f) u_prev=(%.5f,%.5f,%.5f)\n",
                    static_cast<unsigned long>(self->dob_frames_since_configure_),
                    q_rad[0], q_rad[1], q_rad[2],
                    qdot[0], qdot[1], qdot[2],
                    self->dob_prev_tau_nm_[0], self->dob_prev_tau_nm_[1], self->dob_prev_tau_nm_[2],
                    self->dob_bank_.joint(0).tauDisHatNm(),
                    self->dob_bank_.joint(1).tauDisHatNm(),
                    self->dob_bank_.joint(2).tauDisHatNm(),
                    self->dob_bank_.joint(0).lpfState(),
                    self->dob_bank_.joint(1).lpfState(),
                    self->dob_bank_.joint(2).lpfState(),
                    self->dob_bank_.joint(0).uPrevState(),
                    self->dob_bank_.joint(1).uPrevState(),
                    self->dob_bank_.joint(2).uPrevState());
    }

    {
      const touch_haptics::Mat3 J_now = touch_haptics::jacobian(q_rad);
      self->dob_prev_tau_nm_ =
        touch_haptics::jacobianTransposeTimes(J_now, force_device);
    }

    const touch_haptics::SingularityMetrics sing =
      touch_haptics::singularityMetrics(q_rad);

    // ForceEstimate, computed unconditionally whenever the bank is configured
    // -- this feeds force_ext_hat regardless of which write path is live, so
    // the peer arm can consume it even while this arm still runs Cartesian.
    touch_haptics::ForceEstimateResult fe{};
    if (self->dob_bank_.configured()) {
      fe = touch_haptics::computeForceEstimate(
        self->dob_bank_.tauDisHatNm(), self->dob_cfg_, true);
    }

    bool joint_path_commanded = false;
    // Joint-space PD diagnostic breakdown (DobDiagnostics), captured only
    // inside the joint_space_control_ branch below -- stays at these
    // defaults (inactive, all zero) whenever that branch doesn't run.
    bool joint_pd_active = false;
    touch_haptics::Vec3 joint_pd_gravity_ff_out{0.0, 0.0, 0.0};
    touch_haptics::Vec3 joint_pd_error_term_out{0.0, 0.0, 0.0};
    touch_haptics::Vec3 joint_pd_cmd_out{0.0, 0.0, 0.0};
    bool joint_pd_write_ok = false;
    if (self->joint_space_control_ && self->dob_bank_.configured() && !sing.near_singular) {
      joint_pd_active = true;
      // Joint-space PD (0.5), replacing the Cartesian-derived reference below
      // when explicitly requested. tau_pd uses THIS frame's q/qdot -- qdotRadS()
      // returns qdot_prev_, which dob_bank_.update() just refreshed to the
      // current frame's backward difference (see the reassignment at the end
      // of JointDobBank::update()).
      const std::array<double, touch_haptics::kNumActiveJoints> &qdot =
        self->dob_bank_.qdotRadS();

      const bool joint_target_fresh =
        command.joint_target_valid &&
        std::chrono::duration<double>(now - command.last_joint_target_time).count() <=
          self->stale_timeout_s_;
      const bool joint_ext_fresh =
        command.joint_torque_ext_valid &&
        std::chrono::duration<double>(now - command.last_joint_torque_ext_time).count() <=
          self->stale_timeout_s_;

      // Gravity feedforward, added 2026-08-27. Without this the PD term alone
      // has to hold the arm against its own weight using only error*Kp -- and
      // Kp is deliberately small (see the comment at kp_nm_per_rad_'s default)
      // to avoid saturating the tiny per-joint torque clamp. Confirmed live:
      // with no feedforward the shoulder (this arm's most gravity-loaded
      // joint per the Phase 2 floor analysis -- "shoulder floor is 142% of
      // torque authority") got pulled onto its hard limit regardless of Kp,
      // because nothing was fighting gravity itself, only the small residual
      // position error. Same sign convention as everywhere else in this file:
      // "tau = M*qddot + C*qdot + G(q) => at rest, tau_cmd EQUALS G(q))"
      // (touch_dob.hpp's JointDobParams comment), so this is added, not
      // subtracted. Safe to call unconditionally: this branch is gated on
      // dob_bank_.configured(), and JointDobParams::valid() already requires
      // gravityModelLoaded(), so grav_coef is never the NaN sentinel here.
      touch_haptics::Vec3 tau_cmd{0.0, 0.0, 0.0};
      for (int i = 0; i < touch_haptics::kNumActiveJoints; ++i) {
        tau_cmd[i] = self->dob_cfg_.joints[i].gravityTorqueNm(q_rad[1], q_rad[2]);
      }
      // Snapshot for DobDiagnostics -- gravity feedforward alone, before the
      // error/damping/Coulomb terms below are added. See the comment on
      // DobDiagnostics.msg's joint_pd_gravity_ff_nm.
      joint_pd_gravity_ff_out = tau_cmd;
      // Filtered velocity, joint-space PD path only -- SEPARATE from
      // dob_bank_'s own internal qdot_prev_ (used above for the Kd term,
      // untouched, still raw). A single-pole low-pass with a ~30ms time
      // constant, updated every tick regardless of joint_target_fresh so it
      // stays current. This does NOT cascade with the observer's own
      // Q-filter (touch_dob.hpp's "do not add a second smoothing filter on
      // qdot" warning is about tau_dis_hat's estimation path specifically,
      // where two lags in series were shown to pump); this is a different
      // consumer entirely -- gating a control-law feedforward term below,
      // never fed back into dob_bank_ or the observer's own math.
      constexpr double kVelFilterTauS = 0.03;
      const double filt_alpha =
        1.0 - std::exp(-self->dob_servo_period_s_ / kVelFilterTauS);
      for (int i = 0; i < touch_haptics::kNumActiveJoints; ++i) {
        self->qdot_filt_rad_s_[i] =
          (1.0 - filt_alpha) * self->qdot_filt_rad_s_[i] + filt_alpha * qdot[i];
      }

      // Coulomb breakaway feedforward, re-added 2026-09-02 after the
      // 2026-09-02 revert (see git history / FOUR_CHANNEL_TODO.md for that
      // attempt's limit-cycle finding). Gravity feedforward alone leaves
      // friction uncompensated, so Kp*err drops below the Coulomb threshold
      // near a small residual error and the joint simply sticks. The
      // reverted attempt gated this on RAW qdot, which spikes to a fixed,
      // large magnitude (~0.4 rad/s on this arm) on a single encoder tick
      // even when the joint is genuinely at rest -- indistinguishable from
      // real motion by a threshold on the raw signal, and the resulting
      // rapid open/close gate chatter (not the feedforward magnitude
      // itself) is what sustained the limit cycle: a term that keeps
      // toggling on and off at high frequency injects energy at that
      // frequency regardless of how small the deadband is. Gating on the
      // filtered velocity above instead removes the chatter -- a real
      // quantization tick is smoothed to a small, slowly-varying value over
      // the ~30ms window, while genuine sustained motion still shows up
      // clearly, so the gate opens and closes at most once per real
      // stall/breakaway event instead of every servo tick.
      if (joint_target_fresh) {
        for (int i = 0; i < touch_haptics::kNumActiveJoints; ++i) {
          const double err = command.joint_target_rad[i] - q_rad[i];
          // Kd term uses the filtered velocity (qdot_filt_rad_s_), not the
          // raw backward-difference qdot -- raw qdot spikes ~0.4 rad/s from
          // encoder quantization even at rest (see the filter's own comment
          // above), and waist's Kd=0.5 was turning that into visible,
          // motion-unrelated torque jumps on real hardware (found live
          // 2026-09-03, first real C1 session). The observer's own qdot
          // (used elsewhere, e.g. the DOB's Kd-equivalent) stays raw and
          // untouched -- this filtered value was already being computed for
          // the Coulomb gate below and never wired into this term.
          double err_term = self->kp_nm_per_rad_[i] * err -
                             self->kd_nms_per_rad_[i] * self->qdot_filt_rad_s_[i];
          constexpr double kErrSignEpsRad = 0.015;
          constexpr double kStictionQdotThreshRadS = 0.05;
          if (std::abs(self->qdot_filt_rad_s_[i]) < kStictionQdotThreshRadS) {
            err_term += self->dob_cfg_.joints[i].fric_coulomb_nm *
                        std::tanh(err / kErrSignEpsRad);
          }
          tau_cmd[i] += err_term;
          joint_pd_error_term_out[i] = err_term;
        }
        // RESOLVED [C4], 2026-09-03. The limit cycle was genuinely fixed
        // (smooth, isolated drive converges cleanly), but
        // test_write_armed_tracks_joint_target kept failing at roughly -60%
        // to -75%. Two hypotheses were checked against direct evidence and
        // rejected before the real cause was found -- recorded here so
        // neither gets re-tried:
        //   1. Inter-joint INERTIAL coupling (mass-matrix off-diagonal
        //      terms) -- WRONG: arm_model.hpp's m_coupling is zero by
        //      default, no validated M(q) with off-diagonal terms exists
        //      anywhere in this repo.
        //   2. Gravity-feedforward coupling (gravityTorqueNm(q2, q3) being
        //      a function of shoulder angle for every joint, so elbow's
        //      feedforward shifts as shoulder moves) -- also WRONG, once
        //      actually measured: added joint_pd_gravity_ff_nm/
        //      joint_pd_error_term_nm/joint_pd_cmd_nm to DobDiagnostics.msg
        //      and logged elbow's breakdown live (see
        //      sim/tests/diag_c4_gravity_coupling.py and the
        //      TOUCH_DIAG_C4_ELBOW=1 stderr trace a few lines below). The
        //      data showed joint_pd_error_term_nm sitting at EXACTLY 0.0 for
        //      70-190 consecutive servo ticks in every failing run, always
        //      at the same three points -- meaning joint_target_fresh was
        //      false, not that gravity FF was overpowering a live PD term.
        //   The actual cause: test_joint_space_write_gated.py's own
        //   _drive_and_measure() called q_c.spin_for(0.1) / spin_for(0.2)
        //   at the hold->drive transition and right before reading the
        //   final position -- both windows never call target_pub.publish(),
        //   and stale_timeout_s_'s 0.05s default is short enough that
        //   joint_target_fresh goes false during them. With no live PD
        //   term, gravity feedforward alone (which cancels gravity at the
        //   CURRENT pose, but does nothing to track a moving target) let
        //   the arm drift freely -- and the second gap sits immediately
        //   before q_final is read, so a real chunk of the measured
        //   "failure" was this drift, not the control law's actual
        //   tracking. bilateral_4ch.py (the real production path) has no
        //   equivalent gap -- it publishes joint_target unconditionally
        //   every tick -- so this was a test methodology bug, not a C4
        //   control-law bug. Fixed in test_joint_space_write_gated.py
        //   (_publish_and_settle() keeps publishing during both windows);
        //   no change to the control law itself was needed. See
        //   FOUR_CHANNEL_TODO.md's 2026-09-03 section for the full
        //   diagnostic writeup.
      }
      if (joint_ext_fresh) {
        for (int i = 0; i < touch_haptics::kNumActiveJoints; ++i) {
          tau_cmd[i] += command.joint_torque_ext_deadbanded_nm[i];
        }
      }

      std::string why;
      joint_pd_write_ok = touch_haptics::prepareJointTorqueCommand(self->dob_cfg_, tau_cmd, &why);
      // prepareJointTorqueCommand mutates tau_cmd in place (clamps it, or
      // zeroes it on refusal) -- capture AFTER the call so joint_pd_cmd_out
      // reflects what was actually written (or the zeroed refusal), never
      // the pre-clamp value.
      joint_pd_cmd_out = tau_cmd;
      if (joint_pd_write_ok) {
        // Convert N*m (this file's own units, joint_pd_cmd_out above stays
        // in them for diagnostics) to the device's actual mN*m on real
        // hardware only -- see kJointTorqueNmToDeviceUnits's comment.
        const touch_haptics::Vec3 tau_device{
          tau_cmd[0] * kJointTorqueNmToDeviceUnits,
          tau_cmd[1] * kJointTorqueNmToDeviceUnits,
          tau_cmd[2] * kJointTorqueNmToDeviceUnits,
        };
        hdSetDoublev(HD_CURRENT_JOINT_TORQUE, tau_device.data());
        joint_path_commanded = true;
      }

      // Diagnostic-only, gated by TOUCH_DIAG_C4_ELBOW=1: server-side stderr
      // trace for the C4 gravity-coupling investigation (2026-09-03), added
      // because a client-side ROS subscription to dob_diagnostics -- even
      // carefully rate-limited -- was found to measurably perturb this
      // scenario's own closed-loop outcome (confirmed empirically: same
      // code, same scenario, consistently ~+0.85 with a diagnostic
      // subscriber attached vs. a consistent -0.60 to -0.65 cluster from
      // the real pytest run with none). A plain stderr fprintf sidesteps
      // that entirely -- same pattern as arm_model.hpp's own "[hd_sim]
      // init-pose seed:" debug line. Elbow only (index 2), this
      // investigation's specific question; not meant to stay long-term.
      if (std::getenv("TOUCH_DIAG_C4_ELBOW") != nullptr) {
        std::fprintf(stderr,
                     "[diag_c4] q3=%.5f err=%.5f grav_ff=%.5f err_term=%.5f "
                     "cmd=%.5f ok=%d\n",
                     q_rad[2], command.joint_target_rad[2] - q_rad[2],
                     joint_pd_gravity_ff_out[2], joint_pd_error_term_out[2],
                     joint_pd_cmd_out[2], joint_pd_write_ok ? 1 : 0);
      }
    } else if (self->dob_bank_.configured() && !sing.near_singular) {
      // Unchanged Cartesian-derived reference path, live whenever
      // joint_space_control_ is false (the default). tau_ref = J^T * F, with F
      // the SAME Cartesian wrench the Cartesian path below would have sent.
      // Taking it post-clampNorm is what makes the derived per-joint limits
      // valid, since derivePerJointTorqueLimits() assumes ||F|| <= max_force_n_.
      // Spring/damper and force_cmd are already summed into force_ros, and
      // J^T(A) + J^T(B) == J^T(A+B), so one multiply suffices -- no separate
      // tau_command_cmd term is needed.
      //
      // FRAME: force_device, not force_ros. J(q) is expressed in the paper's
      // Cartesian frame; mapRosForceToDevice() is the only mapping available
      // into device axes. Whether the paper's frame equals HDAPI's device frame
      // is NOT yet verified -- fkResidualVsMeasured() is the check, and it must
      // read small before this path is ever armed.
      const touch_haptics::Mat3 J = touch_haptics::jacobian(q_rad);
      touch_haptics::Vec3 tau_ref =
        touch_haptics::jacobianTransposeTimes(J, force_device);

      // Threshold-gated DOB correction, unchanged from touch_dob.hpp.
      touch_haptics::Vec3 tau_corrected = self->dob_bank_.correct(tau_ref);

      std::string why;
      if (touch_haptics::prepareJointTorqueCommand(self->dob_cfg_, tau_corrected, &why)) {
        // Unreachable until prerequisites (a) and (b) are done and both flags
        // are set by hand. Present so the path is reviewable, not so it runs.
        // Same N*m -> mN*m conversion as the joint-space branch above, see
        // kJointTorqueNmToDeviceUnits.
        const touch_haptics::Vec3 tau_device{
          tau_corrected[0] * kJointTorqueNmToDeviceUnits,
          tau_corrected[1] * kJointTorqueNmToDeviceUnits,
          tau_corrected[2] * kJointTorqueNmToDeviceUnits,
        };
        hdSetDoublev(HD_CURRENT_JOINT_TORQUE, tau_device.data());
        joint_path_commanded = true;
      }
    }

    // C1/C4-onto-Cartesian relay, added 2026-09-03, fixed same day. Lets
    // bilateral_4ch.py's C1 output (on the master, relaying the slave's
    // disturbance-observer force estimate) or C4 output (on the slave,
    // relaying the master's) contribute a force on the CARTESIAN write
    // path via joint_torque_ext_cmd, instead of requiring the separate
    // joint-space PD path (joint_space_control_:=true) exclusively.
    // Symmetric code, works on whichever node it runs on -- command.
    // joint_torque_ext_deadbanded_nm is populated from THIS node's own
    // joint_torque_ext_cmd subscription regardless of which channel that
    // happens to be for. Motivation: the joint-space path's own C2/C3
    // position tracking was found live to feel structurally worse than the
    // existing Cartesian bridge (bridge_master_slave.py) -- Kp/Kd acting on
    // joint angles rather than Cartesian x/y/z doesn't correspond
    // intuitively to felt motion the way the Cartesian spring/damper does,
    // a property of joint-space control, not a tunable bug. This block
    // lets C1/C4 ride alongside the Cartesian bridge's already-good
    // position feel instead.
    //
    // FIXED same day: the first version applied ANY fresh signal
    // unconditionally, including bilateral_4ch's own resting-state
    // residual (confirmed live: waist/elbow's disturbance estimate is
    // never exactly zero even with nothing touching the peer arm, ~0.0003-
    // 0.05 N*m). Continuously nudging THIS node's real Cartesian position
    // with that residual let bridge_master_slave.py's C2/C3 -- which reads
    // this node's live measured position to compute the peer's target --
    // mistake the nudge for deliberate motion, slowly dragging the peer in
    // an axis nobody intended. kMinRelayTorqueNormNm below requires the
    // raw joint torque to clear a real, deliberate-contact-sized floor
    // BEFORE any conversion happens, so resting noise never reaches the
    // Cartesian output at all -- not a downstream clamp on the result,
    // which wouldn't have prevented position-tracking contamination from a
    // small-but-nonzero force.
    //
    // Independent of joint_path_commanded/joint_space_control_'s own
    // branches above -- only meaningful (and only applied) when this node
    // is actually in Cartesian mode and not near a singularity, since
    // cartesianForceFromJointTorque's inversion requires that (see its own
    // precondition comment in touch_dob.hpp; det(J^T) == det(J), so the
    // existing near_singular check already covers it). ALSO gated on the
    // same command.enabled/deadman_ok this file's primary Cartesian force
    // already requires (computed above, in scope here) -- a relayed C1/C4
    // signal must not be able to apply force to a device the operator has
    // not enabled, the same safety property the spring/damper already has.
    std::array<double, 3> relay_torque{0.0, 0.0, 0.0};
    std::array<double, 3> relay_force_ros{0.0, 0.0, 0.0};
    std::array<double, 3> force_before_final_clamp_device = force_device;
    bool relay_fresh = false;
    bool relay_applied = false;
    bool final_force_clamped = false;
    if (!self->joint_space_control_ && !sing.near_singular &&
        command.enabled && deadman_ok) {
      const bool joint_ext_fresh_cartesian =
        command.joint_torque_ext_valid &&
        std::chrono::duration<double>(now - command.last_joint_torque_ext_time).count() <=
          self->stale_timeout_s_;
      relay_fresh = joint_ext_fresh_cartesian;
      if (joint_ext_fresh_cartesian) {
        const touch_haptics::Vec3 tau_ext{
          command.joint_torque_ext_deadbanded_nm[0],
          command.joint_torque_ext_deadbanded_nm[1],
          command.joint_torque_ext_deadbanded_nm[2],
        };
        relay_torque = tau_ext;
        // Floor sized from live observation: genuine resting-state
        // residual on the two joints C1 actually relays (waist, elbow --
        // shoulder is separately excluded upstream in bilateral_4ch.py)
        // topped out around 0.03-0.05 N*m; real deliberate contact reached
        // 0.03-0.4+ N*m during the same session's live testing. 0.02 sits
        // below the real-contact range and above most, not all, resting
        // noise -- a first cut, not a precision-tuned value; revisit with a
        // proper sim/hardware sweep before trusting it at the margin.
        constexpr double kMinRelayTorqueNormNm = 0.02;
        const double tau_norm = std::sqrt(
          tau_ext[0] * tau_ext[0] + tau_ext[1] * tau_ext[1] + tau_ext[2] * tau_ext[2]);
        if (tau_norm >= kMinRelayTorqueNormNm) {
          const touch_haptics::Mat3 J = touch_haptics::jacobian(q_rad);
          const touch_haptics::Vec3 f_ext =
            touch_haptics::cartesianForceFromJointTorque(J, tau_ext, sing.det);
          relay_force_ros = self->mapDeviceVectorToRos(f_ext);
          relay_applied = true;
          if (std::getenv("TOUCH_DIAG_CARTESIAN_RELAY") != nullptr) {
            std::fprintf(stderr,
                         "[diag_relay_math] tau_ext=(%.5f,%.5f,%.5f) "
                         "force_device_pre=(%.5f,%.5f,%.5f) "
                         "f_ext_added=(%.5f,%.5f,%.5f)\n",
                         tau_ext[0], tau_ext[1], tau_ext[2],
                         force_device[0], force_device[1], force_device[2],
                         f_ext[0], f_ext[1], f_ext[2]);
          }
          force_device[0] += f_ext[0];
          force_device[1] += f_ext[1];
          force_device[2] += f_ext[2];
          force_before_final_clamp_device = force_device;
          // Re-clamp in device units: the Cartesian spring/damper's own
          // clampNorm already ran on force_ros before mapRosForceToDevice,
          // but this addition happens after that -- re-apply the same
          // ceiling so max_force_n_'s safety guarantee holds regardless of
          // how large the relayed signal is.
          final_force_clamped = std::sqrt(force_device[0]*force_device[0] + force_device[1]*force_device[1] + force_device[2]*force_device[2]) > self->max_force_n_;
          clampNorm(force_device, self->max_force_n_);
        }
      }
    }

    if (std::getenv("TOUCH_DIAG_CARTESIAN_RELAY") != nullptr) {
      std::fprintf(stderr, "[diag_relay] joint_path_commanded=%d force_device=(%.5f,%.5f,%.5f)\n",
                   joint_path_commanded ? 1 : 0, force_device[0], force_device[1], force_device[2]);
    }
    // Cartesian path -- the live command whenever the joint path did not fire.
    if (!joint_path_commanded) {
      const double hd_force[3] = {force_device[0], force_device[1], force_device[2]};
      hdSetDoublev(HD_CURRENT_FORCE, hd_force);
    }

    // Read back the resulting motor commands. Done after the force is set so
    // the joint torque reflects this frame's command rather than the last one.
    // (True for HD_CURRENT_*; NOT true for HD_LAST_*, which is why the observer
    // samples HD_LAST_JOINT_TORQUE above instead of here.)
    double joint_torque[3] = {0.0, 0.0, 0.0};
    double gimbal_torque[3] = {0.0, 0.0, 0.0};
    double joint_angles[3] = {dob_q[0], dob_q[1], dob_q[2]};
    double motor_temperature[3] = {0.0, 0.0, 0.0};
    double last_force[3] = {0.0, 0.0, 0.0};
    if (self->publish_telemetry_) {
      // Previous frame's force, after the driver's clamping and ramping. Read in
      // device axes and mapped back so it is comparable with applied_force.
      hdGetDoublev(HD_LAST_FORCE, last_force);
      hdGetDoublev(HD_CURRENT_JOINT_TORQUE, joint_torque);
      // Inverse of kJointTorqueNmToDeviceUnits: the device reports back in
      // its own units (mN*m on real hardware, N*m in sim), convert to N*m
      // here so the `joint_torque` telemetry topic stays in this file's own
      // units on both, same as every other torque value it publishes.
      for (double &v : joint_torque) {
        v /= kJointTorqueNmToDeviceUnits;
      }
      hdGetDoublev(HD_CURRENT_GIMBAL_TORQUE, gimbal_torque);
      hdGetDoublev(HD_MOTOR_TEMPERATURE, motor_temperature);
    }

    hdEndFrame(self->hHD_);

    HDErrorInfo error = hdGetError();
    if (!HD_DEVICE_ERROR(error)) {
      std::lock_guard<std::mutex> lock(self->mtx_);
      self->state_.pos_m = pos_m;
      self->state_.vel_mps = vel_mps;
      self->state_.quat_xyzw = {qx, qy, qz, qw};
      self->state_.rpy_rad = {roll, pitch, yaw};
      // BUG FIXED 2026-09-04: this used to read `force_ros` unconditionally,
      // captured at line ~823 BEFORE the C1/C4-onto-Cartesian relay block
      // (~lines 1189-1254) has run -- so whenever that relay was active,
      // `applied_force` (documented project-wide, e.g. log_phase2_scenario.py's
      // own docstring, as "the REAL rendered Cartesian force... the haptic
      // force the operator actually feels") silently omitted the relay's
      // contribution, even though the actual `hdSetDoublev(HD_CURRENT_FORCE,
      // ...)` write (using `force_device`, mutated by that same relay block)
      // was correct. Found via sim/tests/diag_c1c4_cartesian_baseline.py: the
      // device's own `last_force` readback showed the relay's effect, the
      // `applied_force` topic never moved regardless of relay weight.
      //
      // Fix mirrors the exact pattern `last_force` already uses two blocks
      // below (mapDeviceVectorToRos of a device-frame vector) -- NOT a new
      // convention. Only the Cartesian branch is affected: `force_device` is
      // only what was actually written when `!joint_path_commanded`; when the
      // joint-space PD path fires instead, the real command was a joint
      // torque, not a Cartesian force, and `force_ros` (the Cartesian
      // spring/damper's own would-be output, never written to the device in
      // that branch) is left as the reported value, UNCHANGED from before
      // this fix -- that branch is off by default, not what this bug was
      // about, and its own applied_force semantics are a separate, distinct
      // question this fix does not attempt to resolve. No controller
      // behavior changed: `force_device` itself is untouched, only what gets
      // read out of it for this one telemetry topic.
      self->state_.applied_force_n =
        joint_path_commanded ? force_ros : self->mapDeviceVectorToRos(force_device);
      self->state_.target_position_m = command.target_pos_m;
      self->state_.position_error_clamped_m = clamped_error;
      self->state_.spring_force_n = spring_force;
      self->state_.damping_force_n = damping_force;
      self->state_.auxiliary_force_n = auxiliary_force;
      self->state_.force_before_primary_clamp_n = force_before_primary_clamp;
      self->state_.force_after_primary_clamp_n = force_after_primary_clamp;
      self->state_.relay_torque_received_nm = relay_torque;
      self->state_.relay_force_n = relay_force_ros;
      self->state_.force_before_final_clamp_n =
        self->mapDeviceVectorToRos(force_before_final_clamp_device);
      self->state_.enabled = command.enabled;
      self->state_.deadman_ok = deadman_ok;
      self->state_.target_valid = command.target_valid;
      self->state_.target_fresh = target_fresh;
      self->state_.force_valid = command.force_valid;
      self->state_.force_fresh = force_fresh;
      self->state_.relay_valid = command.joint_torque_ext_valid;
      self->state_.relay_fresh = relay_fresh;
      self->state_.relay_applied = relay_applied;
      self->state_.position_error_clamped = position_error_clamped;
      self->state_.primary_force_clamped = primary_force_clamped;
      self->state_.final_force_clamped = final_force_clamped;
      self->state_.near_singular = sing.near_singular;
      self->state_.tracking_error_m = tracking_error;
      self->state_.tracking_valid = tracking_valid;
      self->state_.buttons = buttons;
      self->state_.clutch = ((buttons & self->clutch_button_mask_) != 0);
      self->state_.stamp = self->now();

      // Observer snapshot, taken under the same lock as the rest of the state.
      // Copied out here rather than published here: publishing from a
      // HD_MAX_SCHEDULER_PRIORITY callback can allocate and take locks, and a
      // stalled servo frame is the failure this file is built to avoid.
      self->state_.dob_configured = self->dob_bank_.configured();
      if (self->state_.dob_configured) {
        self->state_.dob_tau_dis_hat_nm = self->dob_bank_.tauDisHatNm();
        self->state_.dob_qdot_rad_s = self->dob_bank_.qdotRadS();
        self->state_.dob_tau_cmd_nm = self->dob_prev_tau_nm_;
        for (int i = 0; i < touch_haptics::kNumActiveJoints; ++i) {
          self->state_.dob_contact[i] = self->dob_bank_.joint(i).contactSuspected();
          self->state_.dob_q_rad[i] = q_rad[i];
        }
      }
      self->state_.fe_valid = fe.valid;
      self->state_.fe_tau_ext_nm = fe.tau_ext_nm;
      self->state_.fe_tau_dis_hat_nm = fe.tau_dis_hat_nm;
      self->state_.fe_tau_ext_deadbanded_nm = fe.tau_ext_deadbanded_nm;
      self->state_.fe_confidence = fe.confidence;
      self->state_.joint_path_commanded = joint_path_commanded;
      self->state_.joint_pd_active = joint_pd_active;
      self->state_.joint_pd_gravity_ff_nm = joint_pd_gravity_ff_out;
      self->state_.joint_pd_error_term_nm = joint_pd_error_term_out;
      self->state_.joint_pd_cmd_nm = joint_pd_cmd_out;
      self->state_.joint_pd_write_ok = joint_pd_write_ok;
      if (self->publish_telemetry_) {
        self->state_.last_force_n =
          self->mapDeviceVectorToRos({last_force[0], last_force[1], last_force[2]});
        self->state_.joint_torque_nm = {joint_torque[0], joint_torque[1], joint_torque[2]};
        self->state_.gimbal_torque_nm = {gimbal_torque[0], gimbal_torque[1], gimbal_torque[2]};
        self->state_.joint_angles_rad = {joint_angles[0], joint_angles[1], joint_angles[2]};
        self->state_.motor_temperature = {
          motor_temperature[0], motor_temperature[1], motor_temperature[2]};
      }
    }

    return HD_CALLBACK_CONTINUE;
  }

  std::array<double, 3> mapDeviceVectorToRos(std::array<double, 3> v) const
  {
    if (swap_xy_) {
      std::swap(v[0], v[1]);
    }
    if (invert_x_) {
      v[0] = -v[0];
    }
    if (invert_y_) {
      v[1] = -v[1];
    }
    if (invert_z_) {
      v[2] = -v[2];
    }
    return v;
  }

  std::array<double, 3> mapRosForceToDevice(std::array<double, 3> f) const
  {
    if (invert_x_) {
      f[0] = -f[0];
    }
    if (invert_y_) {
      f[1] = -f[1];
    }
    if (invert_z_) {
      f[2] = -f[2];
    }
    if (swap_xy_) {
      std::swap(f[0], f[1]);
    }
    return f;
  }

  void publishTick()
  {
    DeviceState s;
    {
      std::lock_guard<std::mutex> lock(mtx_);
      s = state_;
    }

    geometry_msgs::msg::PoseStamped pose;
    pose.header.stamp = s.stamp;
    pose.header.frame_id = frame_id_;
    pose.pose.position.x = s.pos_m[0];
    pose.pose.position.y = s.pos_m[1];
    pose.pose.position.z = s.pos_m[2];
    pose.pose.orientation.x = s.quat_xyzw[0];
    pose.pose.orientation.y = s.quat_xyzw[1];
    pose.pose.orientation.z = s.quat_xyzw[2];
    pose.pose.orientation.w = s.quat_xyzw[3];
    pose_pub_->publish(pose);

    geometry_msgs::msg::Vector3Stamped rpy;
    rpy.header = pose.header;
    rpy.vector.x = s.rpy_rad[0];
    rpy.vector.y = s.rpy_rad[1];
    rpy.vector.z = s.rpy_rad[2];
    rpy_pub_->publish(rpy);

    std_msgs::msg::Int32 buttons;
    buttons.data = s.buttons;
    buttons_pub_->publish(buttons);

    std_msgs::msg::Bool clutch;
    clutch.data = s.clutch;
    clutch_pub_->publish(clutch);

    geometry_msgs::msg::WrenchStamped wrench;
    wrench.header = pose.header;
    wrench.wrench.force.x = s.applied_force_n[0];
    wrench.wrench.force.y = s.applied_force_n[1];
    wrench.wrench.force.z = s.applied_force_n[2];
    applied_force_pub_->publish(wrench);

    touch_haptics::msg::CartesianControllerTelemetry ct;
    ct.header = pose.header;
    ct.position_m = s.pos_m; ct.velocity_mps = s.vel_mps;
    ct.target_position_m = s.target_position_m;
    ct.position_error_raw_m = s.tracking_error_m;
    ct.position_error_clamped_m = s.position_error_clamped_m;
    ct.spring_force_n = s.spring_force_n; ct.damping_force_n = s.damping_force_n;
    ct.auxiliary_force_n = s.auxiliary_force_n;
    ct.force_before_primary_clamp_n = s.force_before_primary_clamp_n;
    ct.force_after_primary_clamp_n = s.force_after_primary_clamp_n;
    ct.relay_torque_received_nm = s.relay_torque_received_nm;
    ct.relay_force_n = s.relay_force_n;
    ct.force_before_final_clamp_n = s.force_before_final_clamp_n;
    ct.applied_force_n = s.applied_force_n;
    ct.enabled = s.enabled; ct.deadman_ok = s.deadman_ok;
    ct.target_valid = s.target_valid; ct.target_fresh = s.target_fresh;
    ct.force_valid = s.force_valid; ct.force_fresh = s.force_fresh;
    ct.relay_valid = s.relay_valid; ct.relay_fresh = s.relay_fresh;
    ct.relay_applied = s.relay_applied; ct.tracking_valid = s.tracking_valid;
    ct.position_error_clamped = s.position_error_clamped;
    ct.primary_force_clamped = s.primary_force_clamped;
    ct.final_force_clamped = s.final_force_clamped;
    ct.joint_path_commanded = s.joint_path_commanded;
    ct.cartesian_write_active = !s.joint_path_commanded;
    ct.near_singular = s.near_singular;
    ct.stiffness_n_per_m = stiffness_n_per_m_; ct.damping_n_s_per_m = damping_n_s_per_m_;
    ct.max_position_error_m = max_position_error_m_; ct.max_force_n = max_force_n_;
    ct.stale_timeout_s = stale_timeout_s_;
    cartesian_telemetry_pub_->publish(ct);

    // Observer diagnostics. Published regardless of publish_telemetry_: the
    // observer is a separate concern from device telemetry, and this is the
    // topic the hardware test watches.
    if (dob_diag_pub_) {
      touch_haptics::msg::DobDiagnostics d;
      d.header = pose.header;
      d.configured = s.dob_configured;
      for (int i = 0; i < touch_haptics::kNumActiveJoints; ++i) {
        d.tau_dis_hat_nm[i] = s.dob_tau_dis_hat_nm[i];
        d.qdot_rad_s[i] = s.dob_qdot_rad_s[i];
        d.tau_cmd_nm[i] = s.dob_tau_cmd_nm[i];
        d.contact_suspected[i] = s.dob_contact[i];
        d.q_rad[i] = s.dob_q_rad[i];
        d.joint_pd_gravity_ff_nm[i] = s.joint_pd_gravity_ff_nm[i];
        d.joint_pd_error_term_nm[i] = s.joint_pd_error_term_nm[i];
        d.joint_pd_cmd_nm[i] = s.joint_pd_cmd_nm[i];
      }
      d.joint_pd_active = s.joint_pd_active;
      d.joint_pd_write_ok = s.joint_pd_write_ok;
      dob_diag_pub_->publish(d);
    }

    // ForceEstimate for the peer arm's C1/C4 input, published regardless of
    // publish_telemetry_ for the same reason DobDiagnostics is: this is the
    // bilateral controller's data path, not device telemetry.
    if (force_ext_hat_pub_) {
      touch_haptics::msg::ForceEstimate fe;
      fe.header = pose.header;
      for (int i = 0; i < touch_haptics::kNumActiveJoints; ++i) {
        fe.tau_ext_nm[i] = s.fe_tau_ext_nm[i];
        fe.tau_dis_hat_nm[i] = s.fe_tau_dis_hat_nm[i];
        fe.tau_ext_deadbanded_nm[i] = s.fe_tau_ext_deadbanded_nm[i];
        fe.confidence[i] = s.fe_confidence[i];
      }
      fe.valid = s.fe_valid;
      fe.borrowed_params = false;
      force_ext_hat_pub_->publish(fe);
    }

    if (!publish_telemetry_) {
      return;
    }

    auto publish_vec3 = [&](
      const rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr &pub,
      const std::array<double, 3> &v)
    {
      if (!pub) {
        return;
      }
      geometry_msgs::msg::Vector3Stamped msg;
      msg.header = pose.header;
      msg.vector.x = v[0];
      msg.vector.y = v[1];
      msg.vector.z = v[2];
      pub->publish(msg);
    };

    if (last_force_pub_) {
      geometry_msgs::msg::WrenchStamped msg;
      msg.header = pose.header;
      msg.wrench.force.x = s.last_force_n[0];
      msg.wrench.force.y = s.last_force_n[1];
      msg.wrench.force.z = s.last_force_n[2];
      last_force_pub_->publish(msg);
    }

    publish_vec3(joint_torque_pub_, s.joint_torque_nm);
    publish_vec3(gimbal_torque_pub_, s.gimbal_torque_nm);
    publish_vec3(joint_angles_pub_, s.joint_angles_rad);
    publish_vec3(motor_temperature_pub_, s.motor_temperature);
    publish_vec3(
      tracking_error_pub_,
      s.tracking_valid ? s.tracking_error_m : std::array<double, 3>{0.0, 0.0, 0.0});
  }

private:
  std::string device_name_;
  std::string frame_id_;
  int publish_hz_{200};

  double pos_scale_{0.001};
  bool invert_x_{false};
  bool invert_y_{false};
  bool invert_z_{false};
  bool swap_xy_{false};

  double stiffness_n_per_m_{8.0};
  double damping_n_s_per_m_{0.08};
  double max_force_n_{0.45};
  double dob_torque_f_max_n_{0.6};
  double dob_g_dis_override_rad_s_{-1.0};  // Phase 4B sweep hook, see declare_parameter site
  double dob_g_dis_scale_{1.0};  // preferred sweep mechanism, see declare_parameter site
  double dob_servo_period_s_{0.001};  // fallback matches configureDob()'s 1000Hz default
  // Low-pass filtered qdot, joint-space PD path only -- see the Coulomb
  // breakaway feedforward block below for why this exists and why it is a
  // SEPARATE state from dob_bank_'s own internal (deliberately unfiltered)
  // qdot_prev_.
  touch_haptics::Vec3 qdot_filt_rad_s_{0.0, 0.0, 0.0};
  double max_position_error_m_{0.04};
  double stale_timeout_s_{0.05};
  bool require_deadman_{true};
  int deadman_button_mask_{HD_DEVICE_BUTTON_1};
  int clutch_button_mask_{HD_DEVICE_BUTTON_2};
  bool enabled_on_start_{false};
  bool publish_telemetry_{false};
  bool capabilities_logged_{false};

  bool joint_space_control_{false};
  std::vector<double> kp_nm_per_rad_{0.30, 0.20, 0.27};
  std::vector<double> kd_nms_per_rad_{0.50, 0.30, 0.10};

  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr rpy_pub_;
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr buttons_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr clutch_pub_;
  rclcpp::Publisher<geometry_msgs::msg::WrenchStamped>::SharedPtr applied_force_pub_;
  rclcpp::Publisher<geometry_msgs::msg::WrenchStamped>::SharedPtr last_force_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr joint_torque_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr gimbal_torque_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr joint_angles_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr motor_temperature_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr tracking_error_pub_;

  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr target_sub_;
  rclcpp::Subscription<geometry_msgs::msg::WrenchStamped>::SharedPtr force_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr enable_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Vector3Stamped>::SharedPtr joint_target_sub_;
  rclcpp::Subscription<touch_haptics::msg::ForceEstimate>::SharedPtr joint_torque_ext_sub_;
  rclcpp::Publisher<touch_haptics::msg::ForceEstimate>::SharedPtr force_ext_hat_pub_;
  rclcpp::Publisher<touch_haptics::msg::CartesianControllerTelemetry>::SharedPtr cartesian_telemetry_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  HHD hHD_{HD_INVALID_HANDLE};
  HDSchedulerHandle callback_handle_{HD_INVALID_HANDLE};

  std::mutex mtx_;
  DeviceState state_;
  CommandState command_;

  // Disturbance observer. Both live entirely inside hdCallback and are never
  // touched from the ROS thread, so they stay outside the mtx_ discipline by
  // construction rather than by convention. If any of this is ever published,
  // copy it under mtx_ like DeviceState -- do not read it unlocked.
  //
  // dob_cfg_ is left at its defaults on purpose: M_n, g_dis, threshold and
  // max_torque are all unset sentinels, and calibration_a_complete /
  // clamp_b_complete / enable_torque_write are all false. So dob_bank_ never
  // configures, prepareJointTorqueCommand() always refuses, and the joint-space
  // write is unreachable. Populating these is a manual commissioning step and
  // is deliberately not done in code.
  touch_haptics::DobConfig dob_cfg_{};
  touch_haptics::JointDobBank dob_bank_{};

  // Previous frame's J^T(q)*F_cmd, N*m. Touched only by the servo callback, so
  // it needs no lock. Carries the one-frame lag the bank's q_prev_ pairing
  // expects -- see the note at the update() call site.
  touch_haptics::Vec3 dob_prev_tau_nm_{0.0, 0.0, 0.0};

  // Servo frames since the last successful dob_bank_.configure(), i.e. since
  // this process started (configure() runs once at node startup, never
  // again). Used only by the TOUCH_DIAG_DOB_RESTART_TRANSIENT trace below to
  // bound how many early frames get logged -- see the 2026-09-04 restart-
  // transient investigation (2.79 N*m anomaly, FOUR_CHANNEL_TODO.md).
  uint64_t dob_frames_since_configure_{0};

  std::string dob_inertia_path_;
  std::string dob_gravity_path_;
  bool dob_enable_{true};
  rclcpp::Publisher<touch_haptics::msg::DobDiagnostics>::SharedPtr dob_diag_pub_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  try {
    auto node = std::make_shared<TouchSlave>();
    rclcpp::spin(node);
  } catch (const std::exception &e) {
    fprintf(stderr, "%s\n", e.what());
  }
  rclcpp::shutdown();
  return 0;
}
