// touch_dob.hpp -- per-joint disturbance observer (classical Ohnishi DOBS form)
//
// Three independent SISO observers, one per actuated joint of the Touch/Omni
// (J1 waist, J2 shoulder, J3 elbow). The gimbal is passive and is not observed.
//
// ---------------------------------------------------------------------------
// STATUS: NOT COMMISSIONED. The torque write path is gated OFF.
// ---------------------------------------------------------------------------
// Two prerequisites must be complete before this may command the device:
//
//   (a) HD_LAST_JOINT_TORQUE scale factor must be calibrated. As of writing,
//       exactly one data point exists (0.30 N -> 24.07 on J2, single pose).
//       Linearity and pose-dependence are both unverified. tools/
//       hdapi_torque_calib.cpp exists but has never been run; it sweeps one
//       axis per invocation, so it needs x, y, z at 2-3 distinct poses.
//
//   (b) A per-joint torque clamp must exist and be calibrated against (a).
//       Switching from HD_CURRENT_FORCE to HD_CURRENT_JOINT_TORQUE bypasses
//       clampNorm(force_ros, max_force_n_) in touch_slave.cpp:401, which is
//       currently the only force ceiling in the system. clampJointTorque()
//       below is the replacement, but its limits are uncalibrated until (a).
//
// Until both are done, DobConfig::writeAuthorized() returns false and the
// caller must not write HD_CURRENT_JOINT_TORQUE. See the gate at the bottom.
//
// ---------------------------------------------------------------------------
// THE OBSERVER
// ---------------------------------------------------------------------------
// Plant, per joint:   M*qddot = tau_cmd - tau_dis
// so                  tau_dis = tau_cmd - M*qddot
//
// The naive estimator Q(s)*[tau_cmd - M_n*qddot] needs acceleration. We never
// compute it. With Q(s) = g/(s+g) the acceleration term factors out exactly:
//
//   g/(s+g) * M_n*s*qdot = g*M_n*qdot - g^2*M_n/(s+g) * qdot
//
// which rearranges to the realizable form implemented in JointDob::update():
//
//   tau_dis_hat = g/(s+g) * [ (tau_cmd - G(q)) + g*M_n*qdot ] - g*M_n*qdot
//                 \_____________________  _____________________/
//                          one first-order LPF on a
//                       velocity-and-torque sum, with the
//                        modelled gravity torque removed
//
// G(q) is the gravity torque from the nominal model, subtracted from tau_cmd
// before the filter. Sign convention, verified empirically rather than assumed:
//
//     tau = M*qddot + C*qdot + G(q)      =>   at rest, tau_cmd EQUALS G(q)
//
// so at rest with an exact model the bracket goes to zero and tau_dis_hat goes
// to zero, which is the whole point. Adding G instead of subtracting would
// double the arm's weight into the estimate.
//
// Only qdot appears as a derivative. No second derivative is taken anywhere in
// this file -- deliberate, given this system's history of delayed-derivative
// pumping when damping was enabled. Gravity compensation touches ONLY the
// tau_cmd term and never the velocity path, for that same reason.
//
// NOTE ON qdot's SOURCE: HDAPI exposes HD_CURRENT_VELOCITY in *Cartesian*
// mm/s, not joint space, and offers no joint-velocity register. Absent J(q),
// the only J-free route is a single backward difference of
// HD_CURRENT_JOINT_ANGLES, which JointDobBank does internally. That is one
// differentiation of an encoder signal, not two, so it does not reintroduce
// the failure mode above -- but it is quantization-noisy, and the observer's
// own LPF is what suppresses it. Do not add a second smoothing filter on qdot;
// cascading two lags is what makes this loop pump.
//
// ---------------------------------------------------------------------------
// WHAT tau_dis_hat ACTUALLY CONTAINS  (read before tuning thresholds)
// ---------------------------------------------------------------------------
// The nominal model here is a constant inertia M_n per joint plus a gravity
// model G(q), and, since Phase 2.6 (2026-08-26), an OPTIONAL Coulomb+viscous
// friction model -- see JointDobParams::frictionTorqueNm(). Anything outside
// that still lands in tau_dis_hat -- by construction, not by mistake:
//
//   * Coulomb + viscous friction, WHEN NO FRICTION MODEL IS LOADED for a
//     joint (fric_coulomb_nm == b_visc_nm_s_per_rad == 0.0, the default).
//     Unlike the pre-2026-08-26 state, this is now a per-joint, per-arm
//     runtime fact, not a fixed statement about the codebase: the left arm's
//     inertia_left.json carries positive, well-conditioned Coulomb/viscous
//     terms and IS compensated live; the right arm has never been identified
//     (Phase 4, still not started) and so still sits at the old, uncompensated
//     floor. Compensating friction does not by itself clear the Phase 2
//     STOP -- that separability test predates this change and has not been
//     re-run against the compensated floor (Phase 2.7, still not started) --
//     it only removes one specific, previously-confirmed contributor.
//     Tuning threshold_nm above whatever floor remains is still what the
//     gate below depends on.
//
//   * Off-diagonal inertial coupling and Coriolis terms, since M_n is a
//     scalar per joint rather than a row of M(q).
//
//   * RESIDUAL gravity, i.e. the error in G(q) rather than gravity itself.
//     Gravity is now modelled and subtracted (2026-08-24), so what remains is
//     only how wrong the map is. Measured against the identification data,
//     that residual matched the map's own leave-one-out error -- 0.0075 N*m
//     shoulder, 0.0082 N*m elbow -- with nothing unexplained left over.
//
// So tau_dis_hat is still "everything unmodeled", but gravity has moved out of
// that set and into the model. That change was not cosmetic: with gravity left
// in, the shoulder's p99 floor was 142% of that joint's entire torque
// authority at g_dis = 2*pi*20, meaning the gate could not fire on anything
// the actuator is able to produce. Subtracting G(q) cut the shoulder and elbow
// floors by 54-58%. The waist floor was unchanged to four decimal places,
// exactly as physics requires since gravity cannot load a vertical axis --
// which is a free, independent confirmation that the sign is right.
//
// STILL UNRESOLVED, and the reason the gate remains uncommissioned: even with
// gravity modelled, the remaining floor consumes 40-56% of full-scale torque
// at g_dis = 2*pi*5 and more at higher cutoffs. threshold_nm must clear that
// floor, so the band left for genuine contact is narrow. Lower g_dis helps and
// costs response time. That trade is the operator's to make.
//
#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <string>

namespace touch_haptics
{

// J1 waist, J2 shoulder, J3 elbow. The gimbal (J4-J6) is passive.
static constexpr int kNumActiveJoints = 3;

// Sentinel for the uncalibrated HD_LAST_JOINT_TORQUE scale. Negative is
// deliberately non-physical so it cannot be mistaken for a real ratio, and so
// a zero-initialized struct does not silently look calibrated.
static constexpr double kUncalibratedTorqueScale = -1.0;

// Sentinel for an unloaded gravity coefficient. NaN is used rather than a
// magic number because every arithmetic use of it propagates, so a coefficient
// that escapes the guards cannot quietly behave like a real value -- it makes
// the estimate NaN, which is impossible to mistake for a working observer.
// Zero would have been the dangerous choice: an unloaded model would then look
// exactly like "gravity compensation active, and gravity happens to be zero".
static constexpr double kUnsetGravityCoef =
  std::numeric_limits<double>::quiet_NaN();

// Number of terms in the gravity basis, and the ONLY order they may appear in:
//
//     [ cos(q2), sin(q2), cos(q3), sin(q3) ]
//
// This must stay identical to the "basis" field emitted by
// identification/fit_gravity.py (BASIS_MINIMAL). Loading a fit produced with a
// different basis -- e.g. --basis tilt, which has six terms -- against this
// array would silently reinterpret coefficients as belonging to terms they do
// not, so the loader must check the JSON's recorded basis, not assume it.
static constexpr int kNumGravityCoefs = 4;

// Compile-time master gate for the torque write path. Prerequisites (a) and
// (b) are both open, so this defaults to 0 and the write code is not even
// compiled in. Do not flip this to 1 as a convenience during bring-up; the
// runtime gate exists for that, and it explains itself when it refuses.
#ifndef TOUCH_DOB_ENABLE_TORQUE_WRITE
#define TOUCH_DOB_ENABLE_TORQUE_WRITE 0
#endif

// ===========================================================================
// KINEMATICS -- J(q), and the forward kinematics implied by it
// ===========================================================================
// J(q) as supplied: Jarillo-Silva et al. (2009) Eq. (15).
// Link lengths: Koul et al. (2010), L1 = a2 = 0.135 m, L2 = a3 = 0.135 m.
//
// VERIFICATION STATUS -- read before trusting any torque this produces.
//
//  [VERIFIED, symbolically] The supplied J is exactly integrable: there exists
//  an FK whose Jacobian is J, term for term, with zero residual. That FK is
//  fkFromJacobianConvention() below. This is a strong internal-consistency
//  result -- an algebra slip or a transcription error in any one of the nine
//  entries would almost certainly break integrability -- and it was derived
//  here, not taken on trust.
//
//  [NOT VERIFIED] Agreement with Koul et al.'s DH parameters. Neither paper is
//  in this repo and I have no copy of either, so the independent cross-check
//  against Koul's DH table was NOT performed. Only the link lengths are used
//  from Koul, and those were supplied rather than read. Do not record this as a
//  two-source agreement; it is one source plus a self-consistency proof.
//
//  [NOT VERIFIED, SAFETY-CRITICAL] That the paper's q1/q2/q3 convention -- zero
//  reference, sign, and axis order -- matches what HD_CURRENT_JOINT_ANGLES
//  actually returns, and that the paper's Cartesian frame matches HDAPI's
//  device frame. A mismatch here puts force in the WRONG DIRECTION, which is
//  exactly the failure mode that makes this worse than no force at all.
//
//  This last item is empirically checkable with telemetry that already exists,
//  and checking it is the intended commissioning step: see
//  fkResidualVsMeasured(). Because the FK above was *recovered from* J rather
//  than assumed, agreement between FK(q) and HD_CURRENT_TRANSFORM across
//  several poses simultaneously confirms the angle convention, the frame
//  convention, and both link lengths. Disagreement localizes the fault.
// ===========================================================================

// Koul et al. (2010). SUPPLIED VALUES, not independently read from the paper.
static constexpr double kL1Meters = 0.135;
static constexpr double kL2Meters = 0.135;

// Joint index convention used everywhere in this file, including the
// JointDobParams array. Derived from the recovered FK, not assumed:
//   q1 (index 0) WAIST    -- FK y-coordinate is independent of q1, so q1 is
//                            rotation about the vertical axis. Base yaw.
//   q2 (index 1) SHOULDER -- drives link L1: radial L1*cos(q2), height L1*sin(q2)
//   q3 (index 2) ELBOW    -- drives link L2: radial L2*sin(q3), height -L2*cos(q3)
//
// Note q2 and q3 enter through DIFFERENT trig functions. That is not a typo: q3
// is measured from a different reference than q2 (absolute rather than relative
// to link 1), which is conventional for this arm. It means J is NOT the naive
// serial-chain form, and it is why the singularity condition below involves
// cos(q2 - q3) rather than sin(q3).
enum JointIndex
{
  kWaist = 0,
  kShoulder = 1,
  kElbow = 2
};

// Row-major 3x3.
using Mat3 = std::array<double, 9>;
using Vec3 = std::array<double, 3>;

// J(q), Jarillo-Silva Eq. (15).
inline Mat3 jacobian(const Vec3 &q, double l1 = kL1Meters, double l2 = kL2Meters)
{
  const double c1 = std::cos(q[0]), s1 = std::sin(q[0]);
  const double c2 = std::cos(q[1]), s2 = std::sin(q[1]);
  const double c3 = std::cos(q[2]), s3 = std::sin(q[2]);
  return Mat3{
    -(l1 * c1 * c2 + l2 * s3 * c1),  l1 * s1 * s2,   -l2 * c3 * s1,
    0.0,                             l1 * c2,         l2 * s3,
    -l1 * c2 * s1 - l2 * s3 * s1,   -l1 * s2 * c1,    l2 * c3 * c1};
}

// The FK implied by the above J, recovered by integrating column 1 and then
// confirmed against columns 2 and 3 (zero residual, checked symbolically):
//   x = -sin(q1) * (L1*cos(q2) + L2*sin(q3))
//   y =  L1*sin(q2) - L2*cos(q3)
//   z =  cos(q1) * (L1*cos(q2) + L2*sin(q3))
// Expressed in the PAPER's frame, which is not yet known to equal HDAPI's.
inline Vec3 fkFromJacobianConvention(const Vec3 &q, double l1 = kL1Meters,
                                     double l2 = kL2Meters)
{
  const double r = l1 * std::cos(q[1]) + l2 * std::sin(q[2]);
  return Vec3{-std::sin(q[0]) * r, l1 * std::sin(q[1]) - l2 * std::cos(q[2]),
              std::cos(q[0]) * r};
}

// tau = J^T * F.
inline Vec3 jacobianTransposeTimes(const Mat3 &J, const Vec3 &f)
{
  Vec3 tau{};
  for (int i = 0; i < 3; ++i) {
    tau[i] = J[0 * 3 + i] * f[0] + J[1 * 3 + i] * f[1] + J[2 * 3 + i] * f[2];
  }
  return tau;
}

// Inverse of jacobianTransposeTimes: solve J^T * F = tau for F. Added
// 2026-09-03 to let C1's relayed joint-space external torque (from
// bilateral_4ch.py, via joint_torque_ext_cmd) contribute a Cartesian force
// on the CARTESIAN control path -- i.e. ride alongside C2/C3's Cartesian
// bridge -- instead of requiring the separate joint-space PD path
// exclusively. Closed-form 3x3 inverse (adjugate/determinant), not a
// general solver, since J is always exactly 3x3 here.
//
// PRECONDITION: caller must have already checked
// !singularityMetrics(q).near_singular before calling this. Deliberately NOT
// re-checked here -- det(J^T) == det(J) for any square matrix, so that
// existing check already guarantees this inversion is well-conditioned;
// duplicating the threshold here would be a second source of truth for the
// same fact. Pass sing.det directly (already computed at the call site) to
// avoid recomputing it.
inline Vec3 cartesianForceFromJointTorque(const Mat3 &J, const Vec3 &tau, double det)
{
  // M = J^T, row-major. Adjugate(M) / det(M) gives M^-1; F = M^-1 * tau.
  const Mat3 M{
    J[0], J[3], J[6],
    J[1], J[4], J[7],
    J[2], J[5], J[8],
  };
  const double invDet = 1.0 / det;
  const Mat3 Minv{
    (M[4] * M[8] - M[5] * M[7]) * invDet,
    (M[2] * M[7] - M[1] * M[8]) * invDet,
    (M[1] * M[5] - M[2] * M[4]) * invDet,
    (M[5] * M[6] - M[3] * M[8]) * invDet,
    (M[0] * M[8] - M[2] * M[6]) * invDet,
    (M[2] * M[3] - M[0] * M[5]) * invDet,
    (M[3] * M[7] - M[4] * M[6]) * invDet,
    (M[1] * M[6] - M[0] * M[7]) * invDet,
    (M[0] * M[4] - M[1] * M[3]) * invDet,
  };
  return Vec3{
    Minv[0] * tau[0] + Minv[1] * tau[1] + Minv[2] * tau[2],
    Minv[3] * tau[0] + Minv[4] * tau[1] + Minv[5] * tau[2],
    Minv[6] * tau[0] + Minv[7] * tau[1] + Minv[8] * tau[2],
  };
}

// Singularity metrics. det(J) = -L1*L2*(L1*cos q2 + L2*sin q3)*cos(q2 - q3),
// derived symbolically here rather than cited. Two independent singular loci:
//
//   radial:  R = L1*cos(q2) + L2*sin(q3) = 0  -- arm folded onto its own
//            vertical axis; the waist column collapses and q1 moves nothing.
//   aligned: cos(q2 - q3) = 0, i.e. q2 - q3 = +/-90 deg -- the two links line
//            up and columns 2 and 3 become parallel.
//
// JOINT RANGE: NOT CONFIRMED. Koul's joint limits were not available, and HDAPI
// exposes no joint-limit register (only Cartesian workspace extents), so I
// could not certify "no singularity across the range" as asked. Rather than
// assume a range, this is enforced at RUNTIME every frame -- which is strictly
// stronger than a one-off range check, since it also covers miscalibrated
// encoder offsets and any convention error in the angles themselves.
struct SingularityMetrics
{
  double det = 0.0;
  double radial_r = 0.0;      // L1*cos(q2) + L2*sin(q3), metres
  double align_cos = 0.0;     // cos(q2 - q3)
  bool near_singular = false;
};

// Floors are dimensionally meaningful, not tuned: 0.02 m is ~7% of full reach
// (L1+L2 = 0.27 m); 0.20 keeps the links at least ~11.5 deg from alignment.
// Both need confirming against the real joint range once it is known.
static constexpr double kMinRadialMeters = 0.02;
static constexpr double kMinAlignCos = 0.20;

inline SingularityMetrics singularityMetrics(const Vec3 &q, double l1 = kL1Meters,
                                            double l2 = kL2Meters)
{
  SingularityMetrics m;
  m.radial_r = l1 * std::cos(q[1]) + l2 * std::sin(q[2]);
  m.align_cos = std::cos(q[1] - q[2]);
  m.det = -l1 * l2 * m.radial_r * m.align_cos;
  m.near_singular = std::fabs(m.radial_r) < kMinRadialMeters ||
                    std::fabs(m.align_cos) < kMinAlignCos;
  return m;
}

// Commissioning check for the unverified convention question above. Returns
// ||FK(q) - measured||, in metres. Near zero across several distinct poses
// confirms angle convention, frame convention, and both link lengths at once.
// A large residual means at least one of them is wrong -- do not command joint
// torque until this reads small.
inline double fkResidualVsMeasured(const Vec3 &q, const Vec3 &measured_xyz_m,
                                  double l1 = kL1Meters, double l2 = kL2Meters)
{
  const Vec3 fk = fkFromJacobianConvention(q, l1, l2);
  double sum = 0.0;
  for (int i = 0; i < 3; ++i) {
    const double d = fk[i] - measured_xyz_m[i];
    sum += d * d;
  }
  return std::sqrt(sum);
}

// ---------------------------------------------------------------------------
// Per-joint torque ceilings DERIVED from J(q) and the Cartesian force ceiling.
// ---------------------------------------------------------------------------
// tau = J^T F, so tau_i = (column i of J) . F, hence by Cauchy-Schwarz
//
//     |tau_i| <= || J column i || * ||F||
//
// with equality when F aligns with that column. The column norms collapse to
// closed form (derived symbolically here):
//
//     ||col 1 (waist)||    = |L1*cos(q2) + L2*sin(q3)|   -> max L1 + L2
//     ||col 2 (shoulder)|| = L1                          -> CONSTANT
//     ||col 3 (elbow)||    = L2                          -> CONSTANT
//
// Columns 2 and 3 have pose-independent norm, which is why their limits are
// exact rather than worst-case. Only the waist varies, maximal when the arm is
// fully extended radially: cos(q2) = 1 and sin(q3) = 1, i.e. q2 = 0, q3 = 90
// deg, giving R = L1 + L2 = 0.270 m.
//
// At the shipped 1.5 N Cartesian ceiling with L1 = L2 = 0.135 m:
//
//     waist    1.5 * 0.270 = 0.4050 N*m
//     shoulder 1.5 * 0.135 = 0.2025 N*m
//     elbow    1.5 * 0.135 = 0.2025 N*m
//
// The waist figure is conservative: q2 = 0 with q3 = 90 deg may lie outside the
// real joint range, in which case the true maximum of R is smaller and the
// limit could be tightened. Conservative is the correct direction for a safety
// bound, so it stands until the joint range is known.
//
// DIRECTIONALITY WARNING -- this bound runs one way only. It guarantees that a
// legitimate <=1.5 N Cartesian command maps to torques within these limits. It
// does NOT guarantee the converse: F = J^-T tau gives ||F|| <= ||tau|| /
// sigma_min(J), and sigma_min -> 0 at the singular loci above, so bounded joint
// torques can still produce unbounded Cartesian force near a singularity. The
// per-joint clamp is therefore NOT a substitute for the Cartesian clamp, and
// the runtime singularity guard is what makes it meaningful. Keep all three.
inline Vec3 derivePerJointTorqueLimits(double f_max_n, double l1 = kL1Meters,
                                      double l2 = kL2Meters)
{
  return Vec3{f_max_n * (l1 + l2), f_max_n * l1, f_max_n * l2};
}

// ---------------------------------------------------------------------------
// Per-joint parameters. Independently tunable -- there is no expectation that
// the three joints share any of these. All default to an unset sentinel so a
// default-constructed config is invalid rather than plausible.
// ---------------------------------------------------------------------------
struct JointDobParams
{
  // Nominal inertia, kg*m^2. Constant, not M(q) -- see the header note: a
  // constant M_n is what makes the Q-filter LTI, and is the true Ohnishi form.
  // UNSET. No value is assumed. Source unresolved (Koul et al. vs. identify).
  double m_n_kg_m2 = 0.0;

  // Q-filter cutoff, rad/s. Starting point 2*pi*[20..30] Hz = 126..189 rad/s,
  // per-joint, requiring experimental tuning. UNSET by default.
  double g_dis_rad_s = 0.0;

  // Gate threshold, N*m. Below this, tau_dis_hat is treated as friction/model
  // error and cancelled; above, it is treated as possible genuine contact and
  // left alone. Derive by measurement (free motion, no contact, across the
  // workspace; take ~99th percentile of |tau_dis_hat| and multiply by 1.5-2),
  // NOT by guessing. Must be re-derived once the gravity question is settled.
  // UNSET by default.
  double threshold_nm = 0.0;

  // Release fraction for gate hysteresis. 1.0 reproduces the plain
  // threshold comparison exactly as specified.
  //
  // WHY THIS KNOB EXISTS: the plain comparison is discontinuous. Just below
  // threshold we subtract up to threshold_nm of torque; just above we subtract
  // nothing. So crossing the boundary injects a torque step of up to
  // threshold_nm into a 1 kHz haptic loop -- audible as a click, and a
  // plausible limit-cycle driver if tau_dis_hat dithers around the boundary.
  // This is the same class of defect already documented at bridge:423 (the
  // engage-instant force step), and the reflection controller already solves
  // it with a 0.6 release ratio. Setting ~0.6 here mirrors that. Left at 1.0
  // by default so the shipped behaviour is the specified behaviour; hysteresis
  // narrows the chatter band but does not remove the step, so a smooth taper
  // is worth considering instead. Flagging, not silently deciding.
  double gate_release_ratio = 1.0;

  // Per-joint torque ceiling, N*m, enforced before every device write.
  // UNSET by default. Uncalibrated until prerequisite (a).
  //
  // CAVEAT: three independent per-joint limits are NOT equivalent to the
  // Cartesian clampNorm they replace. clampNorm bounds ||F||; this bounds each
  // joint separately, so all three at their limit can yield a Cartesian
  // magnitude above the old 1.5 N ceiling. Relating these limits back to that
  // ceiling requires J(q). Choose them conservatively until it exists.
  double max_torque_nm = 0.0;

  // Gravity model for THIS joint, over the fixed basis
  //     [ cos(q2), sin(q2), cos(q3), sin(q3) ]
  // with q2 = shoulder and q3 = elbow. Note that every joint's gravity torque
  // is a function of q2 and q3 only -- not of its own angle. The waist is
  // included for uniformity; on a level base its coefficients should come out
  // near zero, and that is a useful check rather than wasted storage.
  //
  // Units N*m, sign convention:
  //
  //     tau = M*qddot + C*qdot + G(q)      =>   at rest, tau_cmd EQUALS G(q)
  //
  // so the observer SUBTRACTS this from tau_cmd. That sign was verified
  // empirically, not assumed: the ratio of J^T(hold bias) to the fitted map
  // measured -1.02 across nine captures, and getting it backwards is not a
  // subtle failure -- it doubles gravity instead of removing it, which on real
  // data drove R^2 to -6.9 and produced negative damping and negative inertia.
  //
  // UNSET by default. See kUnsetGravityCoef for why NaN rather than zero.
  std::array<double, kNumGravityCoefs> grav_coef{
    kUnsetGravityCoef, kUnsetGravityCoef, kUnsetGravityCoef, kUnsetGravityCoef};

  bool gravityModelLoaded() const
  {
    for (double c : grav_coef) {
      if (!std::isfinite(c)) {
        return false;
      }
    }
    return true;
  }

  // ---------------------------------------------------------------------
  // Friction compensation (Phase 2.6). OPTIONAL, unlike the gravity model:
  // zero is a valid, meaningful default here ("no friction compensation
  // configured"), so unlike grav_coef there is no NaN-sentinel dance and
  // this is NOT required by valid() below. An observer with these left at
  // zero behaves EXACTLY as it did before this phase existed -- friction
  // compensation degrades gracefully to absent, it does not block startup.
  //
  // SOURCE, DELIBERATELY: inertia_left.json's `b_visc`/`c_coulomb_nm`
  // (from fit_inertia.py), NOT friction_left.json's raw per-joint median.
  // The static-sweep file's own numbers are not safe to use directly here:
  // its waist median is NEGATIVE (-0.0058 N*m as of the 2026-08-24 fit),
  // and identification/README.md documents exactly this failure mode --
  // "Negative Coulomb estimates are non-physical and bound nothing." It
  // also carries no viscous term at all (the static sweep is quasi-static
  // by construction, qdot~0 at breakaway). fit_inertia.py's regression
  // fits both terms jointly over the full dynamic capture and cross-checks
  // its Coulomb term against the static one -- see identification/README.md
  // -- and every value in inertia_left.json as of writing is positive and
  // marked well_conditioned:true. That is the number this compensates with.
  //
  // Coulomb term, N*m -- the constant breakaway/kinetic friction magnitude.
  double fric_coulomb_nm = 0.0;

  // Viscous term, N*m*s/rad -- linear-in-velocity friction.
  double b_visc_nm_s_per_rad = 0.0;

  // Smoothing bandwidth for the Coulomb sign, rad/s. A hard sign(qdot) is
  // exactly the kind of thing this file's own history warns against (see
  // arm_model.hpp's frictionTrueNm(), which uses the same tanh() smoothing
  // for the identical reason): a discontinuous friction estimate right at
  // qdot=0 would chatter every time the arm is nearly still, injecting
  // exactly the kind of near-zero-velocity dither this observer already has
  // to fight elsewhere. Default matches ArmParams::stribeck_vel (0.02 rad/s)
  // for consistency with the sim's own friction model, not because the real
  // arm's Stribeck velocity was independently measured -- it was not.
  double vel_thresh_rad_s = 0.02;

  bool frictionModelLoaded() const
  {
    return fric_coulomb_nm > 0.0 || b_visc_nm_s_per_rad > 0.0;
  }

  // Estimated joint friction torque at this velocity, N*m. Zero, exactly,
  // when no friction model is loaded -- see frictionModelLoaded() above.
  double frictionTorqueNm(double qdot_rad_s) const
  {
    const double vt = (vel_thresh_rad_s > 1e-9) ? vel_thresh_rad_s : 0.02;
    return fric_coulomb_nm * std::tanh(qdot_rad_s / vt) +
           b_visc_nm_s_per_rad * qdot_rad_s;
  }

  // G(q) for this joint, N*m. Allocation-free and branch-free: this runs once
  // per joint per servo frame at ~1 kHz.
  //
  // Callers must not reach here with an unloaded model -- configure() refuses
  // such params -- but if one did, the NaN sentinel propagates into the result
  // rather than silently producing a plausible zero.
  double gravityTorqueNm(double q2_rad, double q3_rad) const
  {
    return grav_coef[0] * std::cos(q2_rad) + grav_coef[1] * std::sin(q2_rad) +
           grav_coef[2] * std::cos(q3_rad) + grav_coef[3] * std::sin(q3_rad);
  }

  bool valid() const
  {
    return m_n_kg_m2 > 0.0 && g_dis_rad_s > 0.0 && threshold_nm > 0.0 &&
           max_torque_nm > 0.0 && gate_release_ratio > 0.0 &&
           gate_release_ratio <= 1.0 && gravityModelLoaded();
  }

  // Why valid() is false, for the same "explain the refusal" reason
  // DobConfig::writeAuthorized() exists. Returns nullptr when valid.
  const char *invalidReason() const
  {
    if (!(m_n_kg_m2 > 0.0)) return "m_n_kg_m2 unset";
    if (!(g_dis_rad_s > 0.0)) return "g_dis_rad_s unset";
    if (!(threshold_nm > 0.0)) return "threshold_nm unset";
    if (!(max_torque_nm > 0.0)) return "max_torque_nm unset";
    if (!(gate_release_ratio > 0.0 && gate_release_ratio <= 1.0)) {
      return "gate_release_ratio outside (0, 1]";
    }
    if (!gravityModelLoaded()) {
      return "gravity model unset (grav_coef still at the NaN sentinel); load "
             "the coefficients from fit_gravity.py output, or the observer "
             "would treat gravity as a disturbance -- which on this arm is "
             "larger than the joint's entire torque authority";
    }
    return nullptr;
  }
};

// ---------------------------------------------------------------------------
// Whole-observer config, including both commissioning gates.
// ---------------------------------------------------------------------------
struct DobConfig
{
  std::array<JointDobParams, kNumActiveJoints> joints{};

  // N*m per raw HD_LAST_JOINT_TORQUE unit. Stays at the sentinel until
  // hdapi_torque_calib.cpp has been run across x/y/z at 2-3 poses AND the
  // ratio has been shown constant. The single known point (0.30 N -> 24.07,
  // J2) is not sufficient to fill this in.
  double last_joint_torque_scale_nm_per_unit = kUncalibratedTorqueScale;

  // Prerequisite (a): set true only when the sweep above is actually done.
  bool calibration_a_complete = false;

  // Prerequisite (b): set true only when max_torque_nm has been designed
  // against real calibrated numbers, not placeholders.
  bool clamp_b_complete = false;

  // Runtime arm switch for the device write. Independent of the two flags
  // above: all three must be true to command torque.
  bool enable_torque_write = false;

  bool torqueScaleCalibrated() const
  {
    return last_joint_torque_scale_nm_per_unit > 0.0 &&
           std::isfinite(last_joint_torque_scale_nm_per_unit);
  }

  bool paramsValid() const
  {
    for (const JointDobParams &p : joints) {
      if (!p.valid()) {
        return false;
      }
    }
    return true;
  }

  // The single authority on whether HD_CURRENT_JOINT_TORQUE may be written.
  // Populates `why` with the first failing reason when it returns false.
  bool writeAuthorized(std::string *why = nullptr) const
  {
#if !TOUCH_DOB_ENABLE_TORQUE_WRITE
    if (why) {
      *why = "TOUCH_DOB_ENABLE_TORQUE_WRITE is 0 (compile-time gate)";
    }
    return false;
#else
    if (!calibration_a_complete) {
      if (why) { *why = "prerequisite (a) incomplete: HD_LAST_JOINT_TORQUE "
                        "scale not calibrated (hdapi_torque_calib.cpp unrun)"; }
      return false;
    }
    if (!torqueScaleCalibrated()) {
      if (why) { *why = "last_joint_torque_scale_nm_per_unit still at the "
                        "uncalibrated sentinel"; }
      return false;
    }
    if (!clamp_b_complete) {
      if (why) { *why = "prerequisite (b) incomplete: per-joint torque clamp "
                        "not calibrated against (a)"; }
      return false;
    }
    if (!paramsValid()) {
      if (why) { *why = "per-joint DOB parameters incomplete (M_n / g_dis / "
                        "threshold / max_torque unset)"; }
      return false;
    }
    if (!enable_torque_write) {
      if (why) { *why = "enable_torque_write is false (runtime gate)"; }
      return false;
    }
    return true;
#endif
  }
};

// ---------------------------------------------------------------------------
// One joint's observer.
// ---------------------------------------------------------------------------
class JointDob
{
public:
  // sample_period_s must be the real servo period (~1/1019 s as measured),
  // not the 200 Hz publish period. Returns false if params are incomplete, in
  // which case the observer stays unconfigured and update() is a no-op.
  bool configure(const JointDobParams &params, double sample_period_s)
  {
    if (!params.valid() || !(sample_period_s > 0.0)) {
      configured_ = false;
      return false;
    }
    params_ = params;
    g_m_ = params.g_dis_rad_s * params.m_n_kg_m2;

    // Bilinear (Tustin) discretization of Q(s) = g/(s+g):
    //   y[k] = (2-gT)/(2+gT) * y[k-1] + gT/(2+gT) * (u[k] + u[k-1])
    //
    // Two properties matter here, and a zero-order-hold pole exp(-g*T) with a
    // (1-a) input gain fails the second:
    //
    //  1. Unconditional stability. The pole (2-gT)/(2+gT) has magnitude < 1 for
    //     every positive g and T, so no cutoff chosen during tuning can drive
    //     the filter unstable. On a loop with this system's pumping history
    //     that guarantee is not negotiable. (ZOH also has this property.)
    //
    //  2. No ramp lag. ZOH is exact only for a piecewise-constant input. The
    //     input here is tau_cmd + g*M_n*qdot, and qdot ramps whenever the joint
    //     accelerates, so ZOH lags it by half a sample. That lag is multiplied
    //     by g*M_n, producing a bias of about (g*M_n*T/2)*qddot in the estimate
    //     -- measured at 0.023 N*m for g=2pi*25, M_n=0.01, qddot=30 rad/s^2.
    //     That is the same order as the friction floor and as a plausible
    //     threshold_nm, i.e. large enough to move the contact gate. Tustin is
    //     exact for ramps to first order and removes it.
    //
    // DC gain is exactly 1 (substitute z=1), so a constant disturbance is
    // recovered without steady-state error.
    const double gt = params.g_dis_rad_s * sample_period_s;
    pole_ = (2.0 - gt) / (2.0 + gt);
    in_gain_ = gt / (2.0 + gt);

    configured_ = true;
    reset();
    return true;
  }

  void reset()
  {
    lpf_ = 0.0;
    u_prev_ = 0.0;
    tau_dis_hat_ = 0.0;
    gate_open_ = false;
  }

  bool configured() const { return configured_; }

  // tau_cmd_nm, qdot_rad_s and g_tau_nm must all refer to the SAME servo
  // frame. Because HD_LAST_JOINT_TORQUE lags one frame, the caller passes the
  // previous frame's qdot AND the previous frame's gravity torque --
  // JointDobBank handles that buffering.
  //
  // g_tau_nm is deliberately a REQUIRED argument with no default. A default of
  // 0.0 would let a caller that has not loaded a gravity model compile and run
  // as though compensation were active while silently doing nothing, which is
  // the exact failure this change exists to prevent. Callers that genuinely
  // want no compensation must pass 0.0 and say so at the call site.
  double update(double tau_cmd_nm, double qdot_rad_s, double g_tau_nm)
  {
    if (!configured_) {
      return 0.0;
    }
    const double gm_qdot = g_m_ * qdot_rad_s;
    // Gravity is removed from the commanded torque BEFORE the filter. It is a
    // modelled term, not a disturbance, so leaving it in makes the observer
    // report the arm's own weight as contact. Measured on this arm: doing so
    // put the shoulder's p99 floor at 142% of that joint's full torque
    // authority, i.e. the gate could not fire on anything the actuator can
    // produce. Subtracting it here cut the shoulder and elbow floors by 54-58%.
    //
    // Only the tau_cmd path is touched. The qdot path is untouched on purpose:
    // this file's history is of cascaded lags pumping the loop, and gravity
    // compensation has no business anywhere near the velocity estimate.
    //
    // Friction (Phase 2.6) is removed the same way, for the same reason: it
    // is a modelled term (once identified), not a disturbance, so leaving it
    // in makes the observer report the arm's own friction as contact -- this
    // is the mechanism the 2026-08-26 separability STOP diagnosed (40-57% of
    // torque authority consumed by an uncompensated floor). Uses the same
    // qdot_rad_s the caller already aligned to the previous frame for
    // gm_qdot above -- no new required argument, unlike g_tau_nm, because
    // params_.frictionTorqueNm() returns exactly 0.0 when unconfigured (see
    // JointDobParams::frictionModelLoaded()), so an unloaded friction model
    // is silently a no-op rather than silently wrong the way a defaulted
    // g_tau_nm would have been.
    const double fric_tau_nm = params_.frictionTorqueNm(qdot_rad_s);
    const double u = (tau_cmd_nm - g_tau_nm - fric_tau_nm) + gm_qdot;
    lpf_ = pole_ * lpf_ + in_gain_ * (u + u_prev_);
    u_prev_ = u;
    tau_dis_hat_ = lpf_ - gm_qdot;

    // Gate with optional hysteresis. At gate_release_ratio == 1.0 this is the
    // plain |tau_dis_hat| < threshold comparison.
    const double mag = std::fabs(tau_dis_hat_);
    const double release = params_.threshold_nm * params_.gate_release_ratio;
    if (gate_open_) {
      gate_open_ = mag > release;
    } else {
      gate_open_ = mag > params_.threshold_nm;
    }
    return tau_dis_hat_;
  }

  double tauDisHatNm() const { return tau_dis_hat_; }

  // Raw filter state, exposed read-only for the 2026-09-04 restart-transient
  // investigation (a single 2.79 N*m C4 reading, ~25-35x every other
  // real-hardware measurement, seen once right after a master-node restart,
  // not reproduced on a clean re-capture -- see FOUR_CHANNEL_TODO.md).
  // reset() unconditionally zeroes both, so by construction they MUST read
  // exactly 0.0 immediately after every configure() call; these accessors
  // let a caller confirm that guarantee held on real hardware empirically
  // rather than by code-reading alone, and let it be logged alongside the
  // pose/velocity signals that are the more likely actual culprit (a
  // backward-difference qdot spike on the first primed frame after a
  // device re-init) -- see JointDobBank::update()'s own primed_ handling.
  double lpfState() const { return lpf_; }
  double uPrevState() const { return u_prev_; }

  // True when the estimate is large enough to be plausibly genuine contact,
  // i.e. cancellation is suppressed. "Plausibly" is doing real work in that
  // sentence: with F_fric == 0 and gravity unmodeled, this flag is only as
  // trustworthy as threshold_nm's calibration.
  bool contactSuspected() const { return gate_open_; }

  // Threshold-gated correction. Below threshold the disturbance is treated as
  // friction/model error and cancelled; above, tau_ref passes through
  // untouched and the caller should forward tauDisHatNm() onward instead.
  //
  // Deliberately NOT converted to a Cartesian F_ext here -- that conversion is
  // a separate decision and is not built into this task.
  double correct(double tau_ref_nm) const
  {
    if (!configured_) {
      return tau_ref_nm;
    }
    return gate_open_ ? tau_ref_nm : (tau_ref_nm - tau_dis_hat_);
  }

private:
  JointDobParams params_{};
  double pole_ = 0.0;         // (2-g*Ts)/(2+g*Ts)
  double in_gain_ = 0.0;      // g*Ts/(2+g*Ts)
  double g_m_ = 0.0;          // g_dis * M_n, precomputed
  double lpf_ = 0.0;          // filter state
  double u_prev_ = 0.0;       // previous filter input (Tustin needs u[k-1])
  double tau_dis_hat_ = 0.0;
  bool gate_open_ = false;
  bool configured_ = false;
};

// ---------------------------------------------------------------------------
// The three observers plus the signal conditioning they share: single
// differentiation of joint angles, and the one-sample alignment that
// HD_LAST_JOINT_TORQUE's one-frame lag requires.
// ---------------------------------------------------------------------------
class JointDobBank
{
public:
  bool configure(const DobConfig &cfg, double sample_period_s)
  {
    cfg_ = cfg;
    ts_ = sample_period_s;
    if (!(ts_ > 0.0)) {
      configured_ = false;
      return false;
    }
    bool ok = true;
    for (int i = 0; i < kNumActiveJoints; ++i) {
      ok = dob_[i].configure(cfg.joints[i], ts_) && ok;
    }
    configured_ = ok;
    primed_ = false;
    return ok;
  }

  void reset()
  {
    for (JointDob &d : dob_) {
      d.reset();
    }
    primed_ = false;
    q_prev_.fill(0.0);
    qdot_prev_.fill(0.0);
    g_eval_pose_.fill(0.0);
  }

  bool configured() const { return configured_; }

  // Call once per servo frame from inside hdCallback.
  //   q_rad         -- HD_CURRENT_JOINT_ANGLES, this frame
  //   tau_cmd_raw   -- HD_LAST_JOINT_TORQUE, this frame (raw device units;
  //                    reflects the command issued one frame ago)
  //
  // Returns false while unconfigured or during the first frame, when no
  // velocity estimate exists yet.
  bool update(const std::array<double, kNumActiveJoints> &q_rad,
              const std::array<double, kNumActiveJoints> &tau_cmd_raw)
  {
    if (!configured_) {
      return false;
    }

    // Single backward difference -- one differentiation, never two.
    std::array<double, kNumActiveJoints> qdot_now{};
    for (int i = 0; i < kNumActiveJoints; ++i) {
      qdot_now[i] = primed_ ? (q_rad[i] - q_prev_[i]) / ts_ : 0.0;
    }

    if (!primed_) {
      // First frame: seed history, produce nothing. Feeding a garbage velocity
      // into the filter would take several time constants to wash out.
      q_prev_ = q_rad;
      qdot_prev_ = qdot_now;
      primed_ = true;
      return false;
    }

    // Scale raw device torque into N*m. Guarded because an uncalibrated scale
    // makes every estimate below meaningless -- see prerequisite (a).
    const bool scale_ok = cfg_.torqueScaleCalibrated();
    const double scale = scale_ok ? cfg_.last_joint_torque_scale_nm_per_unit : 0.0;

    // Gravity is evaluated at q_prev_, NOT at q_rad, for exactly the same
    // reason qdot_prev_ is used below: this frame's HD_LAST_JOINT_TORQUE is the
    // command issued one frame ago, and that command was computed for the pose
    // the arm held then. Using q_rad here would pair a torque with a pose one
    // sample in its future, injecting a spurious dG/dt * Ts term that grows
    // with joint speed -- precisely the kind of velocity-dependent artefact
    // this observer must not manufacture.
    //
    // Suppressed when the torque scale is uncalibrated: tau_cmd is forced to
    // zero there, so subtracting a real G(q) would leave -G(q) standing as a
    // large fictitious disturbance. Better to degrade to "estimate is zero and
    // update() reports false" than to "estimate is confidently wrong".
    const double q2_prev = q_prev_[1];
    const double q3_prev = q_prev_[2];
    // Observational only -- see lastGravityEvalPoseRad(). Never read back.
    g_eval_pose_[0] = q2_prev;
    g_eval_pose_[1] = q3_prev;

    for (int i = 0; i < kNumActiveJoints; ++i) {
      const double g_tau =
        scale_ok ? cfg_.joints[i].gravityTorqueNm(q2_prev, q3_prev) : 0.0;
      // qdot_prev_ pairs with this frame's HD_LAST_JOINT_TORQUE, which was
      // commanded one frame ago. This is the explicit one-sample alignment;
      // do not substitute qdot_now. The same-frame reasoning in the comment at
      // touch_slave.cpp:406 is correct for HD_CURRENT_* and wrong for HD_LAST_*.
      dob_[i].update(tau_cmd_raw[i] * scale, qdot_prev_[i], g_tau);
    }

    q_prev_ = q_rad;
    qdot_prev_ = qdot_now;
    return scale_ok;
  }

  const JointDob &joint(int i) const { return dob_[i]; }

  std::array<double, kNumActiveJoints> tauDisHatNm() const
  {
    std::array<double, kNumActiveJoints> out{};
    for (int i = 0; i < kNumActiveJoints; ++i) {
      out[i] = dob_[i].tauDisHatNm();
    }
    return out;
  }

  // Per-joint threshold-gated correction of a joint-space reference torque.
  std::array<double, kNumActiveJoints> correct(
    const std::array<double, kNumActiveJoints> &tau_ref_nm) const
  {
    std::array<double, kNumActiveJoints> out{};
    for (int i = 0; i < kNumActiveJoints; ++i) {
      out[i] = dob_[i].correct(tau_ref_nm[i]);
    }
    return out;
  }

  // Last computed joint velocity, rad/s. Exposed for logging and for the
  // still-unverified HD_CURRENT_VELOCITY vs. J(q)^-1 cross-check.
  const std::array<double, kNumActiveJoints> &qdotRadS() const { return qdot_prev_; }

  // {q2, q3} that the gravity term was last evaluated at, radians.
  //
  // PURELY OBSERVATIONAL. Nothing in this file reads g_eval_pose_; it is
  // written once per update() and never fed back into the observer, so it
  // cannot influence tau_dis_hat by any path. The cost is two double stores
  // per frame -- no allocation, no branch, no call.
  //
  // It exists because the one-frame lag on this term is invisible from the
  // outside. Perturbing q_rad to infer which pose gravity used also changes
  // the backward-difference qdot for that frame, so an indirect test cannot
  // distinguish "gravity correctly used q_prev_" from "gravity used q_rad and
  // the velocity term happened to cancel it". Recording the pose directly is
  // the only way to pin the alignment down, and without a test a later
  // simplification pass could swap q_prev_ for q_rad with every other test
  // still green.
  //
  // Written unconditionally, including when the torque scale is uncalibrated
  // and the gravity result is discarded: the question it answers is "which
  // pose did the gravity path read", not "was the answer used".
  const std::array<double, 2> &lastGravityEvalPoseRad() const { return g_eval_pose_; }

private:
  DobConfig cfg_{};
  std::array<JointDob, kNumActiveJoints> dob_{};
  std::array<double, kNumActiveJoints> q_prev_{};
  std::array<double, kNumActiveJoints> qdot_prev_{};
  std::array<double, 2> g_eval_pose_{0.0, 0.0};
  double ts_ = 0.0;
  bool primed_ = false;
  bool configured_ = false;
};

// ---------------------------------------------------------------------------
// Prerequisite (b): the replacement safety ceiling.
// ---------------------------------------------------------------------------
enum class ClampStatus
{
  kOk,                    // within limits
  kClamped,               // limited, safe to command
  kRefusedUncalibrated,   // scale factor unknown -- do not command
  kRefusedUnconfigured    // limits unset -- do not command
};

// Per-joint magnitude clamp, sign preserved (mirroring the direction-preserving
// intent of clampNorm, applied per joint).
//
// Refuses rather than clamping when it cannot know what the numbers mean: with
// an uncalibrated scale factor a "1.0 N*m" limit is an arbitrary number, and a
// clamp that silently passes an arbitrary limit is worse than no clamp, because
// it reads as a safety feature. On refusal tau_nm is zeroed and the caller MUST
// NOT write to the device.
inline ClampStatus clampJointTorque(
  const DobConfig &cfg, std::array<double, kNumActiveJoints> &tau_nm)
{
  if (!cfg.torqueScaleCalibrated() || !cfg.calibration_a_complete) {
    tau_nm.fill(0.0);
    return ClampStatus::kRefusedUncalibrated;
  }
  for (int i = 0; i < kNumActiveJoints; ++i) {
    if (!(cfg.joints[i].max_torque_nm > 0.0)) {
      tau_nm.fill(0.0);
      return ClampStatus::kRefusedUnconfigured;
    }
  }

  bool clamped = false;
  for (int i = 0; i < kNumActiveJoints; ++i) {
    const double lim = cfg.joints[i].max_torque_nm;
    if (!std::isfinite(tau_nm[i])) {
      // NaN/inf must never reach the motors. A non-finite estimate means the
      // observer diverged; zero the axis rather than propagate it.
      tau_nm[i] = 0.0;
      clamped = true;
      continue;
    }
    if (tau_nm[i] > lim) {
      tau_nm[i] = lim;
      clamped = true;
    } else if (tau_nm[i] < -lim) {
      tau_nm[i] = -lim;
      clamped = true;
    }
  }
  return clamped ? ClampStatus::kClamped : ClampStatus::kOk;
}

inline const char *toString(ClampStatus s)
{
  switch (s) {
    case ClampStatus::kOk: return "ok";
    case ClampStatus::kClamped: return "clamped";
    case ClampStatus::kRefusedUncalibrated: return "refused:uncalibrated";
    case ClampStatus::kRefusedUnconfigured: return "refused:unconfigured";
  }
  return "unknown";
}

// ---------------------------------------------------------------------------
// The only sanctioned way to reach HD_CURRENT_JOINT_TORQUE.
// ---------------------------------------------------------------------------
// Returns true only if every gate passes AND the clamp succeeded, in which
// case tau_nm is safe to hand to hdSetDoublev. Returns false with tau_nm
// zeroed otherwise, and `why` explains which gate refused.
//
// The caller must treat false as "do not write anything this frame" -- not as
// "write zeros" and not as "fall back to the previous command".
inline bool prepareJointTorqueCommand(
  const DobConfig &cfg,
  std::array<double, kNumActiveJoints> &tau_nm,
  std::string *why = nullptr)
{
  if (!cfg.writeAuthorized(why)) {
    tau_nm.fill(0.0);
    return false;
  }
  const ClampStatus st = clampJointTorque(cfg, tau_nm);
  if (st == ClampStatus::kRefusedUncalibrated || st == ClampStatus::kRefusedUnconfigured) {
    if (why) {
      *why = std::string("clampJointTorque ") + toString(st);
    }
    return false;
  }
  return true;
}

}  // namespace touch_haptics
