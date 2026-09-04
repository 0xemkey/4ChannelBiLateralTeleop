// arm_model.hpp
//
// 3-DOF rigid-body model of a Phantom Omni / Geomagic Touch arm, for the
// hardware-free simulator. Header-only and dependency-free apart from
// touch_dob.hpp, which supplies the SAME kinematics the real node uses.
//
// ---------------------------------------------------------------------------
// THE MOST IMPORTANT DESIGN RULE IN THIS FILE
// ---------------------------------------------------------------------------
// The simulator's parameters MUST DELIBERATELY DIFFER from the identified ones
// in ~/dob_logs/results/*.json.
//
// If the sim used the same M_n and the same G(q) the observer uses, then by
// construction
//
//     tau_dis_hat = Q(s)[(tau_cmd - G_model) - M_model*qddot] == 0
//
// and every threshold, every gate, every separability test and every safety
// clamp would pass vacuously. The sim would be a mirror, not a test.
//
// So: `inertia_mismatch` and `gravity_mismatch` default to 1.08 and 1.06, and a
// base-tilt term is added to the waist that the observer's basis cannot
// represent at all. Those numbers are not physical measurements -- they are a
// deliberate, documented modelling error of roughly the size the real arm shows.
//
// If you ever find yourself "fixing" the sim by setting the mismatches to 1.0
// because the observer output looks too noisy, stop: you have just deleted the
// only thing the sim was measuring.
//
// ---------------------------------------------------------------------------
// WHAT IS AND IS NOT MODELLED
// ---------------------------------------------------------------------------
// Modelled:   diagonal constant inertia, gravity over the fitted basis, base
//             tilt, Coulomb + viscous + Stribeck friction, encoder
//             quantisation, joint limits, semi-implicit integration.
// NOT modelled: a validated M(q) with off-diagonal coupling (no such model
//             exists in this repo -- the identification produced a constant
//             diagonal M_n and nothing more), Coriolis/centrifugal terms,
//             cable stretch, structural compliance, motor thermal behaviour.
//
// Pretending to a full M(q) here would be worse than the honest diagonal: it
// would encode invented numbers as if they were measured. The optional
// off-diagonal coupling term exists so that effect can be *probed*, and is zero
// by default.
#pragma once

#include "touch_haptics/touch_dob.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <limits>
#include <random>

namespace touch_sim
{

using touch_haptics::Vec3;
using touch_haptics::Mat3;

// ---------------------------------------------------------------------------
// Per-arm configuration.
// ---------------------------------------------------------------------------
struct ArmParams
{
  // Nominal inertia, kg*m^2, BEFORE mismatch is applied. Defaults are the
  // identified left-arm values from inertia_left.json.
  Vec3 m_n{3.3694e-3, 2.9452e-3, 2.3291e-4};

  // Gravity coefficients per joint over the basis
  //   [cos q2, sin q2, cos q3, sin q3]
  // matching fit_gravity.py's BASIS_MINIMAL, sign convention
  //   tau = M qddot + C qdot + G(q)   =>  at rest tau_cmd == G(q).
  // Defaults are gravity_left_minimal.json.
  std::array<std::array<double, 4>, 3> grav_coef{{
    {{-0.00730,  0.00262,  0.00448,  0.00308}},   // waist
    {{ 0.11979,  0.03195, -0.03347, -0.04448}},   // shoulder
    {{-0.00108, -0.01253,  0.00057,  0.08863}},   // elbow
  }};

  // DELIBERATE MODEL ERROR -- see the header note. These are the whole point.
  double inertia_mismatch = 1.08;
  double gravity_mismatch = 1.06;

  // Base tilt. The real rig measured ~3-6 degrees, which loads the waist --
  // a joint that on a level base cannot be loaded by gravity at all, and which
  // the observer's basis (a function of q2/q3 only) structurally cannot model.
  // This is the single most valuable disturbance in the simulator.
  double base_tilt_rad = 0.0785;          // ~4.5 deg
  double base_tilt_phase_rad = -2.36;     // measured phase, ~-135 deg
  double tilt_torque_scale = 0.0214;      // N*m amplitude on the waist

  // Friction, per joint. Coulomb / static / viscous / Stribeck velocity.
  Vec3 fric_coulomb{0.00194, 0.00292, 0.00150};   // from friction_left.json
  Vec3 fric_static{0.00280, 0.00400, 0.00210};    // stiction > Coulomb
  Vec3 fric_viscous{0.00162, 0.00122, 0.00087};   // from fit_inertia b terms
  double stribeck_vel = 0.02;                     // rad/s
  double fric_eps = 0.01;                         // tanh width, rad/s

  // Optional off-diagonal inertia coupling, kg*m^2. ZERO BY DEFAULT because no
  // validated M(q) exists for this arm. Set it only to probe sensitivity, and
  // never report a result from a non-zero value as if it were measured.
  Mat3 m_coupling{0, 0, 0, 0, 0, 0, 0, 0, 0};

  // Encoder quantisation, radians. NOT optional in practice: without it
  // tau_dis_hat comes out unrealistically clean and every threshold test in the
  // suite passes for the wrong reason. 0.055 mm nominal tip resolution over a
  // 0.135 m link.
  double encoder_quantum_rad = 4.07e-4;

  // Joint travel limits, radians. Approximate; they exist so a runaway in a
  // controller test hits a wall instead of integrating to infinity.
  Vec3 q_min{-1.6, -0.6, -0.3};
  Vec3 q_max{ 1.6,  1.6,  2.6};

  // Motor saturation, N*m per joint, before the driver pipeline.
  Vec3 tau_motor_max{0.35, 0.25, 0.25};

  // Servo rate the scheduler advertises and integrates at.
  int update_rate_hz = 1000;

  // Raw units per N*m for HD_LAST_JOINT_TORQUE. The real device's scale has
  // never been calibrated; comparing logged raw against J^T F put it near
  // 1e3 raw per N*m. Reproducing it lets hdapi_torque_calib be dry-run against
  // the sim, which is the only way to validate that procedure before it touches
  // an arm.
  double raw_units_per_nm = 1000.0;

  // Optional seed for reset(): a Cartesian tip position (device frame,
  // metres) to start the arm's joint state AT, instead of the fixed
  // {0, 0.45, 0.90} default below. Off by default (has_init_pose_m=false).
  //
  // WHY THIS EXISTS: a real-vs-sim comparison harness anchors a shared
  // Cartesian target to the REAL device's own starting position (see
  // bridge_master_slave.py's right_anchor), then sends that same absolute
  // target to this sim twin. If the twin's own state starts at the fixed
  // default pose instead, the resulting "tracking error" from frame zero is
  // just the gap between two unrelated reference points, not anything about
  // friction or contact -- and was confirmed (2026-08-26 diagnostic) to be
  // large enough to slam all three joints into their travel limits before
  // any real disturbance occurs. Setting has_init_pose_m/init_pose_m lets
  // the launcher seed the twin at (approximately) the real device's actual
  // starting Cartesian position, via solveIkToPosition() in reset() below.
  bool has_init_pose_m = false;
  Vec3 init_pose_m{0.0, 0.0, 0.0};
};

// ---------------------------------------------------------------------------
// Small local inverse kinematics: iteratively drive q toward a target
// Cartesian tip position, using the SAME Jacobian the DOB and the real node
// use (touch_haptics::jacobian / fkFromJacobianConvention). This exists only
// to seed ArmModel::reset() with a joint state close to a given Cartesian
// pose -- it is not used anywhere in the runtime control/integration path.
// Damped Gauss-Newton with partial-pivot Gaussian elimination for the 3x3
// solve; bails out (returns false) on a near-singular Jacobian rather than
// producing a garbage step. The caller uses q_inout either way -- it holds
// the best iterate reached, converged or not.
// ---------------------------------------------------------------------------
// q_min/q_max default to a very wide range (no effective constraint) so
// existing callers that do not care about joint limits are unaffected.
// ArmModel::reset() passes its own p_.q_min/p_.q_max: without clamping
// EVERY iteration (not just the final result), the Newton walk can leave
// the feasible region entirely and converge to a mathematically-valid but
// physically-unreachable branch (e.g. wrapping past +-pi/2 on a joint that
// cannot actually turn that far) -- confirmed happening in practice: an
// unclamped solve for a near-workspace-centre target converged to
// q=(-1.6, 1.6, 0.85), i.e. waist and shoulder BOTH already sitting exactly
// on their limits, which is not what "close to the real device's actual
// pose" is supposed to mean. Clamping inside the loop keeps every iterate
// feasible, so a target near the edge of reach converges to the closest
// feasible point instead of a bogus out-of-range one.
inline bool solveIkToPosition(const Vec3 &target_m, Vec3 &q_inout,
                               const Vec3 &q_min = {-1e3, -1e3, -1e3},
                               const Vec3 &q_max = {1e3, 1e3, 1e3},
                               int max_iters = 25, double tol_m = 1e-4)
{
  Vec3 q = q_inout;
  for (int i = 0; i < 3; ++i) q[i] = std::min(std::max(q[i], q_min[i]), q_max[i]);
  bool converged = false;
  for (int iter = 0; iter < max_iters; ++iter) {
    const Vec3 fk = touch_haptics::fkFromJacobianConvention(q);
    const Vec3 err{target_m[0] - fk[0], target_m[1] - fk[1], target_m[2] - fk[2]};
    const double err_norm = std::sqrt(err[0] * err[0] + err[1] * err[1] + err[2] * err[2]);
    if (err_norm < tol_m) { converged = true; break; }

    Mat3 Jc = touch_haptics::jacobian(q);
    Vec3 b = err;
    bool singular = false;
    for (int col = 0; col < 3 && !singular; ++col) {
      int piv = col;
      double best = std::fabs(Jc[piv * 3 + col]);
      for (int r = col + 1; r < 3; ++r) {
        const double v = std::fabs(Jc[r * 3 + col]);
        if (v > best) { best = v; piv = r; }
      }
      if (best < 1e-9) { singular = true; break; }
      if (piv != col) {
        for (int c = 0; c < 3; ++c) std::swap(Jc[col * 3 + c], Jc[piv * 3 + c]);
        std::swap(b[col], b[piv]);
      }
      for (int r = col + 1; r < 3; ++r) {
        const double f = Jc[r * 3 + col] / Jc[col * 3 + col];
        for (int c = col; c < 3; ++c) Jc[r * 3 + c] -= f * Jc[col * 3 + c];
        b[r] -= f * b[col];
      }
    }
    if (singular) break;

    Vec3 dq{};
    for (int r = 2; r >= 0; --r) {
      double s = b[r];
      for (int c = r + 1; c < 3; ++c) s -= Jc[r * 3 + c] * dq[c];
      dq[r] = s / Jc[r * 3 + r];
    }
    // Damped step (0.7x): full Newton steps can overshoot and oscillate
    // near this arm's workspace edges; damping trades a few extra
    // iterations for not diverging.
    for (int i = 0; i < 3; ++i) {
      q[i] = std::min(std::max(q[i] + 0.7 * dq[i], q_min[i]), q_max[i]);
    }
  }
  q_inout = q;
  return converged;
}

// Multi-start wrapper: solveIkToPosition() above is a local (Newton) solver
// and can land on a different branch depending on where it starts -- a
// redundant/aliased solution that satisfies FK(q)=target algebraically but
// sits hard against a joint limit is a real failure mode seen in practice
// (see the comment above). A 4x4x4 grid of seeds spanning [q_min, q_max],
// each locally polished and clamped, was checked against a 2000-restart
// bounded least-squares reference (scipy) on a real recorded anchor point:
// grid result 0.0923 m vs. the reference's 0.0895 m true minimum -- within
// ~3%, i.e. this reliably finds close to the GLOBAL optimum, not just
// "a" local one. Runs once, at reset(), so 64 solves costing microseconds
// each is cheap.
//
// IMPORTANT, and worth knowing before trusting a "seeded" session's t=0
// numbers: for that same real anchor, 89.5 mm was the best ANY seed could
// do within joint limits. That is not a solver defect -- it means this
// simplified 3-link model (L1=L2=0.135 m, the same kinematics touch_dob.hpp
// and the DOB use) genuinely cannot reach every Cartesian point the real
// device's actual workspace covers. A seeded sim session should always be
// checked against the residual this function's caller logs, not assumed to
// land within "a few mm" -- some real starting poses are simply outside
// what this model can represent, and the honest residual is a few tens of
// mm, not zero.
inline bool solveIkToPositionMultiStart(const Vec3 &target_m, Vec3 &q_inout,
                                         const Vec3 &q_min, const Vec3 &q_max,
                                         int max_iters = 25, double tol_m = 1e-4)
{
  Vec3 best_q = q_inout;
  double best_err = std::numeric_limits<double>::infinity();
  bool best_converged = false;
  constexpr int kGridN = 4;
  for (int a = 0; a < kGridN; ++a) {
    for (int b = 0; b < kGridN; ++b) {
      for (int c = 0; c < kGridN; ++c) {
        const Vec3 seed{
          q_min[0] + (q_max[0] - q_min[0]) * a / (kGridN - 1),
          q_min[1] + (q_max[1] - q_min[1]) * b / (kGridN - 1),
          q_min[2] + (q_max[2] - q_min[2]) * c / (kGridN - 1),
        };
        Vec3 q = seed;
        const bool ok = solveIkToPosition(target_m, q, q_min, q_max, max_iters, tol_m);
        const Vec3 fk = touch_haptics::fkFromJacobianConvention(q);
        const Vec3 err{target_m[0] - fk[0], target_m[1] - fk[1], target_m[2] - fk[2]};
        const double err_norm = std::sqrt(err[0] * err[0] + err[1] * err[1] + err[2] * err[2]);
        if (err_norm < best_err) {
          best_err = err_norm;
          best_q = q;
          best_converged = ok;
        }
        if (ok && best_err < tol_m) { q_inout = best_q; return true; }
      }
    }
  }
  q_inout = best_q;
  return best_converged;
}

// ---------------------------------------------------------------------------
// External force applied at the tip, in device Cartesian coordinates (metres,
// newtons). Supplied by the environment/hand models.
// ---------------------------------------------------------------------------
struct ExternalWrench
{
  Vec3 f_tip_n{0.0, 0.0, 0.0};
};

// ---------------------------------------------------------------------------
// The arm.
// ---------------------------------------------------------------------------
class ArmModel
{
public:
  explicit ArmModel(const ArmParams &p = ArmParams{}) : p_(p) { reset(); }

  void reset()
  {
    // A pose that holds itself reasonably and sits inside the gravity map's
    // fitted range (q2 in [+0.175, +1.359]), so the sim's default start is not
    // already extrapolating the model under test.
    q_ = {0.0, 0.45, 0.90};

    if (p_.has_init_pose_m) {
      // Seed from a real device's actual starting Cartesian position instead
      // -- see ArmParams::has_init_pose_m for why. Bounded, multi-start IK
      // (see solveIkToPositionMultiStart): a target near the edge of what
      // this simplified kinematic model can reach converges to the closest
      // FEASIBLE point rather than an out-of-range branch, so q_ always ends
      // up inside [q_min, q_max] without needing a post-hoc clamp that would
      // silently throw the solution away.
      Vec3 q_seed = q_;
      solveIkToPositionMultiStart(p_.init_pose_m, q_seed, p_.q_min, p_.q_max);
      q_ = q_seed;
      const Vec3 achieved = touch_haptics::fkFromJacobianConvention(q_);
      const Vec3 resid{p_.init_pose_m[0] - achieved[0], p_.init_pose_m[1] - achieved[1],
                        p_.init_pose_m[2] - achieved[2]};
      const double resid_norm =
        std::sqrt(resid[0] * resid[0] + resid[1] * resid[1] + resid[2] * resid[2]);
      std::fprintf(stderr,
                   "[hd_sim] init-pose seed: target=(%.4f,%.4f,%.4f) achieved=(%.4f,%.4f,%.4f) "
                   "residual=%.4f m%s\n",
                   p_.init_pose_m[0], p_.init_pose_m[1], p_.init_pose_m[2],
                   achieved[0], achieved[1], achieved[2], resid_norm,
                   resid_norm > 0.01 ? " -- outside this model's reach at this pose" : "");
    }

    qdot_ = {0.0, 0.0, 0.0};
    q_reported_ = q_;
    tau_applied_ = {0.0, 0.0, 0.0};
    tau_prev_ = {0.0, 0.0, 0.0};
    f_ext_last_ = {0.0, 0.0, 0.0};
  }

  const ArmParams &params() const { return p_; }
  ArmParams &mutableParams() { return p_; }

  // TRUE state -- for ground-truth export only. The node must never see this.
  const Vec3 &qTrue() const { return q_; }
  const Vec3 &qdotTrue() const { return qdot_; }

  // What the encoders report: quantised. This is what HD_CURRENT_JOINT_ANGLES
  // returns, and the difference between this and qTrue() is a large part of
  // what the observer has to cope with.
  const Vec3 &qReported() const { return q_reported_; }

  // Previous frame's applied joint torque, N*m. HD_LAST_JOINT_TORQUE reports
  // this one frame late, which is the semantic JointDobBank's alignment was
  // built around -- see step().
  const Vec3 &tauPrevNm() const { return tau_prev_; }
  const Vec3 &tauAppliedNm() const { return tau_applied_; }

  const Vec3 &lastExternalForce() const { return f_ext_last_; }

  Vec3 tipPositionM() const
  {
    return touch_haptics::fkFromJacobianConvention(q_reported_);
  }

  Vec3 tipVelocityMps() const
  {
    const Mat3 J = touch_haptics::jacobian(q_reported_);
    Vec3 v{};
    for (int i = 0; i < 3; ++i) {
      v[i] = J[i * 3 + 0] * qdot_[0] + J[i * 3 + 1] * qdot_[1] + J[i * 3 + 2] * qdot_[2];
    }
    return v;
  }

  // Gravity torque actually acting on the arm, including the base-tilt term the
  // observer's model cannot represent.
  Vec3 gravityTrueNm() const
  {
    const double q1 = q_[0], q2 = q_[1], q3 = q_[2];
    Vec3 g{};
    for (int j = 0; j < 3; ++j) {
      const auto &c = p_.grav_coef[j];
      g[j] = p_.gravity_mismatch *
             (c[0] * std::cos(q2) + c[1] * std::sin(q2) +
              c[2] * std::cos(q3) + c[3] * std::sin(q3));
    }
    // Base tilt gives gravity a horizontal component, which loads the waist as
    // a sinusoid in q1 scaled by reach. Structurally outside the observer's
    // q2/q3-only basis, so it appears as pure disturbance -- exactly as on the
    // real rig.
    const double r = touch_haptics::kL1Meters * std::cos(q2) +
                     touch_haptics::kL2Meters * std::sin(q3);
    const double reach_norm = r / (touch_haptics::kL1Meters + touch_haptics::kL2Meters);
    g[0] += p_.tilt_torque_scale * reach_norm *
            std::sin(q1 + p_.base_tilt_phase_rad) *
            (p_.base_tilt_rad / 0.0785);
    return g;
  }

  Vec3 frictionTrueNm() const
  {
    Vec3 f{};
    for (int j = 0; j < 3; ++j) {
      const double v = qdot_[j];
      const double s = v / p_.stribeck_vel;
      const double stribeck =
        p_.fric_coulomb[j] + (p_.fric_static[j] - p_.fric_coulomb[j]) * std::exp(-s * s);
      // tanh, not sign: a hard sign() in a fixed-step integrator produces
      // stiction chatter that looks exactly like a control instability and is
      // purely a numerical artefact. Chasing that would waste a day.
      f[j] = stribeck * std::tanh(v / p_.fric_eps) + p_.fric_viscous[j] * v;
    }
    return f;
  }

  // Advance one servo tick.
  //   tau_cmd_nm : joint torque the driver decided to apply this frame
  //   ext        : external tip force (environment or hand), device frame
  void step(const Vec3 &tau_cmd_nm, const ExternalWrench &ext, double dt)
  {
    // HD_LAST_* semantics: what the device reports as "last" is the command
    // from the PREVIOUS frame. Snapshot before overwriting, so the one-frame
    // lag the observer aligns against is reproduced exactly rather than
    // approximated. Getting this wrong would either hide a real alignment bug
    // or invent one that does not exist on hardware.
    tau_prev_ = tau_applied_;

    Vec3 tau = tau_cmd_nm;
    for (int j = 0; j < 3; ++j) {
      tau[j] = std::min(std::max(tau[j], -p_.tau_motor_max[j]), p_.tau_motor_max[j]);
    }
    tau_applied_ = tau;

    // External force -> joint torque through the SAME Jacobian the node uses.
    const Mat3 J = touch_haptics::jacobian(q_);
    const Vec3 tau_ext = touch_haptics::jacobianTransposeTimes(J, ext.f_tip_n);
    f_ext_last_ = ext.f_tip_n;

    const Vec3 g = gravityTrueNm();
    const Vec3 fr = frictionTrueNm();

    // M qddot = tau + J^T F_ext - G - friction
    Vec3 rhs{};
    for (int j = 0; j < 3; ++j) {
      rhs[j] = tau[j] + tau_ext[j] - g[j] - fr[j];
    }
    // Optional coupling: rhs -= M_offdiag * qddot_prev (explicit, small).
    // Zero by default; see ArmParams::m_coupling.
    Vec3 qddot{};
    for (int j = 0; j < 3; ++j) {
      const double m = p_.m_n[j] * p_.inertia_mismatch;
      double coupling = 0.0;
      for (int k = 0; k < 3; ++k) {
        if (k != j) coupling += p_.m_coupling[j * 3 + k] * qddot_prev_[k];
      }
      qddot[j] = (rhs[j] - coupling) / m;
    }
    qddot_prev_ = qddot;

    // Semi-implicit (symplectic) Euler: velocity first, then position with the
    // NEW velocity. Explicit Euler is unstable with these damping terms;
    // semi-implicit is stable here (omega*dt ~ 0.03) and conserves energy far
    // better, which matters because passivity is one of the things under test.
    for (int j = 0; j < 3; ++j) {
      qdot_[j] += qddot[j] * dt;
      q_[j] += qdot_[j] * dt;

      // Joint limits as a hard stop with restitution 0 -- a runaway in a
      // controller test should hit a wall, not integrate to infinity.
      if (q_[j] < p_.q_min[j]) { q_[j] = p_.q_min[j]; if (qdot_[j] < 0) qdot_[j] = 0.0; }
      if (q_[j] > p_.q_max[j]) { q_[j] = p_.q_max[j]; if (qdot_[j] > 0) qdot_[j] = 0.0; }
    }

    // Encoder quantisation. Applied to the REPORTED angle only; the true state
    // stays continuous. This is the dominant noise source the observer sees.
    for (int j = 0; j < 3; ++j) {
      q_reported_[j] = std::round(q_[j] / p_.encoder_quantum_rad) * p_.encoder_quantum_rad;
    }
  }

private:
  ArmParams p_;
  Vec3 q_{}, qdot_{}, q_reported_{};
  Vec3 qddot_prev_{0.0, 0.0, 0.0};
  Vec3 tau_applied_{}, tau_prev_{};
  Vec3 f_ext_last_{};
};

}  // namespace touch_sim
