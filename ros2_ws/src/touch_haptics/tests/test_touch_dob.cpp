// test_touch_dob.cpp
//
// Unit tests for the gravity-compensated disturbance observer in
// touch_dob.hpp. Header-only under test, so this is a single self-contained
// translation unit with no framework dependency.
//
// BUILD / RUN:
//   g++ -std=gnu++17 -I../include test_touch_dob.cpp -o test_touch_dob && ./test_touch_dob
//
// The central test is STEADY-STATE SIGN (test 4). It is the software
// equivalent of the hardware check that validated the identification
// pipeline: there, the ratio J^T(hold bias) / g_map measured -1.02, proving
// that the fitted map equals the torque required to hold the arm. Here, the
// same fact is asserted at the observer level -- feed tau_cmd == G(q) at
// rest and tau_dis_hat must converge to zero. If the sign were flipped the
// estimate would settle at -2*G(q) instead, which is exactly the failure that
// produced negative inertia and R^2 = -12 on real data before it was caught.

#include "touch_haptics/touch_dob.hpp"

#include <cmath>
#include <cstdio>
#include <string>

static int g_failures = 0;

static void check(bool cond, const std::string &what)
{
  if (cond) {
    printf("  PASS  %s\n", what.c_str());
  } else {
    printf("  FAIL  %s\n", what.c_str());
    ++g_failures;
  }
}

static void checkNear(double got, double want, double tol, const std::string &what)
{
  const bool ok = std::fabs(got - want) <= tol;
  printf("  %s  %-58s got %+.9f want %+.9f (tol %.1e)\n",
         ok ? "PASS" : "FAIL", what.c_str(), got, want, tol);
  if (!ok) ++g_failures;
}

// A params set that is valid EXCEPT that the caller chooses the gravity model.
static touch_haptics::JointDobParams makeParams(double m_n, double g_dis)
{
  touch_haptics::JointDobParams p;
  p.m_n_kg_m2 = m_n;
  p.g_dis_rad_s = g_dis;
  p.threshold_nm = 1.0;      // deliberately high so the gate stays shut and
                             // correct() exercises the cancellation branch
  p.max_torque_nm = 0.5;
  p.gate_release_ratio = 1.0;
  return p;
}

// Gravity coefficients from session B (fit_gravity.py, --basis minimal),
// reused from test 4 so the two follow-up tests assert against the same model
// the sign check was validated on.
static const std::array<double, touch_haptics::kNumGravityCoefs> kGravWaist{
  -0.00730, 0.00262, 0.00448, 0.00308};
static const std::array<double, touch_haptics::kNumGravityCoefs> kGravShoulder{
  0.11979, 0.03195, -0.03347, -0.04448};
static const std::array<double, touch_haptics::kNumGravityCoefs> kGravElbow{
  -0.00108, -0.01253, 0.00057, 0.08863};

// A DobConfig that is valid on all three joints. The torque scale is left at
// its uncalibrated sentinel; callers opt in explicitly, because whether the
// scale is calibrated is precisely what test 10 varies.
static touch_haptics::DobConfig makeBankConfig()
{
  touch_haptics::DobConfig cfg{};
  const double m[3] = {3.3694e-3, 2.9452e-3, 2.3291e-4};   // inertia_left.json
  for (int i = 0; i < touch_haptics::kNumActiveJoints; ++i) {
    cfg.joints[i].m_n_kg_m2 = m[i];
    cfg.joints[i].g_dis_rad_s = 2.0 * M_PI * 5.0;
    cfg.joints[i].threshold_nm = 1.0;    // high, so the gate stays shut
    cfg.joints[i].max_torque_nm = 0.5;
    cfg.joints[i].gate_release_ratio = 1.0;
  }
  cfg.joints[0].grav_coef = kGravWaist;
  cfg.joints[1].grav_coef = kGravShoulder;
  cfg.joints[2].grav_coef = kGravElbow;
  return cfg;
}

int main()
{
  const double ts = 1.0 / 1019.0;      // measured servo period

  // -----------------------------------------------------------------------
  printf("\n[1] unset gravity model must be rejected, not silently treated as zero\n");
  {
    touch_haptics::JointDobParams p = makeParams(3.3694e-3, 2.0 * M_PI * 5.0);
    check(!p.gravityModelLoaded(), "default grav_coef reads as not loaded");
    check(!p.valid(), "valid() is false with gravity unset");
    check(p.invalidReason() != nullptr, "invalidReason() explains the refusal");
    if (p.invalidReason()) printf("        reason: %s\n", p.invalidReason());

    touch_haptics::JointDob dob;
    check(!dob.configure(p, ts), "configure() refuses params with unset gravity");
    check(!dob.configured(), "observer stays unconfigured");
    checkNear(dob.update(0.5, 1.0, 0.0), 0.0, 0.0,
              "update() on an unconfigured observer is inert");
  }

  // -----------------------------------------------------------------------
  printf("\n[2] a loaded gravity model makes params valid\n");
  {
    touch_haptics::JointDobParams p = makeParams(3.3694e-3, 2.0 * M_PI * 5.0);
    p.grav_coef = {0.11979, 0.03195, -0.03347, -0.04448};   // shoulder, session B
    check(p.gravityModelLoaded(), "gravityModelLoaded() true once set");
    check(p.valid(), "valid() true");
    check(p.invalidReason() == nullptr, "invalidReason() returns nullptr when valid");

    touch_haptics::JointDob dob;
    check(dob.configure(p, ts), "configure() succeeds");
  }

  // -----------------------------------------------------------------------
  printf("\n[3] G(q) evaluates the documented basis in the documented order\n");
  {
    touch_haptics::JointDobParams p = makeParams(1.0e-3, 1.0);
    // One coefficient at a time isolates each basis term, so a transposed or
    // reordered basis cannot pass.
    const double q2 = 0.37, q3 = 1.12;
    p.grav_coef = {1.0, 0.0, 0.0, 0.0};
    checkNear(p.gravityTorqueNm(q2, q3), std::cos(q2), 1e-12, "term 0 is cos(q2)");
    p.grav_coef = {0.0, 1.0, 0.0, 0.0};
    checkNear(p.gravityTorqueNm(q2, q3), std::sin(q2), 1e-12, "term 1 is sin(q2)");
    p.grav_coef = {0.0, 0.0, 1.0, 0.0};
    checkNear(p.gravityTorqueNm(q2, q3), std::cos(q3), 1e-12, "term 2 is cos(q3)");
    p.grav_coef = {0.0, 0.0, 0.0, 1.0};
    checkNear(p.gravityTorqueNm(q2, q3), std::sin(q3), 1e-12, "term 3 is sin(q3)");
  }

  // -----------------------------------------------------------------------
  printf("\n[4] STEADY-STATE SIGN: at rest with tau_cmd == G(q), tau_dis_hat -> 0\n");
  printf("    (software equivalent of the J^T(bias)/g_map = -1.02 hardware check)\n");
  {
    touch_haptics::JointDobParams p = makeParams(2.9452e-3, 2.0 * M_PI * 5.0);
    p.grav_coef = {0.11979, 0.03195, -0.03347, -0.04448};   // shoulder, session B
    touch_haptics::JointDob dob;
    check(dob.configure(p, ts), "configured");

    const double q2 = 0.754, q3 = 1.056;      // session B pose 2
    const double g_tau = p.gravityTorqueNm(q2, q3);
    printf("        G(q) at this pose = %+.6f N*m\n", g_tau);
    check(std::fabs(g_tau) > 0.01, "the test pose has non-trivial gravity");

    // At rest the arm is held by exactly its own weight: tau_cmd == G(q).
    for (int k = 0; k < 20000; ++k) {
      dob.update(g_tau, 0.0, g_tau);
    }
    checkNear(dob.tauDisHatNm(), 0.0, 1e-9,
              "tau_dis_hat converges to zero (gravity fully removed)");
    check(!dob.contactSuspected(), "gate stays shut -- weight is not contact");

    // The inverted-sign failure, asserted explicitly so a future regression
    // cannot pass silently: adding G instead of subtracting settles at 2*G.
    touch_haptics::JointDob wrong;
    wrong.configure(p, ts);
    for (int k = 0; k < 20000; ++k) {
      wrong.update(g_tau, 0.0, -g_tau);       // sign flipped
    }
    checkNear(wrong.tauDisHatNm(), 2.0 * g_tau, 1e-6,
              "flipped sign settles at 2*G(q), i.e. gravity counted twice");
  }

  // -----------------------------------------------------------------------
  printf("\n[5] without compensation the same rest pose reports its weight as disturbance\n");
  {
    touch_haptics::JointDobParams p = makeParams(2.9452e-3, 2.0 * M_PI * 5.0);
    p.grav_coef = {0.11979, 0.03195, -0.03347, -0.04448};
    touch_haptics::JointDob dob;
    dob.configure(p, ts);
    const double g_tau = p.gravityTorqueNm(0.754, 1.056);

    for (int k = 0; k < 20000; ++k) {
      dob.update(g_tau, 0.0, 0.0);            // gravity NOT passed
    }
    checkNear(dob.tauDisHatNm(), g_tau, 1e-6,
              "uncompensated estimate settles at exactly G(q)");
    printf("        this is the 142%%-of-torque-authority failure, reproduced\n");
  }

  // -----------------------------------------------------------------------
  printf("\n[6] a genuine constant disturbance is still recovered (DC gain 1)\n");
  {
    touch_haptics::JointDobParams p = makeParams(2.9452e-3, 2.0 * M_PI * 5.0);
    p.grav_coef = {0.11979, 0.03195, -0.03347, -0.04448};
    touch_haptics::JointDob dob;
    dob.configure(p, ts);
    const double g_tau = p.gravityTorqueNm(0.754, 1.056);
    const double contact = 0.040;             // N*m of real external torque

    for (int k = 0; k < 20000; ++k) {
      dob.update(g_tau + contact, 0.0, g_tau);
    }
    checkNear(dob.tauDisHatNm(), contact, 1e-6,
              "gravity removed, contact preserved");
  }

  // -----------------------------------------------------------------------
  printf("\n[7] gravity path does not disturb the velocity path\n");
  {
    // Same qdot trajectory, two gravity values. The difference in the estimate
    // must equal the difference in G alone -- i.e. gravity enters additively
    // and never multiplies or lags the qdot term.
    touch_haptics::JointDobParams p = makeParams(2.9452e-3, 2.0 * M_PI * 20.0);
    p.grav_coef = {0.11979, 0.03195, -0.03347, -0.04448};
    touch_haptics::JointDob a, b;
    a.configure(p, ts);
    b.configure(p, ts);
    const double gA = 0.0, gB = 0.037;
    // The first samples are skipped deliberately. The two observers start from
    // the same zero state, so their difference has to charge through the LPF
    // before it can equal the constant gravity offset -- it begins at 0 and
    // reaches gB-gA after a few time constants. That transient is correct
    // behaviour, not leakage between the paths, so measuring through it would
    // assert something untrue. At g_dis = 2*pi*20 the time constant is ~8 ms
    // (~8 samples); 1000 is comfortably past it.
    const int kSettle = 1000;
    double worst = 0.0;
    for (int k = 0; k < 6000; ++k) {
      const double qdot = 0.8 * std::sin(2.0 * M_PI * 3.0 * k * ts);
      const double tau = 0.01 * std::cos(2.0 * M_PI * 3.0 * k * ts);
      const double ea = a.update(tau, qdot, gA);
      const double eb = b.update(tau, qdot, gB);
      if (k >= kSettle) {
        worst = std::max(worst, std::fabs((ea - eb) - (gB - gA)));
      }
    }
    checkNear(worst, 0.0, 1e-9,
              "estimate difference equals the gravity difference, at all speeds");
  }

  // -----------------------------------------------------------------------
  printf("\n[8] bank-level: unset gravity leaves the whole bank unconfigured\n");
  {
    touch_haptics::DobConfig cfg{};           // everything at defaults
    touch_haptics::JointDobBank bank;
    check(!bank.configure(cfg, ts), "bank refuses default (unset) config");
    check(!bank.configured(), "bank reports unconfigured");
    std::array<double, touch_haptics::kNumActiveJoints> q{0.1, 0.75, 1.05};
    std::array<double, touch_haptics::kNumActiveJoints> t{0.0, 0.0, 0.0};
    check(!bank.update(q, t), "bank update() is a no-op while unconfigured");
    const auto est = bank.tauDisHatNm();
    check(est[0] == 0.0 && est[1] == 0.0 && est[2] == 0.0,
          "estimates stay exactly zero, never NaN, while unconfigured");
  }

  // -----------------------------------------------------------------------
  printf("\n[9] FRAME ALIGNMENT: gravity uses the PREVIOUS pose, not the current one\n");
  printf("    HD_LAST_JOINT_TORQUE reports the command issued one frame ago, so it\n");
  printf("    must be paired with the pose held then. Reading q_rad instead would\n");
  printf("    pair a torque with a pose one sample in its future, injecting a\n");
  printf("    dG/dt * Ts artefact that grows with joint speed. Nothing else in this\n");
  printf("    suite would notice the swap: every other test holds the pose still.\n");
  {
    touch_haptics::DobConfig cfg = makeBankConfig();
    cfg.last_joint_torque_scale_nm_per_unit = 1.0;    // calibrated
    check(cfg.torqueScaleCalibrated(), "torque scale reads as calibrated");

    touch_haptics::JointDobBank bank;
    check(bank.configure(cfg, ts), "bank configures with valid params");

    // Poses separated far beyond quantisation noise, and both away from zero
    // so a broken evaluator cannot pass by returning something near-zero.
    const std::array<double, 3> q0{0.10, 0.754, 1.056};
    const std::array<double, 3> q1{0.55, 1.120, 0.610};
    check(std::fabs(q1[1] - q0[1]) > 0.3 && std::fabs(q1[2] - q0[2]) > 0.3,
          "q0 and q1 differ by > 0.3 rad on both q2 and q3");

    const std::array<double, 3> tau_raw{0.02, 0.05, 0.01};

    // First call only primes the history and returns false; no gravity yet.
    check(!bank.update(q0, tau_raw), "first update() primes and reports unusable");

    // Second call: q_prev_ is q0, so gravity must be evaluated there.
    check(bank.update(q1, tau_raw), "second update() is usable");
    const auto pose = bank.lastGravityEvalPoseRad();
    checkNear(pose[0], q0[1], 1e-12, "recorded q2 is q0's, not q1's");
    checkNear(pose[1], q0[2], 1e-12, "recorded q3 is q0's, not q1's");

    // Explicit negative guard, in the style of test 4's flipped-sign check: an
    // accidental swap to q_rad must fail loudly, not slip through on proximity.
    check(std::fabs(pose[0] - q1[1]) > 0.3, "recorded q2 is NOT q1's (>0.3 rad away)");
    check(std::fabs(pose[1] - q1[2]) > 0.3, "recorded q3 is NOT q1's (>0.3 rad away)");

    // And the torque it produces is G(q0), computed independently here from the
    // basis order pinned down in test 3.
    const auto &gp = cfg.joints[1];
    const double g_at_q0 = gp.grav_coef[0] * std::cos(q0[1]) +
                           gp.grav_coef[1] * std::sin(q0[1]) +
                           gp.grav_coef[2] * std::cos(q0[2]) +
                           gp.grav_coef[3] * std::sin(q0[2]);
    checkNear(gp.gravityTorqueNm(pose[0], pose[1]), g_at_q0, 1e-12,
              "G(recorded pose) equals hand-computed G(q0)");
    const double g_at_q1 = gp.grav_coef[0] * std::cos(q1[1]) +
                           gp.grav_coef[1] * std::sin(q1[1]) +
                           gp.grav_coef[2] * std::cos(q1[2]) +
                           gp.grav_coef[3] * std::sin(q1[2]);
    check(std::fabs(g_at_q0 - g_at_q1) > 0.01,
          "G(q0) and G(q1) are far apart, so the test can discriminate");

    // Multi-step trajectory: a single-step check would still pass if someone
    // special-cased the first frame. Assert the lag at EVERY step.
    touch_haptics::JointDobBank traj;
    traj.configure(cfg, ts);
    std::array<double, 3> prev{0.0, 0.0, 0.0};
    double worst_lag_err = 0.0;
    bool primed = false;
    for (int k = 0; k < 10; ++k) {
      // Slow sinusoid on both q2 and q3, deliberately out of phase.
      std::array<double, 3> q{0.2,
                              0.75 + 0.35 * std::sin(0.7 * k),
                              1.05 + 0.35 * std::cos(0.5 * k)};
      traj.update(q, tau_raw);
      if (primed) {
        const auto p = traj.lastGravityEvalPoseRad();
        worst_lag_err = std::max(worst_lag_err,
                                 std::max(std::fabs(p[0] - prev[1]),
                                          std::fabs(p[1] - prev[2])));
      }
      prev = q;
      primed = true;
    }
    checkNear(worst_lag_err, 0.0, 1e-12,
              "one-frame lag holds at every step of a 10-frame trajectory");
  }

  // -----------------------------------------------------------------------
  printf("\n[10] UNCALIBRATED TORQUE SCALE must suppress gravity subtraction\n");
  printf("     With the scale at its sentinel, tau_cmd is forced to zero. Subtracting\n");
  printf("     a real G(q) there would leave -G(q) standing as a large fictitious\n");
  printf("     disturbance -- 'confidently wrong', which is the failure mode the\n");
  printf("     sentinel pattern exists to prevent. Estimates must stay at zero.\n");
  {
    touch_haptics::DobConfig cfg = makeBankConfig();
    // Scale deliberately left at kUncalibratedTorqueScale.
    check(cfg.last_joint_torque_scale_nm_per_unit ==
            touch_haptics::kUncalibratedTorqueScale,
          "scale is at the uncalibrated sentinel");
    check(!cfg.torqueScaleCalibrated(), "torqueScaleCalibrated() is false");
    for (int i = 0; i < touch_haptics::kNumActiveJoints; ++i) {
      check(cfg.joints[i].valid(), "joint params otherwise valid (gravity loaded)");
    }

    touch_haptics::JointDobBank bank;
    check(bank.configure(cfg, ts), "bank still configures -- scale is a runtime gate");

    const std::array<double, 3> q{0.10, 0.754, 1.056};   // test 4's pose
    const std::array<double, 3> tau_raw{50.0, 120.0, 40.0};   // nonzero raw units
    const double g_shoulder = cfg.joints[1].gravityTorqueNm(q[1], q[2]);
    printf("         G(q) on shoulder = %+.6f N*m\n", g_shoulder);
    check(std::fabs(g_shoulder) > 0.01, "the pose has non-trivial gravity");

    bank.update(q, tau_raw);                       // prime
    check(!bank.update(q, tau_raw), "update() returns false while uncalibrated");
    for (int k = 0; k < 5000; ++k) {
      bank.update(q, tau_raw);
    }
    const auto est = bank.tauDisHatNm();
    for (int i = 0; i < touch_haptics::kNumActiveJoints; ++i) {
      checkNear(est[i], 0.0, 1e-12, "estimate stays exactly zero while uncalibrated");
    }
    // The specific wrong value, asserted explicitly so a regression fails on
    // the actual bug rather than generically.
    check(std::fabs(est[1] - (-g_shoulder)) > 0.01,
          "shoulder estimate is NOT -G(q), i.e. gravity was not left standing");

    // State-dependent gate, not a permanent lockout: calibrate and the bank
    // must start producing estimates. This mirrors the real bring-up order,
    // where hdapi_torque_calib runs after the observer is already wired up.
    touch_haptics::DobConfig cal = cfg;
    cal.last_joint_torque_scale_nm_per_unit = 1.0e-3;   // N*m per raw unit
    check(cal.torqueScaleCalibrated(), "scale now reads as calibrated");
    touch_haptics::JointDobBank bank2;
    check(bank2.configure(cal, ts), "reconfigure with a calibrated scale");
    bank2.update(q, tau_raw);                      // prime
    check(bank2.update(q, tau_raw), "update() now returns true");
    for (int k = 0; k < 20000; ++k) {
      bank2.update(q, tau_raw);
    }
    const auto est2 = bank2.tauDisHatNm();
    // At rest, tau_cmd = raw*scale and gravity is removed, so the estimate
    // settles at (raw*scale - G(q)).
    const double expect_shoulder = tau_raw[1] * 1.0e-3 - g_shoulder;
    checkNear(est2[1], expect_shoulder, 1e-6,
              "calibrated: estimate settles at tau_cmd - G(q)");
    check(std::fabs(est2[1]) > 1e-6, "suppression lifted -- estimate is non-zero");
  }

  printf("\n==================================================\n");
  if (g_failures == 0) {
    printf("ALL TESTS PASSED\n");
  } else {
    printf("%d TEST(S) FAILED\n", g_failures);
  }
  printf("==================================================\n");
  return g_failures == 0 ? 0 : 1;
}
