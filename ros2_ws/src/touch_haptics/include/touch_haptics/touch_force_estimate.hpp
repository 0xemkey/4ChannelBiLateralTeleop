// touch_force_estimate.hpp -- pure helper turning JointDobBank output into the
// touch_haptics/ForceEstimate wire fields.
//
// Deliberately NOT part of touch_dob.hpp, which is frozen: this is messaging
// logic (deadband shape, confidence heuristic), not observer math, and it has
// no ROS dependency so it can be unit tested the same way test_touch_dob.cpp
// is -- a bare g++ compile, no framework.
#pragma once

#include "touch_haptics/touch_dob.hpp"

#include <algorithm>
#include <array>
#include <cmath>

namespace touch_haptics
{

struct ForceEstimateResult
{
  std::array<double, kNumActiveJoints> tau_ext_nm{};
  std::array<double, kNumActiveJoints> tau_dis_hat_nm{};
  std::array<double, kNumActiveJoints> tau_ext_deadbanded_nm{};
  std::array<double, kNumActiveJoints> confidence{};
  bool valid{false};
};

// tau_dis_hat_nm -- this frame's JointDobBank::tauDisHatNm() output.
// cfg            -- the same DobConfig the bank was configured with (for
//                   per-joint threshold_nm).
// dob_configured -- JointDobBank::configured(); ForceEstimateResult.valid
//                   mirrors it directly. borrowed_params is a separate,
//                   caller-known fact (which JSON was loaded) and is not
//                   computed here.
//
// Sign convention: tau_ext_hat = -tau_dis_hat, see msg/ForceEstimate.msg.
// Deadband is the SHRINKING form sign*max(0,|tau|-threshold), not a hard
// gate -- a hard gate produces a step, and a step into a human hand is an
// energy injection (the same reasoning as JointDob's gate_release_ratio).
inline ForceEstimateResult computeForceEstimate(
  const std::array<double, kNumActiveJoints> &tau_dis_hat_nm,
  const DobConfig &cfg,
  bool dob_configured)
{
  ForceEstimateResult r;
  r.valid = dob_configured;

  for (int i = 0; i < kNumActiveJoints; ++i) {
    const double dis = tau_dis_hat_nm[i];
    const double ext = -dis;
    r.tau_dis_hat_nm[i] = dis;
    r.tau_ext_nm[i] = ext;

    const double threshold = cfg.joints[i].threshold_nm;
    const double mag = std::fabs(ext);
    const double margin = mag - threshold;

    r.tau_ext_deadbanded_nm[i] = (margin > 0.0) ? std::copysign(margin, ext) : 0.0;

    // Confidence folds deadband margin and observer configuration: 0 at or
    // below threshold, ramping linearly to 1 at twice the threshold. Not a
    // probability -- a monotonic knob for a consumer to scale or refuse a
    // channel by, cheap enough to recompute every servo frame.
    double conf = 0.0;
    if (dob_configured && threshold > 0.0) {
      conf = std::clamp(margin / threshold, 0.0, 1.0);
    }
    r.confidence[i] = conf;
  }

  return r;
}

}  // namespace touch_haptics
