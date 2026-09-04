// environment.hpp
//
// What pushes back on a simulated arm: a contact environment for the slave, and
// an operator's hand impedance for the master. Header-only.
//
// ---------------------------------------------------------------------------
// WHY THE HAND MODEL MATTERS AS MUCH AS THE WALL
// ---------------------------------------------------------------------------
// The documented instability on this rig -- reflection damping driving
// peak-to-peak force from 0.007 N to 0.65 N, a 93x growth -- closed its loop
// THROUGH THE OPERATOR'S HAND. A bilateral controller cannot be assessed for
// stability without some model of what is gripping the master, because the hand
// is part of the loop, not a boundary condition.
//
// bridge_master_slave.py's own comment admits its stability claim rested on a
// simulation whose "hand model is a guess". This file makes the guess explicit,
// parameterised, and swappable, so a stability result can be quoted against a
// stated assumption instead of an unstated one. Three named presets are
// provided (stiff / loose / none) precisely so results carry that label.
//
// The chirp trajectory is the deliverable that matters most: it sweeps the
// closed loop across the band containing the arm's ~4.4 Hz resonance and finds
// the pumping frequency with zero hardware time.
#pragma once

#include "arm_model.hpp"

#include <cmath>
#include <string>

namespace touch_sim
{

// ---------------------------------------------------------------------------
// Contact environment acting on the tip.
// ---------------------------------------------------------------------------
struct EnvParams
{
  enum class Kind { kFree, kPlane, kBox };
  Kind kind = Kind::kFree;

  // Plane: contact when (x - point) . normal < 0, i.e. penetration along -n.
  Vec3 plane_normal{0.0, 1.0, 0.0};
  Vec3 plane_point{0.0, -0.05, 0.0};

  // Box half-extents about box_center; contact on any face.
  Vec3 box_center{0.0, 0.0, 0.20};
  Vec3 box_half{0.05, 0.05, 0.05};

  // Contact impedance. 10 kN/m is "metal", 200 N/m is "foam" -- the reference
  // paper swept ten materials across roughly this range, and robustness across
  // it is the interesting question, not behaviour at any single value.
  double k_contact = 2000.0;   // N/m
  double b_contact = 8.0;      // N*s/m
};

class Environment
{
public:
  explicit Environment(const EnvParams &p = EnvParams{}) : p_(p) {}

  EnvParams &mutableParams() { return p_; }
  const EnvParams &params() const { return p_; }

  // Force the environment applies TO the tip, device frame, newtons.
  ExternalWrench force(const Vec3 &x_m, const Vec3 &v_mps) const
  {
    ExternalWrench w;
    if (p_.kind == EnvParams::Kind::kFree) return w;

    double depth = 0.0;
    Vec3 n{0, 0, 0};

    if (p_.kind == EnvParams::Kind::kPlane) {
      double d = 0.0, nn = 0.0;
      for (int i = 0; i < 3; ++i) {
        d += (x_m[i] - p_.plane_point[i]) * p_.plane_normal[i];
        nn += p_.plane_normal[i] * p_.plane_normal[i];
      }
      nn = std::sqrt(nn);
      if (nn < 1e-12) return w;
      d /= nn;
      if (d >= 0.0) return w;          // not in contact
      depth = -d;
      for (int i = 0; i < 3; ++i) n[i] = p_.plane_normal[i] / nn;
    } else {
      // Box: push out along the least-penetrated axis, which is the standard
      // cheap approximation and is adequate for a contact-detection testbed.
      double best = 1e9;
      int axis = -1, sign = 1;
      for (int i = 0; i < 3; ++i) {
        const double d = x_m[i] - p_.box_center[i];
        if (std::fabs(d) > p_.box_half[i]) return w;   // outside on this axis
        const double pen = p_.box_half[i] - std::fabs(d);
        if (pen < best) { best = pen; axis = i; sign = (d >= 0.0) ? 1 : -1; }
      }
      if (axis < 0) return w;
      depth = best;
      n[axis] = static_cast<double>(sign);
    }

    // Spring-damper, and the damper is one-sided: it resists penetration but
    // never PULLS the tip back as it leaves. A two-sided damper makes the
    // surface sticky and can inject energy on release, which would show up as a
    // spurious instability that the real wall does not have.
    double vn = 0.0;
    for (int i = 0; i < 3; ++i) vn += v_mps[i] * n[i];
    double mag = p_.k_contact * depth;
    if (vn < 0.0) mag += -p_.b_contact * vn;
    if (mag < 0.0) mag = 0.0;
    for (int i = 0; i < 3; ++i) w.f_tip_n[i] = mag * n[i];
    return w;
  }

private:
  EnvParams p_;
};

// ---------------------------------------------------------------------------
// Operator hand on the master.
// ---------------------------------------------------------------------------
struct HandParams
{
  enum class Motion { kNone, kSine, kChirp, kStep, kHold };
  Motion motion = Motion::kNone;

  // Hand impedance. "stiff" ~ a firm grip, "loose" ~ fingertips.
  double k_hand = 120.0;   // N/m
  double b_hand = 4.0;     // N*s/m
  double m_hand = 0.03;    // kg

  // Commanded trajectory, about the arm's start position.
  Vec3 axis{0.0, 1.0, 0.0};
  double amplitude_m = 0.03;
  double f0_hz = 0.2;
  double f1_hz = 20.0;     // chirp end -- spans the ~4.4 Hz resonance
  double duration_s = 30.0;

  static HandParams preset(const std::string &name)
  {
    HandParams h;
    if (name == "stiff")      { h.k_hand = 400.0; h.b_hand = 8.0;  h.m_hand = 0.05; }
    else if (name == "loose") { h.k_hand = 40.0;  h.b_hand = 1.5;  h.m_hand = 0.01; }
    else if (name == "none")  { h.k_hand = 0.0;   h.b_hand = 0.0;  h.m_hand = 0.0;
                                h.motion = Motion::kNone; }
    return h;
  }
};

class Hand
{
public:
  explicit Hand(const HandParams &p = HandParams{}) : p_(p) {}

  HandParams &mutableParams() { return p_; }
  const HandParams &params() const { return p_; }

  void setOrigin(const Vec3 &x0) { x0_ = x0; }

  // Where the hand is trying to be at time t.
  Vec3 commanded(double t) const
  {
    double s = 0.0;
    switch (p_.motion) {
      case HandParams::Motion::kNone: return x0_;
      case HandParams::Motion::kHold: s = 0.0; break;
      case HandParams::Motion::kStep: s = (t > 1.0) ? 1.0 : 0.0; break;
      case HandParams::Motion::kSine: s = std::sin(2.0 * M_PI * p_.f0_hz * t); break;
      case HandParams::Motion::kChirp: {
        // Linear chirp. The phase is the INTEGRAL of the instantaneous
        // frequency, hence the t^2 term -- writing sin(2*pi*f(t)*t) instead is
        // a classic error that produces a different, wrong sweep.
        const double d = (p_.duration_s > 1e-9) ? p_.duration_s : 1.0;
        const double u = std::min(t / d, 1.0);
        const double phase = 2.0 * M_PI *
          (p_.f0_hz * t + 0.5 * (p_.f1_hz - p_.f0_hz) * t * t / d);
        // Raised-cosine envelope: a step at t=0 is a broadband transient that
        // would pollute the very band being swept.
        const double env = 0.5 * (1.0 - std::cos(M_PI * std::min(u / 0.1, 1.0)));
        s = env * std::sin(phase);
        break;
      }
    }
    Vec3 x = x0_;
    for (int i = 0; i < 3; ++i) x[i] += p_.axis[i] * p_.amplitude_m * s;
    return x;
  }

  // Force the hand applies to the stylus.
  ExternalWrench force(double t, const Vec3 &x_m, const Vec3 &v_mps)
  {
    ExternalWrench w;
    if (p_.k_hand <= 0.0 && p_.b_hand <= 0.0) return w;
    const Vec3 xc = commanded(t);
    for (int i = 0; i < 3; ++i) {
      w.f_tip_n[i] = p_.k_hand * (xc[i] - x_m[i]) - p_.b_hand * v_mps[i];
    }
    return w;
  }

private:
  HandParams p_;
  Vec3 x0_{0.0, 0.0, 0.0};
};

}  // namespace touch_sim
