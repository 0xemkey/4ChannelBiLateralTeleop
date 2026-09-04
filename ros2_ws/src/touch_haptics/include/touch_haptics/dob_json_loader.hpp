// dob_json_loader.hpp
//
// Loads identified DOB parameters from the JSON that identification/ emits,
// into a DobConfig. Shared by tools/dob_replay_cpp.cpp (offline sweep) and
// src/touch_slave.cpp (live node) so the two cannot drift apart -- if the
// node loaded parameters differently from the tool that chose them, the
// hardware numbers would not be comparable with the offline prediction, which
// is the entire point of running both.
//
// Deliberately not a general JSON parser. It understands exactly the two file
// shapes this pipeline writes and refuses anything else, which is the right
// trade for a file that decides what torque a haptic device believes in.
//
// WHAT IT REFUSES, AND WHY EACH CHECK EARNS ITS PLACE
// ---------------------------------------------------------------------------
//   * missing sign_convention  -- fit_gravity.py emitted MINUS the holding
//     torque before 2026-08-24. Loading such a file would double gravity
//     rather than remove it: on real data that drove R^2 to -6.9 and produced
//     negative inertia. The field is the only way to tell the two apart from
//     the file alone.
//   * wrong basis order        -- grav_coef is positional. A six-term --basis
//     tilt fit read into four slots is not an error the arithmetic can catch;
//     it silently means something else.
//   * non-positive M_n         -- would make the Q-filter meaningless and is
//     rejected by JointDobParams::valid() anyway; caught here so the message
//     names the file rather than the struct.
#pragma once

#include "touch_haptics/touch_dob.hpp"

#include <cctype>
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

namespace touch_haptics
{
namespace dob_json
{

inline const char *jointKey(int i)
{
  static const char *kNames[3] = {"waist", "shoulder", "elbow"};
  return kNames[i];
}

inline std::string slurp(const std::string &path, std::string *err)
{
  std::ifstream f(path);
  if (!f) {
    if (err) *err = "cannot open " + path;
    return std::string();
  }
  std::stringstream ss;
  ss << f.rdbuf();
  return ss.str();
}

inline size_t findKey(const std::string &s, const std::string &key, size_t from = 0)
{
  return s.find("\"" + key + "\"", from);
}

inline double numberAfter(const std::string &s, size_t pos)
{
  if (pos == std::string::npos) return std::nan("");
  const size_t c = s.find(':', pos);
  if (c == std::string::npos) return std::nan("");
  return std::atof(s.c_str() + c + 1);
}

inline std::vector<double> arrayAfter(const std::string &s, size_t pos)
{
  std::vector<double> out;
  if (pos == std::string::npos) return out;
  const size_t a = s.find('[', pos);
  const size_t b = s.find(']', a);
  if (a == std::string::npos || b == std::string::npos) return out;
  std::stringstream ss(s.substr(a + 1, b - a - 1));
  std::string tok;
  while (std::getline(ss, tok, ',')) out.push_back(std::atof(tok.c_str()));
  return out;
}

inline std::string stripSpace(const std::string &s)
{
  std::string o;
  for (char c : s) {
    if (!std::isspace(static_cast<unsigned char>(c))) o += c;
  }
  return o;
}

// Fills m_n_kg_m2 and grav_coef for all three joints. Leaves every tuned
// field (g_dis_rad_s, threshold_nm, max_torque_nm, gate_release_ratio) and the
// torque scale untouched -- those are the caller's decision, not the JSON's.
//
// Returns false with *err set on any problem, having modified nothing.
inline bool loadIdentifiedParams(const std::string &inertia_path,
                                 const std::string &gravity_path,
                                 DobConfig *cfg,
                                 std::string *err)
{
  if (!cfg) {
    if (err) *err = "null cfg";
    return false;
  }

  std::string e;
  const std::string ij = slurp(inertia_path, &e);
  if (ij.empty()) { if (err) *err = e.empty() ? ("empty " + inertia_path) : e; return false; }
  const std::string gj = slurp(gravity_path, &e);
  if (gj.empty()) { if (err) *err = e.empty() ? ("empty " + gravity_path) : e; return false; }

  if (gj.find("EQUALS G(q)") == std::string::npos) {
    if (err) {
      *err = gravity_path + ": missing or unrecognised \"sign_convention\". This "
             "file predates the 2026-08-24 sign fix, when fit_gravity.py emitted "
             "MINUS the holding torque. Loading it would ADD gravity instead of "
             "removing it. Regenerate with fit_gravity.py.";
    }
    return false;
  }

  {
    const size_t bp = findKey(gj, "basis");
    const std::vector<std::string> want = {"cos q2", "sin q2", "cos q3", "sin q3"};
    const size_t a = gj.find('[', bp);
    const size_t b = gj.find(']', a);
    if (bp == std::string::npos || a == std::string::npos || b == std::string::npos) {
      if (err) *err = gravity_path + ": no \"basis\" array";
      return false;
    }
    std::string got = stripSpace(gj.substr(a + 1, b - a - 1));
    std::string expect;
    for (size_t i = 0; i < want.size(); ++i) {
      expect += "\"" + want[i] + "\"";
      if (i + 1 < want.size()) expect += ",";
    }
    if (got != stripSpace(expect)) {
      if (err) {
        *err = gravity_path + ": basis is [" + got + "], expected [" +
               stripSpace(expect) + "]. grav_coef is positional -- refusing to "
               "reinterpret coefficients as terms they do not belong to.";
      }
      return false;
    }
  }

  // Build into a scratch copy so a late failure leaves cfg untouched.
  DobConfig staged = *cfg;
  for (int j = 0; j < kNumActiveJoints; ++j) {
    const size_t jp_i = findKey(ij, jointKey(j));
    const double m = numberAfter(ij, findKey(ij, "M_n_kg_m2", jp_i));
    if (!(m > 0.0) || !std::isfinite(m)) {
      if (err) {
        *err = inertia_path + ": joint " + jointKey(j) +
               " has non-positive or non-finite M_n_kg_m2";
      }
      return false;
    }
    staged.joints[j].m_n_kg_m2 = m;

    // Friction compensation (Phase 2.6), OPTIONAL and best-effort -- unlike
    // M_n above, a missing or non-physical value here does not fail the load;
    // it just leaves this joint's friction terms at their zero default (no
    // compensation), exactly the pre-2.6 behaviour. Read from the SAME
    // inertia JSON (fit_inertia.py's b_visc/c_coulomb_nm), deliberately NOT
    // from friction_left.json's raw per-joint median -- see the long comment
    // at JointDobParams::fric_coulomb_nm in touch_dob.hpp for why: that file's
    // waist value is negative and non-physical as of the 2026-08-24 fit, and
    // it carries no viscous term at all.
    const double c_coulomb = numberAfter(ij, findKey(ij, "c_coulomb_nm", jp_i));
    const double b_visc = numberAfter(ij, findKey(ij, "b_visc", jp_i));
    if (std::isfinite(c_coulomb) && c_coulomb > 0.0) {
      staged.joints[j].fric_coulomb_nm = c_coulomb;
    }
    if (std::isfinite(b_visc) && b_visc > 0.0) {
      staged.joints[j].b_visc_nm_s_per_rad = b_visc;
    }

    const size_t jp_g = findKey(gj, jointKey(j));
    const std::vector<double> c = arrayAfter(gj, findKey(gj, "coef", jp_g));
    if (static_cast<int>(c.size()) != kNumGravityCoefs) {
      if (err) {
        *err = gravity_path + ": joint " + jointKey(j) + " has " +
               std::to_string(c.size()) + " coefficients, expected " +
               std::to_string(kNumGravityCoefs);
      }
      return false;
    }
    for (int k = 0; k < kNumGravityCoefs; ++k) {
      if (!std::isfinite(c[k])) {
        if (err) *err = gravity_path + ": joint " + jointKey(j) + " coefficient " +
                        std::to_string(k) + " is not finite";
        return false;
      }
      staged.joints[j].grav_coef[k] = c[k];
    }
  }

  *cfg = staged;
  return true;
}

}  // namespace dob_json
}  // namespace touch_haptics
