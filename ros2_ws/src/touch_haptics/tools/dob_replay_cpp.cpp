// dob_replay_cpp.cpp
//
// OFFLINE replay of the REAL observer over logged captures, to pick g_dis and
// threshold_nm from numbers the shipped code actually produces.
//
// NAMING: every other tool here is hdapi_*, because every other tool talks to
// the device. This one never opens a device, links no HD library, and cannot
// command force -- hence the different prefix. It is safe to run with the arm
// unplugged, or while another process owns it.
//
// ---------------------------------------------------------------------------
// WHY THIS EXISTS WHEN identification/replay_dob.py ALREADY DID THIS
// ---------------------------------------------------------------------------
// replay_dob.py re-implemented the observer in Python to explore the design
// space quickly. That was useful, but the numbers it produced came from a
// parallel implementation, not from JointDobBank. Two differences are known
// and deliberate, and both are large enough to matter:
//
//   1. DISCRETISATION. Python used a zero-order hold, alpha = 1-exp(-g*T).
//      touch_dob.hpp uses Tustin, and its own comment explains why: ZOH is
//      exact only for piecewise-constant inputs, but the input here contains
//      g*M_n*qdot, which ramps whenever the joint accelerates. The resulting
//      lag biases the estimate by about (g*M_n*T/2)*qddot -- measured there at
//      0.023 N*m, i.e. the same order as a plausible threshold_nm. So the two
//      implementations are EXPECTED to diverge, and the C++ is the correct one.
//
//   2. GRAVITY FRAME ALIGNMENT. Python evaluated G at the same sample as
//      tau_cmd. JointDobBank evaluates it at q_prev_, because on hardware
//      HD_LAST_JOINT_TORQUE reports the command issued one frame earlier.
//      This replay feeds the bank the same one-frame-lagged torque hardware
//      would, so the alignment is exercised rather than bypassed.
//
// If those deltas ever grow large enough to change WHICH g_dis looks best,
// that is not a rounding artefact -- it means one implementation is wrong, and
// the right response is to find out which, not to average them.
//
// ---------------------------------------------------------------------------
// THE TORQUE SCALE, AND WHY IT IS 1.0 HERE
// ---------------------------------------------------------------------------
// JointDobBank multiplies its tau_cmd_raw argument by
// last_joint_torque_scale_nm_per_unit internally, so callers pass RAW device
// units. On hardware that raw value is HD_LAST_JOINT_TORQUE.
//
// This replay cannot use the logged raw_lastjt_* columns: that scale has never
// been calibrated (hdapi_torque_calib has not been run), so those numbers have
// no known relationship to N*m. Instead the driving torque is reconstructed the
// same way the whole identification pipeline reconstructs it --
//
//     tau = J^T(q) @ F_cmd,   F_cmd in newtons, exactly known
//
// -- which is already in real N*m. The scale is therefore set to exactly 1.0:
// not a fudge, but the correct conversion factor from "units of N*m" to N*m.
// Setting it also satisfies torqueScaleCalibrated(), without which the bank
// correctly suppresses the whole estimate.
//
// BUILD (from this tools/ directory):
//   g++ -std=gnu++17 -I../include dob_replay_cpp.cpp -o dob_replay_cpp
//
// RUN:
//   ./dob_replay_cpp ~/dob_logs/dyn_Left.csv \
//       --inertia ~/dob_logs/results/inertia_left.json \
//       --gravity ~/dob_logs/results/gravity_left_minimal.json \
//       --python  ~/dob_logs/results/dob_tuning.json

#include "touch_haptics/touch_dob.hpp"
#include "touch_haptics/dob_json_loader.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <map>
#include <sstream>
#include <string>
#include <vector>

// Originally matched replay_dob.py's CANDIDATE_HZ exactly so the two reports
// could be diffed. EXTENDED DOWNWARD 2026-08-25 to cover the band the reference
// paper actually uses: it runs the same hardware at g_dis = {10,10,5} rad/s,
// i.e. 1.59 / 1.59 / 0.80 Hz -- entirely below the old sweep's lowest point.
//
// Without these rows the sub-5 Hz floor could only be extrapolated, and the
// extrapolation is exactly where it matters least reliably: the floor is a sum
// of a term scaling with g*M_n (velocity-noise amplification) and a roughly
// constant friction/model-error term, so a straight line fitted at 5-20 Hz
// over-predicts the reduction available below 5 Hz. threshold_nm is derived
// from these numbers, so guessing them would propagate into a live safety gate.
//
// The Python prototype is NOT being extended to match -- it uses a ZOH
// discretisation this tool deliberately does not, and the cross-check against
// it remains valid only on the overlapping rows.
static const double kCandidateHz[] = {0.8, 1.0, 1.59, 2.5, 5.0, 10.0, 20.0, 30.0, 50.0, 80.0};
static const int kNumCandidates = sizeof(kCandidateHz) / sizeof(kCandidateHz[0]);

// Matches replay_dob.py: captures shorter than this are fragments (an aborted
// run), and the trim discards the raised-cosine envelope ramp at each end.
static const int kMinSamples = 2000;
static const double kTrimFrac = 0.10;

static const char *kJointName[3] = {"waist", "shoulder", "elbow"};

// ---------------------------------------------------------------------------
// Minimal CSV / JSON readers. Deliberately not general-purpose: they handle
// exactly the two shapes this pipeline emits, and complain rather than guess.
// ---------------------------------------------------------------------------

struct Capture {
  int pose_id = 0;
  std::string axis;
  std::vector<double> t;
  std::vector<std::array<double, 3>> q;
  std::vector<std::array<double, 3>> cmd;
  int clipped = 0;

  // Median sample interval. Median, not mean: one scheduling hiccup would drag
  // a mean and silently bias every derivative taken from it. Same choice
  // common.py's Capture.dt makes.
  double dt() const
  {
    if (t.size() < 2) return std::nan("");
    std::vector<double> d;
    d.reserve(t.size() - 1);
    for (size_t i = 1; i < t.size(); ++i) d.push_back(t[i] - t[i - 1]);
    std::sort(d.begin(), d.end());
    return d[d.size() / 2];
  }
};

static std::vector<Capture> loadDynamic(const std::string &path)
{
  std::ifstream f(path);
  if (!f) { printf("FATAL: cannot open %s\n", path.c_str()); exit(1); }

  std::string line;
  if (!std::getline(f, line)) { printf("FATAL: empty log\n"); exit(1); }

  std::map<std::string, int> col;
  {
    std::stringstream ss(line);
    std::string h;
    int i = 0;
    while (std::getline(ss, h, ',')) col[h] = i++;
  }
  // Fail loudly on a layout change rather than reading garbage columns.
  const char *required[] = {"tool", "pose_id", "axis", "unix_time", "sample_idx",
                            "t_s", "cmd_1", "cmd_2", "cmd_3", "q_1", "q_2", "q_3"};
  for (const char *r : required) {
    if (!col.count(r)) {
      printf("FATAL: log is missing column '%s'. Layout has changed; update this "
             "tool and identification/common.py together.\n", r);
      exit(1);
    }
  }

  // Grouped exactly as common.py::load_dynamic does: one capture per
  // (pose_id, axis, unix_time).
  std::map<std::string, Capture> groups;
  std::vector<std::string> order;
  std::vector<std::string> fld;

  while (std::getline(f, line)) {
    if (line.empty()) continue;
    fld.clear();
    std::stringstream ss(line);
    std::string v;
    while (std::getline(ss, v, ',')) fld.push_back(v);
    if ((int)fld.size() <= col["q_3"]) continue;
    if (fld[col["tool"]] != "dynamic_excitation") continue;

    const std::string key =
      fld[col["pose_id"]] + "|" + fld[col["axis"]] + "|" + fld[col["unix_time"]];
    if (!groups.count(key)) {
      Capture c;
      c.pose_id = std::atoi(fld[col["pose_id"]].c_str());
      c.axis = fld[col["axis"]];
      if (col.count("clipped") && (int)fld.size() > col["clipped"]) {
        c.clipped = (int)std::atof(fld[col["clipped"]].c_str());
      }
      groups[key] = c;
      order.push_back(key);
    }
    Capture &c = groups[key];
    c.t.push_back(std::atof(fld[col["t_s"]].c_str()));
    c.q.push_back({std::atof(fld[col["q_1"]].c_str()),
                   std::atof(fld[col["q_2"]].c_str()),
                   std::atof(fld[col["q_3"]].c_str())});
    c.cmd.push_back({std::atof(fld[col["cmd_1"]].c_str()),
                     std::atof(fld[col["cmd_2"]].c_str()),
                     std::atof(fld[col["cmd_3"]].c_str())});
  }

  std::vector<Capture> out;
  for (const std::string &k : order) out.push_back(groups[k]);
  return out;
}

// Pull one number, or an N-element array, out of the identification JSON.
// Enough structure-awareness for these files, no more.




// numpy.percentile(a, 99) with the default linear interpolation, reproduced so
// the C++ and Python p99 columns are comparable rather than merely similar.
static double percentileLinear(std::vector<double> &v, double pct)
{
  if (v.empty()) return std::nan("");
  std::sort(v.begin(), v.end());
  const double idx = (pct / 100.0) * (double)(v.size() - 1);
  const size_t lo = (size_t)std::floor(idx);
  const size_t hi = std::min(lo + 1, v.size() - 1);
  const double frac = idx - (double)lo;
  return v[lo] + frac * (v[hi] - v[lo]);
}

int main(int argc, char **argv)
{
  std::string logPath, inertiaPath, gravityPath, pythonPath;
  for (int i = 1; i < argc; ++i) {
    if (!std::strcmp(argv[i], "--inertia") && i + 1 < argc) inertiaPath = argv[++i];
    else if (!std::strcmp(argv[i], "--gravity") && i + 1 < argc) gravityPath = argv[++i];
    else if (!std::strcmp(argv[i], "--python") && i + 1 < argc) pythonPath = argv[++i];
    else if (logPath.empty()) logPath = argv[i];
  }
  if (logPath.empty() || inertiaPath.empty() || gravityPath.empty()) {
    printf("usage: %s <dyn.csv> --inertia <inertia.json> --gravity <gravity.json> "
           "[--python <dob_tuning.json>]\n", argv[0]);
    return 1;
  }

  printf("=== Offline replay of the REAL JointDobBank ===\n");
  printf("log     : %s\n", logPath.c_str());
  printf("inertia : %s\n", inertiaPath.c_str());
  printf("gravity : %s\n\n", gravityPath.c_str());

  // ---- parameters, treated as fixed inputs ------------------------------
  // Loaded through the SHARED loader that src/touch_slave.cpp also uses. If the
  // node and this tool parsed the JSON differently, the live numbers would not
  // be comparable with the offline prediction -- which is the only reason to
  // run both. One loader, one set of refusals.
  touch_haptics::DobConfig cfg{};
  {
    std::string err;
    if (!touch_haptics::dob_json::loadIdentifiedParams(inertiaPath, gravityPath,
                                                       &cfg, &err)) {
      printf("FATAL: %s\n", err.c_str());
      return 1;
    }
  }
  for (int j = 0; j < 3; ++j) {
    // Not swept here, but valid() requires them before configure() will accept
    // the params. They do not enter tau_dis_hat: the gate affects
    // contactSuspected()/correct(), and this tool reports the raw estimate.
    cfg.joints[j].threshold_nm = 1.0e6;    // high, so the gate stays shut
    cfg.joints[j].max_torque_nm = 1.0;
    cfg.joints[j].gate_release_ratio = 1.0;
  }
  // See the header note: the driving torque is already in N*m.
  cfg.last_joint_torque_scale_nm_per_unit = 1.0;

  printf("M_n      : ");
  for (int j = 0; j < 3; ++j) printf("%s=%.4e  ", kJointName[j], cfg.joints[j].m_n_kg_m2);
  printf("\ngrav_coef: loaded, basis [cos q2, sin q2, cos q3, sin q3] verified\n");
  printf("scale    : 1.0 (tau reconstructed as J^T(q)*F_cmd, already N*m)\n\n");

  // ---- captures ---------------------------------------------------------
  std::vector<Capture> all = loadDynamic(logPath);
  std::vector<const Capture *> caps;
  int n_short = 0, n_clipped = 0;
  for (const Capture &c : all) {
    if ((int)c.t.size() <= kMinSamples) { ++n_short; continue; }
    if (c.clipped) { ++n_clipped; continue; }
    caps.push_back(&c);
  }
  printf("captures : %zu total, %zu usable (%d excluded as fragments <= %d samples",
         all.size(), caps.size(), n_short, kMinSamples);
  if (n_clipped) printf(", %d excluded as clipped", n_clipped);
  printf(")\n");

  size_t n_samples = 0;
  for (const Capture *c : caps) {
    const int k = (int)((double)c->t.size() * kTrimFrac);
    n_samples += c->t.size() - 2 * (size_t)k;
  }
  printf("samples  : %zu per joint after %.0f%% trim at each end\n\n",
         n_samples, kTrimFrac * 100.0);

  printf("NOTE: only the gravity-COMPENSATED case is swept here. Since\n"
         "JointDobParams::valid() refuses to configure without loaded gravity\n"
         "coefficients, the no-gravity configuration is no longer reachable\n"
         "through the real bank, and fabricating zero coefficients to force it\n"
         "would defeat the guard on purpose. For that comparison see the Python\n"
         "run's \"as implemented (NO gravity model)\" block.\n\n");

  // ---- sweep ------------------------------------------------------------
  double p99[kNumCandidates][3];
  double pmax[kNumCandidates][3];

  for (int ci = 0; ci < kNumCandidates; ++ci) {
    const double g_dis = 2.0 * M_PI * kCandidateHz[ci];
    std::vector<double> acc[3];
    for (int j = 0; j < 3; ++j) acc[j].reserve(n_samples);

    for (const Capture *c : caps) {
      touch_haptics::DobConfig cc = cfg;
      for (int j = 0; j < 3; ++j) cc.joints[j].g_dis_rad_s = g_dis;

      touch_haptics::JointDobBank bank;
      if (!bank.configure(cc, c->dt())) {
        printf("FATAL: bank refused to configure. Reason (joint 0): %s\n",
               cc.joints[0].invalidReason() ? cc.joints[0].invalidReason() : "unknown");
        return 1;
      }

      const int n = (int)c->t.size();
      const int k = (int)((double)n * kTrimFrac);

      for (int i = 0; i < n; ++i) {
        // Emulate the hardware pairing: at frame i the device reports the
        // torque it was commanded at frame i-1, computed for the pose held
        // then. The bank then evaluates gravity at q_prev_ = q[i-1], which is
        // the same frame -- so the alignment inside the bank is exercised,
        // not sidestepped.
        touch_haptics::Vec3 tau_lagged{0.0, 0.0, 0.0};
        if (i > 0) {
          const touch_haptics::Mat3 J = touch_haptics::jacobian(c->q[i - 1]);
          tau_lagged = touch_haptics::jacobianTransposeTimes(J, c->cmd[i - 1]);
        }
        bank.update(c->q[i], tau_lagged);

        if (i >= k && i < n - k) {
          const auto est = bank.tauDisHatNm();
          for (int j = 0; j < 3; ++j) acc[j].push_back(std::fabs(est[j]));
        }
      }
    }

    for (int j = 0; j < 3; ++j) {
      pmax[ci][j] = acc[j].empty() ? std::nan("")
                                   : *std::max_element(acc[j].begin(), acc[j].end());
      p99[ci][j] = percentileLinear(acc[j], 99.0);
    }
  }

  // ---- report, laid out like replay_dob.py's -----------------------------
  printf("=== C++ JointDobBank, gravity compensated ===\n");
  printf("%8s %6s  ", "g_dis", "Hz");
  for (int j = 0; j < 3; ++j) printf("%13s", (std::string(kJointName[j]).substr(0, 5) + " p99").c_str());
  for (int j = 0; j < 3; ++j) printf("%13s", (std::string(kJointName[j]).substr(0, 5) + " max").c_str());
  printf("\n");
  for (int ci = 0; ci < kNumCandidates; ++ci) {
    printf("%8.1f %6.1f  ", 2.0 * M_PI * kCandidateHz[ci], kCandidateHz[ci]);
    for (int j = 0; j < 3; ++j) printf("%13.5f", p99[ci][j]);
    for (int j = 0; j < 3; ++j) printf("%13.5f", pmax[ci][j]);
    printf("\n");
  }

  // ---- cross-check against the Python prototype --------------------------
  if (!pythonPath.empty()) {
    std::string perr;
    const std::string pj = touch_haptics::dob_json::slurp(pythonPath, &perr);
    size_t wg = pj.find("\"with G(q) subtracted\"");
    if (wg == std::string::npos) {
      printf("\nWARNING: '%s' has no \"with G(q) subtracted\" block; skipping cross-check.\n",
             pythonPath.c_str());
    } else {
      printf("\n=== Cross-check vs identification/replay_dob.py (p99, N*m) ===\n");
      printf("Deltas are EXPECTED and are not noise. Two deliberate differences:\n");
      printf("  * Python used ZOH, alpha = 1-exp(-g*T); the C++ uses Tustin. The\n");
      printf("    header's own estimate of ZOH's ramp lag is (g*M_n*T/2)*qddot,\n");
      printf("    which grows with g -- so expect the gap to widen with cutoff.\n");
      printf("  * Python evaluated G at the same sample as tau; the bank uses the\n");
      printf("    previous frame, matching HD_LAST_JOINT_TORQUE's one-frame lag.\n\n");
      printf("%6s  %-10s %11s %11s %10s %9s\n", "Hz", "joint", "python", "cpp", "delta", "rel");
      double worst_rel = 0.0;
      int worst_hz = 0;
      for (int ci = 0; ci < kNumCandidates; ++ci) {
        char key[32];
        std::snprintf(key, sizeof(key), "\"%g\"", kCandidateHz[ci]);
        size_t kp = pj.find(key, wg);
        if (kp == std::string::npos) continue;
        size_t pp = touch_haptics::dob_json::findKey(pj, "p99_nm", kp);
        for (int j = 0; j < 3; ++j) {
          size_t jp = touch_haptics::dob_json::findKey(pj, kJointName[j], pp);
          const double py = touch_haptics::dob_json::numberAfter(pj, jp);
          const double cp = p99[ci][j];
          const double d = cp - py;
          const double rel = (std::fabs(py) > 1e-12) ? std::fabs(d) / std::fabs(py) : 0.0;
          if (rel > worst_rel) { worst_rel = rel; worst_hz = (int)kCandidateHz[ci]; }
          printf("%6.1f  %-10s %11.5f %11.5f %+10.5f %8.1f%%\n",
                 kCandidateHz[ci], kJointName[j], py, cp, d, 100.0 * rel);
        }
      }
      printf("\nlargest relative delta: %.1f%% (at %d Hz)\n", 100.0 * worst_rel, worst_hz);

      // Does the disagreement change the ANSWER? That is the question that
      // matters -- a large delta that leaves the ranking intact is a curiosity,
      // one that reorders the candidates is a reason to stop.
      int best_py[3], best_cpp[3];
      for (int j = 0; j < 3; ++j) {
        double bp = 1e300, bc = 1e300;
        best_py[j] = best_cpp[j] = -1;
        for (int ci = 0; ci < kNumCandidates; ++ci) {
          char key[32];
          std::snprintf(key, sizeof(key), "\"%g\"", kCandidateHz[ci]);
          size_t kp = pj.find(key, wg);
          if (kp == std::string::npos) continue;
          size_t pp = touch_haptics::dob_json::findKey(pj, "p99_nm", kp);
          size_t jp = touch_haptics::dob_json::findKey(pj, kJointName[j], pp);
          const double py = touch_haptics::dob_json::numberAfter(pj, jp);
          if (py < bp) { bp = py; best_py[j] = ci; }
          if (p99[ci][j] < bc) { bc = p99[ci][j]; best_cpp[j] = ci; }
        }
      }
      bool ranking_same = true;
      printf("\nlowest-floor candidate per joint:\n");
      for (int j = 0; j < 3; ++j) {
        const bool same = best_py[j] == best_cpp[j];
        ranking_same = ranking_same && same;
        printf("  %-10s python %2.0f Hz   cpp %2.0f Hz   %s\n", kJointName[j],
               best_py[j] >= 0 ? kCandidateHz[best_py[j]] : 0.0,
               best_cpp[j] >= 0 ? kCandidateHz[best_cpp[j]] : 0.0,
               same ? "agree" : "DISAGREE");
      }
      if (!ranking_same) {
        printf("\n  *** The two implementations disagree on which g_dis is best. ***\n");
        printf("  Do not pick values from this table. One of the two is wrong, and\n");
        printf("  which one should be established before committing to a cutoff.\n");
      } else {
        printf("\n  Ranking agrees, so the deltas are a magnitude difference, not a\n");
        printf("  different answer. The C++ column is the one to use: it is what the\n");
        printf("  shipped observer computes.\n");
      }
    }
  }

  // ---- candidate thresholds ---------------------------------------------
  printf("\n=== Candidate threshold_nm = 2 x p99  (replay_dob.py's convention) ===\n");
  printf("%9s  ", "g_dis Hz");
  for (int j = 0; j < 3; ++j) printf("%12s", kJointName[j]);
  printf("\n");
  for (int ci = 0; ci < kNumCandidates; ++ci) {
    printf("%9.1f  ", kCandidateHz[ci]);
    for (int j = 0; j < 3; ++j) printf("%12.5f", 2.0 * p99[ci][j]);
    printf("\n");
  }

  // ---- floor vs torque authority ----------------------------------------
  printf("\n=== p99 floor as %% of per-joint torque authority ===\n");
  printf("(limits from touch_dob.hpp::derivePerJointTorqueLimits, used directly)\n");
  for (double f_max : {0.6, 1.5}) {
    const touch_haptics::Vec3 lim = touch_haptics::derivePerJointTorqueLimits(f_max);
    printf("\n  f_max = %.2f N   limits: ", f_max);
    for (int j = 0; j < 3; ++j) printf("%s=%.4f  ", kJointName[j], lim[j]);
    printf("\n  %9s  ", "g_dis Hz");
    for (int j = 0; j < 3; ++j) printf("%12s", kJointName[j]);
    printf("\n");
    for (int ci = 0; ci < kNumCandidates; ++ci) {
      printf("  %9.1f  ", kCandidateHz[ci]);
      for (int j = 0; j < 3; ++j) printf("%11.0f%%", 100.0 * p99[ci][j] / lim[j]);
      printf("\n");
    }
  }
  printf("\n  A threshold must sit ABOVE the floor. Where the floor already\n"
         "  consumes most of a joint's authority, little band remains in which\n"
         "  contact can register before the clamp bites.\n");

  printf("\n=== Done. ===\n");
  printf("NOTHING was written to touch_dob.hpp, to any config, or to any header.\n"
         "g_dis_rad_s, threshold_nm and max_torque_nm remain unset. Choosing them\n"
         "from this report is a separate, explicit step for the operator.\n");
  return 0;
}
