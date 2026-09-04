// hd_sim_api.cpp
//
// A drop-in replacement for libHD that drives a simulated arm instead of a
// device. Link a node against `hd_sim` instead of `HD` and it runs, unmodified,
// with no hardware attached.
//
// ---------------------------------------------------------------------------
// WHY THIS EXISTS
// ---------------------------------------------------------------------------
// Until now nothing in this repo could be exercised without both arms plugged
// in -- CMake hard-failed without OpenHaptics, and there was no stub anywhere.
// That made every change to the control path a hardware-time change, and
// hardware time is the scarcest resource on this project.
//
// The node under test is byte-identical to the one that runs on the arms. Only
// the library beneath it changes. That is the property that makes results here
// worth anything: if the sim required editing touch_slave.cpp, it would be
// testing a different program.
//
// ---------------------------------------------------------------------------
// SEMANTICS THAT MUST BE EXACTLY RIGHT
// ---------------------------------------------------------------------------
// Two of these are load-bearing for the observer and are otherwise untestable:
//
//  1. HD_LAST_JOINT_TORQUE / HD_LAST_FORCE report the PREVIOUS frame's command.
//     HD_CURRENT_* report this frame's. JointDobBank's q_prev_/qdot_prev_
//     pairing exists solely because of that one-frame lag. Reproducing it
//     faithfully is what lets the frame-alignment unit test mean something;
//     getting it wrong would either mask a real bug or manufacture a fake one.
//
//  2. HD_LAST_JOINT_TORQUE is in RAW device units, not N*m. The real scale has
//     never been calibrated. Reproducing a raw scale here is what allows
//     hdapi_torque_calib to be dry-run against the sim -- validating that
//     procedure, and its newly-added abort path, before it is ever pointed at
//     a real arm commanding real force.
//
// ---------------------------------------------------------------------------
// HEADERS
// ---------------------------------------------------------------------------
// This file includes <HD/hd.h>. CMake points that at the REAL OpenHaptics
// headers when they are installed, so enum values, struct layouts and calling
// conventions are guaranteed identical to hardware. The vendored fallback in
// sim/include is only for machines without OpenHaptics; the real headers are
// always preferred and are never copied into this repo (they are proprietary).

#include <HD/hd.h>

#include "arm_model.hpp"
#include "environment.hpp"
#include "touch_haptics/dob_json_loader.hpp"

#include <atomic>
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <map>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

namespace
{

using touch_sim::ArmModel;
using touch_sim::ArmParams;
using touch_sim::Environment;
using touch_sim::EnvParams;
using touch_sim::Hand;
using touch_sim::HandParams;
using touch_sim::Vec3;

struct ScheduledCallback
{
  HDSchedulerCallback fn = nullptr;
  void *data = nullptr;
  HDSchedulerHandle handle = 0;
  bool async = false;
  bool done = false;
};

// One simulated device.
struct SimDevice
{
  HHD handle = 0;
  std::string name;

  ArmModel arm;
  Environment env;
  Hand hand;

  // Commanded force this frame (Cartesian, device frame) and commanded joint
  // torque, whichever the node used. Both are consumed by the servo tick.
  Vec3 cmd_force_n{0, 0, 0};
  Vec3 cmd_joint_nm{0, 0, 0};
  bool joint_cmd_this_frame = false;

  // Post-pipeline values, reported by HD_LAST_*.
  Vec3 last_force_n{0, 0, 0};
  Vec3 last_joint_nm{0, 0, 0};

  bool force_output_enabled = false;
  std::map<HDenum, bool> caps;

  double t_s = 0.0;
  unsigned long frames = 0;

  std::vector<ScheduledCallback> callbacks;
  std::thread thread;
  std::atomic<bool> running{false};
  std::mutex mtx;

  FILE *truth = nullptr;
};

std::map<HHD, SimDevice *> g_devices;
std::mutex g_devices_mtx;
HHD g_next_handle = 1;

// HDAPI's "current device" is thread-local: the scheduler sets it before
// dispatching, and hdMakeCurrentDevice sets it on the calling thread. Modelling
// it as thread_local is what lets two devices run in one process without their
// callbacks stepping on each other.
thread_local SimDevice *g_current = nullptr;

HDErrorInfo g_no_error{HD_SUCCESS, 0, 0};

double envDouble(const char *key, double fallback)
{
  const char *v = std::getenv(key);
  return v ? std::atof(v) : fallback;
}

const char *envStr(const char *key, const char *fallback)
{
  const char *v = std::getenv(key);
  return v ? v : fallback;
}

// Per-device configuration. Deliberately environment-variable driven rather
// than a config-file format: this is a test fixture, and a scenario script that
// sets TOUCH_SIM_ENV=wall is easier to read, diff and reproduce than a JSON
// file whose provenance nobody can reconstruct six months later.
void configureDevice(SimDevice &d)
{
  ArmParams ap;
  ap.update_rate_hz = static_cast<int>(envDouble("TOUCH_SIM_RATE_HZ", 1000.0));
  ap.inertia_mismatch = envDouble("TOUCH_SIM_INERTIA_MISMATCH", 1.08);
  ap.gravity_mismatch = envDouble("TOUCH_SIM_GRAVITY_MISMATCH", 1.06);
  ap.base_tilt_rad = envDouble("TOUCH_SIM_BASE_TILT_RAD", 0.0785);
  ap.encoder_quantum_rad = envDouble("TOUCH_SIM_ENCODER_QUANTUM", 4.07e-4);
  ap.raw_units_per_nm = envDouble("TOUCH_SIM_RAW_PER_NM", 1000.0);

  // The right arm is a different physical unit. Giving it identical parameters
  // would quietly encode the "arms are symmetric" assumption that the
  // identification work explicitly refused to make.
  const bool is_right = (d.name.find("Default") != std::string::npos) ||
                        (d.name.find("Right") != std::string::npos);
  if (is_right) {
    ap.inertia_mismatch *= 1.05;
    ap.gravity_mismatch *= 0.97;
    ap.base_tilt_rad *= 0.6;
    ap.base_tilt_phase_rad = -1.1;
    for (int j = 0; j < 3; ++j) {
      ap.fric_coulomb[j] *= 1.15;
      ap.fric_static[j] *= 1.15;
    }
  }
  // ---------------------------------------------------------------------
  // TWO PARAMETER MODES -- read this before setting SIM_USE_IDENTIFIED_PARAMS
  // ---------------------------------------------------------------------
  // Default (above): inertia_mismatch/gravity_mismatch/base_tilt_rad are
  // DELIBERATELY WRONG, by design -- see the big comment at the top of
  // arm_model.hpp. Use this mode to stress-test a controller or observer's
  // robustness against general model error. A real-vs-sim residual here
  // mixes "genuine unmodeled dynamics" with "however wrong we chose to make
  // the sim today" in unknown proportions, and is NOT a friction estimate.
  //
  // SIM_USE_IDENTIFIED_PARAMS=true (below): loads M_n and G(q) from the same
  // identified JSON files and the same parsing/validation code the live DOB
  // uses (dob_json_loader.hpp), and sets inertia_mismatch=gravity_mismatch=
  // 1.0 -- i.e. "best available real model", not "known wrong on purpose".
  // With this mode, a real-vs-sim divergence is a genuine second estimate of
  // unmodeled dynamics (friction, unfitted coupling, ...), computed a
  // completely different way from the DOB's own tau_dis_hat -- useful as a
  // cross-check between the two once both exist, NOT a replacement for
  // either. base_tilt_rad is left ON even in this mode: it represents the
  // rig's actual physical base tilt, which the identified G(q) basis
  // (cos q2, sin q2, cos q3, sin q3 only) structurally cannot represent
  // either -- the same blind spot touch_dob.hpp documents for the live
  // observer, so keeping it here is what makes the two comparable rather
  // than a discrepancy between them. Friction stays at this file's existing
  // defaults either way -- those were already loosely set from
  // friction_left.json (see the ArmParams field comments) and swapping them
  // live is not part of what this mode changes.
  if (envStr("SIM_USE_IDENTIFIED_PARAMS", "false") == std::string("true")) {
    const std::string results_dir =
      envStr("SIM_IDENTIFIED_PARAMS_DIR", "");
    std::string base = results_dir;
    if (base.empty()) {
      const char *home = std::getenv("HOME");
      base = (home ? std::string(home) : std::string(".")) + "/dob_logs/results";
    }
    const std::string inertia_path = base + "/inertia_left.json";
    const std::string gravity_path = base + "/gravity_left_minimal.json";

    touch_haptics::DobConfig staged{};
    std::string err;
    if (touch_haptics::dob_json::loadIdentifiedParams(inertia_path, gravity_path,
                                                       &staged, &err)) {
      for (int j = 0; j < 3; ++j) {
        ap.m_n[j] = staged.joints[j].m_n_kg_m2;
        for (int k = 0; k < 4; ++k) ap.grav_coef[j][k] = staged.joints[j].grav_coef[k];
      }
      ap.inertia_mismatch = 1.0;
      ap.gravity_mismatch = 1.0;
      std::fprintf(stderr,
                   "[hd_sim] SIM_USE_IDENTIFIED_PARAMS: loaded %s / %s "
                   "(only the LEFT arm has been identified as of writing -- "
                   "using it on the right device applies left-arm M_n/G(q) "
                   "there, which is a known approximation, not a right-arm fit)\n",
                   inertia_path.c_str(), gravity_path.c_str());
    } else {
      std::fprintf(stderr,
                   "[hd_sim][error] SIM_USE_IDENTIFIED_PARAMS=true but failed to load "
                   "identified params: %s -- falling back to the deliberately-"
                   "mismatched default parameters instead\n", err.c_str());
    }
  }

  // Seed the sim's initial joint state from a real device's actual starting
  // Cartesian position, if the launcher provided one -- see
  // ArmParams::has_init_pose_m in arm_model.hpp for why this exists.
  // Activated by TOUCH_SIM_INIT_POS_X being set at all (Y/Z default to 0 if
  // the caller only set X, but the launcher scripts always set all three
  // together or none).
  if (std::getenv("TOUCH_SIM_INIT_POS_X") != nullptr) {
    ap.has_init_pose_m = true;
    ap.init_pose_m = {
      envDouble("TOUCH_SIM_INIT_POS_X", 0.0),
      envDouble("TOUCH_SIM_INIT_POS_Y", 0.0),
      envDouble("TOUCH_SIM_INIT_POS_Z", 0.0),
    };
  }

  d.arm = ArmModel(ap);

  EnvParams ep;
  const std::string env = envStr("TOUCH_SIM_ENV", "free");
  if (env == "wall" || env == "plane") {
    ep.kind = EnvParams::Kind::kPlane;
    ep.k_contact = envDouble("TOUCH_SIM_WALL_K", 2000.0);
    ep.b_contact = envDouble("TOUCH_SIM_WALL_B", 8.0);
    ep.plane_point = {0.0, envDouble("TOUCH_SIM_WALL_Y", -0.05), 0.0};
  } else if (env == "box") {
    ep.kind = EnvParams::Kind::kBox;
  }
  // The environment belongs to the SLAVE. A wall on the master would be an
  // obstacle in the operator's hand, which is not what any of these scenarios
  // mean.
  if (!is_right) ep.kind = EnvParams::Kind::kFree;
  d.env = Environment(ep);

  HandParams hp = HandParams::preset(envStr("TOUCH_SIM_HAND", "none"));
  const std::string motion = envStr("TOUCH_SIM_HAND_MOTION", "none");
  if (motion == "sine") hp.motion = HandParams::Motion::kSine;
  else if (motion == "chirp") hp.motion = HandParams::Motion::kChirp;
  else if (motion == "step") hp.motion = HandParams::Motion::kStep;
  else if (motion == "hold") hp.motion = HandParams::Motion::kHold;
  hp.f0_hz = envDouble("TOUCH_SIM_HAND_F0", 0.2);
  hp.f1_hz = envDouble("TOUCH_SIM_HAND_F1", 20.0);
  hp.duration_s = envDouble("TOUCH_SIM_HAND_DUR", 30.0);
  hp.amplitude_m = envDouble("TOUCH_SIM_HAND_AMP", 0.03);
  // Likewise the hand belongs to the MASTER.
  if (is_right) { hp.k_hand = 0.0; hp.b_hand = 0.0; hp.motion = HandParams::Motion::kNone; }
  d.hand = Hand(hp);
  d.hand.setOrigin(d.arm.tipPositionM());

  // Ground truth. The sim knows F_env and F_hand exactly; the node can only
  // estimate them. Logging both is what turns "does the observer work" from a
  // judgement call into a subtraction.
  const char *tp = std::getenv("TOUCH_SIM_TRUTH_CSV");
  if (tp && *tp) {
    std::string path = std::string(tp);
    if (path.find("%s") != std::string::npos) {
      char buf[512];
      std::snprintf(buf, sizeof(buf), path.c_str(), d.name.c_str());
      path = buf;
    } else {
      path += "." + d.name + ".csv";
    }
    for (char &c : path) if (c == ' ') c = '_';
    d.truth = std::fopen(path.c_str(), "w");
    if (d.truth) {
      std::fprintf(d.truth,
        "t_s,q1,q2,q3,qdot1,qdot2,qdot3,"
        "tau_cmd1,tau_cmd2,tau_cmd3,"
        "f_ext_true_x,f_ext_true_y,f_ext_true_z,"
        "tau_ext_true1,tau_ext_true2,tau_ext_true3,"
        "g_true1,g_true2,g_true3,fric_true1,fric_true2,fric_true3,"
        "tip_x,tip_y,tip_z\n");
    }
  }

  d.caps[HD_FORCE_OUTPUT] = false;
  d.caps[HD_MAX_FORCE_CLAMPING] = true;
  d.caps[HD_FORCE_RAMPING] = true;
  d.caps[HD_SOFTWARE_FORCE_LIMIT] = true;
  d.caps[HD_ONE_FRAME_LIMIT] = true;
}

void writeTruth(SimDevice &d, const Vec3 &tau_cmd, const Vec3 &f_ext)
{
  if (!d.truth) return;
  const Vec3 &q = d.arm.qTrue();
  const Vec3 &qd = d.arm.qdotTrue();
  const touch_haptics::Mat3 J = touch_haptics::jacobian(q);
  const Vec3 tau_ext = touch_haptics::jacobianTransposeTimes(J, f_ext);
  const Vec3 g = d.arm.gravityTrueNm();
  const Vec3 fr = d.arm.frictionTrueNm();
  const Vec3 tip = d.arm.tipPositionM();
  std::fprintf(d.truth,
    "%.6f,%.9f,%.9f,%.9f,%.9f,%.9f,%.9f,"
    "%.9f,%.9f,%.9f,%.9f,%.9f,%.9f,%.9f,%.9f,%.9f,"
    "%.9f,%.9f,%.9f,%.9f,%.9f,%.9f,%.6f,%.6f,%.6f\n",
    d.t_s, q[0], q[1], q[2], qd[0], qd[1], qd[2],
    tau_cmd[0], tau_cmd[1], tau_cmd[2],
    f_ext[0], f_ext[1], f_ext[2],
    tau_ext[0], tau_ext[1], tau_ext[2],
    g[0], g[1], g[2], fr[0], fr[1], fr[2],
    tip[0], tip[1], tip[2]);
}

// One servo tick: run the node's callbacks, then advance physics.
void servoTick(SimDevice &d)
{
  d.joint_cmd_this_frame = false;
  d.cmd_force_n = {0, 0, 0};
  d.cmd_joint_nm = {0, 0, 0};

  g_current = &d;
  {
    std::lock_guard<std::mutex> lk(d.mtx);
    for (auto &cb : d.callbacks) {
      if (cb.done || !cb.fn) continue;
      const HDCallbackCode rc = cb.fn(cb.data);
      if (rc == HD_CALLBACK_DONE) cb.done = true;
    }
  }

  // Which command path did the node use this frame? touch_slave writes
  // Cartesian force normally and joint torque only when its gate opens, so
  // whichever arrived is the live one. Joint wins if both appear, matching the
  // node's own precedence.
  Vec3 tau_cmd{0, 0, 0};
  if (!d.force_output_enabled) {
    tau_cmd = {0, 0, 0};
  } else if (d.joint_cmd_this_frame) {
    tau_cmd = d.cmd_joint_nm;
  } else {
    const touch_haptics::Mat3 J = touch_haptics::jacobian(d.arm.qReported());
    tau_cmd = touch_haptics::jacobianTransposeTimes(J, d.cmd_force_n);
  }

  // External force: environment on the slave, hand on the master.
  const Vec3 tip = d.arm.tipPositionM();
  const Vec3 vel = d.arm.tipVelocityMps();
  touch_sim::ExternalWrench ext = d.env.force(tip, vel);
  const touch_sim::ExternalWrench h = d.hand.force(d.t_s, tip, vel);
  for (int i = 0; i < 3; ++i) ext.f_tip_n[i] += h.f_tip_n[i];

  const double dt = 1.0 / static_cast<double>(d.arm.params().update_rate_hz);
  d.arm.step(tau_cmd, ext, dt);

  // HD_LAST_* now reflect the command that was just applied -- which, from the
  // NEXT frame's point of view, is the previous frame's. See the header note.
  d.last_force_n = d.cmd_force_n;
  d.last_joint_nm = d.arm.tauPrevNm();

  writeTruth(d, tau_cmd, ext.f_tip_n);

  d.t_s += dt;
  d.frames++;
}

void schedulerThread(SimDevice *d)
{
  using clock = std::chrono::steady_clock;
  const auto period = std::chrono::nanoseconds(
    static_cast<long long>(1e9 / d->arm.params().update_rate_hz));
  auto next = clock::now();
  while (d->running.load()) {
    servoTick(*d);
    next += period;
    std::this_thread::sleep_until(next);
  }
}

SimDevice *currentOr(HHD h)
{
  if (g_current) return g_current;
  std::lock_guard<std::mutex> lk(g_devices_mtx);
  auto it = g_devices.find(h);
  return (it == g_devices.end()) ? nullptr : it->second;
}

}  // namespace

// ===========================================================================
// HDAPI surface
// ===========================================================================
extern "C" {

HHD hdInitDevice(const char *name)
{
  auto *d = new SimDevice();
  d->name = name ? name : "Default Device";
  {
    std::lock_guard<std::mutex> lk(g_devices_mtx);
    d->handle = g_next_handle++;
    g_devices[d->handle] = d;
  }
  configureDevice(*d);
  g_current = d;
  std::fprintf(stderr, "[hd_sim] initialised simulated device '%s' (handle %d) "
                       "at %d Hz\n",
               d->name.c_str(), d->handle, d->arm.params().update_rate_hz);
  const bool identified = d->arm.params().inertia_mismatch == 1.0 &&
                          d->arm.params().gravity_mismatch == 1.0;
  std::fprintf(stderr, "[hd_sim] NOT HARDWARE. %s\n",
               identified
                 ? "SIM_USE_IDENTIFIED_PARAMS mode: M_n/G(q) loaded from "
                   "identified JSON, not deliberately mismatched."
                 : "Model parameters deliberately differ from the identified ones.");
  return d->handle;
}

void hdDisableDevice(HHD h)
{
  SimDevice *d = nullptr;
  {
    std::lock_guard<std::mutex> lk(g_devices_mtx);
    auto it = g_devices.find(h);
    if (it == g_devices.end()) return;
    d = it->second;
    g_devices.erase(it);
  }
  d->running = false;
  if (d->thread.joinable()) d->thread.join();
  if (d->truth) { std::fclose(d->truth); d->truth = nullptr; }
  if (g_current == d) g_current = nullptr;
  delete d;
}

void hdMakeCurrentDevice(HHD h)
{
  std::lock_guard<std::mutex> lk(g_devices_mtx);
  auto it = g_devices.find(h);
  if (it != g_devices.end()) g_current = it->second;
}

HDErrorInfo hdGetError(void) { return g_no_error; }
const char *hdGetErrorString(HDerror) { return "HD_SUCCESS (sim)"; }

void hdBeginFrame(HHD h) { (void)currentOr(h); }
void hdEndFrame(HHD) {}

void hdEnable(HDenum cap)
{
  if (!g_current) return;
  g_current->caps[cap] = true;
  if (cap == HD_FORCE_OUTPUT) g_current->force_output_enabled = true;
}

void hdDisable(HDenum cap)
{
  if (!g_current) return;
  g_current->caps[cap] = false;
  if (cap == HD_FORCE_OUTPUT) g_current->force_output_enabled = false;
}

HDboolean hdIsEnabled(HDenum cap)
{
  if (!g_current) return HD_FALSE;
  auto it = g_current->caps.find(cap);
  return (it != g_current->caps.end() && it->second) ? HD_TRUE : HD_FALSE;
}

HDSchedulerHandle hdScheduleAsynchronous(HDSchedulerCallback fn, void *data, HDushort)
{
  if (!g_current) return 0;
  std::lock_guard<std::mutex> lk(g_current->mtx);
  ScheduledCallback cb;
  cb.fn = fn;
  cb.data = data;
  cb.async = true;
  cb.handle = static_cast<HDSchedulerHandle>(g_current->callbacks.size() + 1);
  g_current->callbacks.push_back(cb);
  return cb.handle;
}

void hdScheduleSynchronous(HDSchedulerCallback fn, void *data, HDushort)
{
  // Synchronous callbacks run once, inline, on the caller's thread. Used by the
  // probe tools; the node does not use it.
  if (fn) fn(data);
}

void hdUnschedule(HDSchedulerHandle handle)
{
  if (!g_current) return;
  std::lock_guard<std::mutex> lk(g_current->mtx);
  for (auto &cb : g_current->callbacks) {
    if (cb.handle == handle) cb.done = true;
  }
}

void hdStartScheduler(void)
{
  if (!g_current || g_current->running.load()) return;
  g_current->running = true;
  g_current->thread = std::thread(schedulerThread, g_current);
}

void hdStopScheduler(void)
{
  if (!g_current) return;
  g_current->running = false;
  if (g_current->thread.joinable()) g_current->thread.join();
}

void hdSetSchedulerRate(HDulong hz)
{
  if (!g_current) return;
  g_current->arm.mutableParams().update_rate_hz = static_cast<int>(hz);
}

void hdSetDoublev(HDenum param, const HDdouble *v)
{
  SimDevice *d = g_current;
  if (!d || !v) return;
  switch (param) {
    case HD_CURRENT_FORCE:
      for (int i = 0; i < 3; ++i) d->cmd_force_n[i] = v[i];
      break;
    case HD_CURRENT_JOINT_TORQUE:
      for (int i = 0; i < 3; ++i) d->cmd_joint_nm[i] = v[i];
      d->joint_cmd_this_frame = true;
      break;
    default:
      break;
  }
}

void hdGetDoublev(HDenum param, HDdouble *v)
{
  SimDevice *d = g_current;
  if (!v) return;
  if (!d) { v[0] = v[1] = v[2] = 0.0; return; }

  const Vec3 &q = d->arm.qReported();

  switch (param) {
    case HD_CURRENT_JOINT_ANGLES:
      for (int i = 0; i < 3; ++i) v[i] = q[i];
      break;

    case HD_CURRENT_POSITION: {
      const Vec3 p = d->arm.tipPositionM();
      for (int i = 0; i < 3; ++i) v[i] = p[i] * 1000.0;   // HDAPI reports mm
      break;
    }

    case HD_CURRENT_VELOCITY: {
      const Vec3 vel = d->arm.tipVelocityMps();
      for (int i = 0; i < 3; ++i) v[i] = vel[i] * 1000.0;
      break;
    }

    case HD_CURRENT_TRANSFORM: {
      // Column-major 4x4, translation in mm. Position must equal
      // fkFromJacobianConvention so hdapi_fk_residual PASSES against the sim --
      // which is what validates that tool itself.
      for (int i = 0; i < 16; ++i) v[i] = 0.0;
      v[0] = v[5] = v[10] = v[15] = 1.0;
      const Vec3 p = d->arm.tipPositionM();
      v[12] = p[0] * 1000.0;
      v[13] = p[1] * 1000.0;
      v[14] = p[2] * 1000.0;
      break;
    }

    case HD_LAST_JOINT_TORQUE:
      // Previous frame's applied torque, in RAW units. See the header note on
      // why the raw scale is reproduced rather than reporting N*m.
      for (int i = 0; i < 3; ++i) {
        v[i] = d->last_joint_nm[i] * d->arm.params().raw_units_per_nm;
      }
      break;

    case HD_CURRENT_JOINT_TORQUE:
      // Dead on the real unit: it is a joint-space COMMAND buffer, and this
      // codebase commands Cartesian force, so it never gets written and reads
      // back zero. Reproduced faithfully -- a sim that returned something
      // plausible here would contradict a measured hardware fact.
      v[0] = v[1] = v[2] = 0.0;
      break;

    case HD_CURRENT_GIMBAL_TORQUE:
      v[0] = v[1] = v[2] = 0.0;   // passive gimbal
      break;

    case HD_LAST_FORCE:
      for (int i = 0; i < 3; ++i) v[i] = d->last_force_n[i];
      break;

    case HD_MOTOR_TEMPERATURE:
      // Also dead on the real unit -- measured flat through 12 s of sustained
      // 0.5 N. Returning a plausible rising value here would invent a safety
      // signal the hardware does not provide.
      v[0] = v[1] = v[2] = 0.0;
      break;

    case HD_NOMINAL_MAX_FORCE:            v[0] = 3.3;  break;
    case HD_NOMINAL_MAX_CONTINUOUS_FORCE: v[0] = 0.88; break;
    case HD_FORCE_RAMPING_RATE:           v[0] = 0.5;  break;

    default:
      v[0] = v[1] = v[2] = 0.0;
      break;
  }
}

void hdGetIntegerv(HDenum param, HDint *v)
{
  if (!v) return;
  SimDevice *d = g_current;
  switch (param) {
    case HD_CURRENT_BUTTONS:
      *v = 0;   // no buttons in sim; deadman must be disabled for sim runs
      break;
    case HD_UPDATE_RATE:
      *v = d ? d->arm.params().update_rate_hz : 1000;
      break;
    case HD_INSTANTANEOUS_UPDATE_RATE:
      *v = d ? d->arm.params().update_rate_hz : 1000;
      break;
    default:
      *v = 0;
      break;
  }
}

}  // extern "C"
