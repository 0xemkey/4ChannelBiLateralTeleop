// hdapi_pose_hold_sweep.cpp
//
// STANDALONE pose-hold sweep: gravity map + static friction, via the
// APPROACH-DIRECTION method (2026-09-08 re-identification protocol, see
// docs/RIGHT_GRAVITY_IDENTIFICATION_PROTOCOL.md), a sibling to
// hdapi_static_sweep.cpp (unmodified, still the tool of record for the
// AXIS-PROBE/breakaway method).
//
// FORKED FROM hdapi_static_sweep.cpp, not diffed in place. Every
// safety-critical primitive below (servoLoop's force clamps and abort
// handling, autoHold's find-average-freeze-verify sequence, kMaxForceN/
// kMaxBiasN/kMaxTotalN) is BYTE-FOR-BYTE UNCHANGED from that file -- this
// fork only adds new logged fields and a new per-pose procedure built on
// top of the same, already-reviewed primitives. The original tool is left
// completely untouched and remains available.
//
// METHOD DIFFERENCE FROM hdapi_static_sweep.cpp: that tool holds ONE pose,
// then probes breakaway force in 6 Cartesian directions to separate
// gravity/friction via AXIS-PAIR decomposition at a fixed configuration.
// This tool instead visits the SAME target pose from two different
// operator-chosen approach directions (moved into from below vs. from
// above) and simply freezes the holding force each time via the existing,
// unmodified autoHold() -- separating gravity/friction via
// APPROACH-DIRECTION HYSTERESIS instead of active axis probing:
//
//     tau_plus(q)  ~ tau_g(q) + tau_c(q)   (approached from the + side)
//     tau_minus(q) ~ tau_g(q) - tau_c(q)   (approached from the - side)
//     tau_g_measured(q) = [tau_plus(q) + tau_minus(q)] / 2
//     tau_c_measured(q) = [tau_plus(q) - tau_minus(q)] / 2
//
// No active force ramp/breakaway step is performed by this tool -- the
// frozen autoHold() bias, converted through J^T(q), IS the held torque at
// that approach. This is deliberately simpler and lower-risk: it reuses
// the ramp-free "find the holding force, freeze it" primitive only, with
// no new force-commanding logic of its own.
//
// This tool does NOT capture CartesianControllerTelemetry, DOB disturbance
// estimates, or bridge/controller-decomposition telemetry -- those require
// touch_slave.cpp (the ROS node) to own the device, which cannot happen
// while this standalone tool holds it (single-owner OpenHaptics
// constraint, see RIGHT_GRAVITY_IDENTIFICATION_PROTOCOL.md's architecture
// discussion). Those columns are written explicitly as "NA" in the log,
// never silently as 0.
//
// ---------------------------------------------------------------------------
// IT REPORTS N*m WITHOUT NEEDING THE TORQUE SCALE CALIBRATION
// ---------------------------------------------------------------------------
// Breakaway torque is computed as tau = J^T(q) * F_commanded. F_commanded is in
// newtons and is exactly known -- we chose it -- so tau comes out in real N*m
// with no dependence on the uncalibrated HD_LAST_JOINT_TORQUE scale factor
// (prerequisite (a)). This tool is therefore usable BEFORE that calibration.
//
// It also logs raw HD_LAST_JOINT_TORQUE at each breakaway. Dividing the two
// gives an INDEPENDENT estimate of that scale factor, per joint and per pose --
// a cross-check on hdapi_torque_calib.cpp from completely different data. If
// the two disagree, at least one is wrong and neither should be trusted.
//
// DEPENDENCY: the N*m figures are only as good as J(q). Run hdapi_fk_residual
// first and get a PASS. If the FK check fails, the raw force and raw torque
// columns here are still valid, but every J^T-derived number is not.
//
// ---------------------------------------------------------------------------
// THE DECOMPOSITION, AND ITS LIMIT
// ---------------------------------------------------------------------------
// At the instant of breakaway the joint is in static equilibrium against
// commanded torque, gravity, and static friction. Moving in the + direction
// friction resists with -c; in the - direction with +c:
//
//     tau_plus  =  c - g
//     tau_minus = -c - g
// so
//     g = -(tau_plus + tau_minus) / 2      gravity torque
//     c =  (tau_plus - tau_minus) / 2      Coulomb magnitude (>= 0 expected)
//
// LIMIT, stated plainly: this excites CARTESIAN directions, not individual
// joints. Per-joint excitation would need joint-space commanding, which is
// gated off. So at any breakaway several joints are loaded at once and the
// first to slip dominates. Consequences:
//
//   * The per-joint c values are approximate, biased toward whichever joint
//     actually broke. The reported max-|dq| joint tells you which that was.
//   * g is better conditioned: the three axis pairs give three independent
//     estimates of the SAME gravity vector at that pose. Their agreement is
//     the internal consistency check, and the spread is printed. Wide spread
//     means the decomposition is not valid at that pose -- discard it.
//   * Touch friction is known to be asymmetric, so c differing between axes is
//     expected and is itself data, not noise.
//
// SAFETY
//   * Ramp is capped at 0.60 N, under the 0.88 N continuous rating. Raised
//     from 0.30 N on 2026-08-24 with operator authorisation -- see the note on
//     kMaxForceN. hdapi_torque_calib.cpp still uses 0.30 N.
//   * Force is zeroed and output disabled on every exit path, including the
//     abort paths and the no-breakaway path.
//   * A direction that never breaks away by the cap is recorded as "no break"
//     and excluded, rather than being pushed harder.
//   * DO NOT GRIP THE STYLUS during a ramp. A hand adds its own resistance and
//     you would be measuring your arm, not the device's. Position, release,
//     then let it ramp.
//
// BUILD (from this tools/ directory):
//   g++ -std=gnu++17 -I../include hdapi_static_sweep.cpp -o hdapi_static_sweep -lHD -lHDU -lrt -lpthread
//
// RUN:
//   ./hdapi_static_sweep                # HD_DEFAULT_DEVICE
//   ./hdapi_static_sweep "Left"         # named device
//
// Per pose you will be prompted six times, once per direction. Restore the same
// pose each time -- the tool checks and warns if it drifted, because the +/-
// pairing assumes both members share a configuration.

#include <HD/hd.h>

#include "touch_haptics/touch_dob.hpp"

#include <cstdio>
#include <cstring>
#include <cstdlib>
#include <cmath>
#include <csignal>
#include <ctime>
#include <unistd.h>

#include <atomic>
#include <map>
#include <string>
#include <thread>

// ---------------------------------------------------------------------------
// ABORT PATH
// ---------------------------------------------------------------------------
// Ctrl-C must not leave a force latched. Without this, SIGINT during a ramp
// terminates the process with a non-zero force still commanded, and whether the
// device goes passive is then up to the driver's watchdog rather than to us --
// not a guarantee worth relying on when the arm is loaded and a hand is near it.
//
// The handler only sets a flag (the sole async-signal-safe action here). The
// real zeroing happens in servoLoop, which sees the flag and commands zero
// force within one servo frame (~1 ms) -- faster than any userspace teardown
// could manage. The ramp and pose loops then unwind and run normal teardown.
static volatile sig_atomic_t g_abort = 0;
static void onSigint(int) { g_abort = 1; }

// Safety ceiling. Device nominal continuous is 0.88 N.
//
// RAISED 0.30 -> 0.60 N on 2026-08-24, with operator authorisation, because
// 0.30 N could not reach breakaway. Evidence: across 24 ramps at four poses,
// zero breakaways; the single breakaway ever recorded needed 0.256 N, i.e. 85%
// of the old cap, and that was at the most compliant pose tried. At 0.30 N the
// shoulder and elbow only ever saw 0.026-0.041 N*m, which is below this
// device's stiction. The measurement was bounded by the cap, not by physics.
//
// 0.60 N is 68% of the 0.88 N continuous rating, so the motors stay inside
// their continuous envelope indefinitely. Peak rating is 3.3 N.
//
// The step size is unchanged, so force resolution is unchanged; the ramp now
// simply takes ~3.0 s instead of ~1.5 s to reach the cap. A slower ramp is
// mildly better for this measurement, which assumes quasi-static conditions.
// This tool has no ramp/breakaway step (see file header) -- kMaxForceN,
// the per-axis direction table, and the pose count cap that bounded the
// original tool's fixed-size results[][] array are therefore not needed
// here and were removed rather than left dead. Open-loop hold-verification
// threshold, needed by autoHold(), unchanged:
static const double kHoldDriftRad = 0.010;

// ---------------------------------------------------------------------------
// LIVE POSE-MATCHING (added 2026-09-08, corrective rerun after the pilot's
// pose-drift finding). Matches the existing pair tolerance
// (kPairPoseTolRad in the original hdapi_static_sweep.cpp / the offline
// analysis script) as the hard ceiling; a tighter value is the visual
// "GOOD" target only.
// ---------------------------------------------------------------------------
static const double kMatchGoodRad = 0.03;         // visual target: GOOD
static const double kMatchAcceptableRad = 0.05;   // hard ceiling: gates the hold, same as
                                                    // the offline analysis script's pair tolerance
static const double kMatchSettleVelMps = 0.006;    // same constant as kHoldSettleVelMps, duplicated
                                                    // here as a named gate value (not renaming the
                                                    // original -- see its own definition/comment)

// Per-pose reference configuration, set once a pose's FIRST successful hold
// is recorded. Read-only after that except when a NEW pose_id's first hold
// completes. Single-writer (main thread only) -- read by statusLoop's
// background thread for live display, which is the same benign-race-for-
// display-only pattern already used for ctx.q throughout this file.
static std::map<int, touch_haptics::Vec3> g_poseRefs;
// Set by the main loop just before it starts waiting at the "second
// approach" gate, so statusLoop can show live match info; cleared (id=-1)
// the rest of the time so the status file doesn't show a stale match block.
static volatile int g_activeMatchPoseId = -1;
static volatile char g_activeMatchDir = '?';

// ---------------------------------------------------------------------------
// OPTIONAL CSV LOG  (--log <path>)
// ---------------------------------------------------------------------------
// Strictly additive: no safety behaviour depends on this, and nothing here runs
// inside servoLoop. All writes happen from the main thread between ramps, so a
// slow or full disk can never stall a servo frame or delay a force zeroing.
// If the file cannot be opened the tool says so and continues WITHOUT logging,
// because losing a log is not a reason to abandon a hands-on hardware session.
static FILE* g_log = nullptr;
static FILE* g_settleLog = nullptr;   // opened later (--settle-log), declared here so
static int g_holdAttemptId = 0;       // logPoseHoldRow (defined below) can reference it

// Metadata constant for the whole session -- written into every row rather
// than once in a header comment, so a downstream CSV concatenation (or a row
// pulled out of context) never loses which session/commit/orientation it
// came from. Populated once in main() before logOpen(), read-only after.
struct SessionMeta {
    std::string sessionId = "NA";
    std::string gitCommit = "NA";
    std::string device = "NA";
    // From RIGHT_RIG_FRAME_VALIDATION.md's resolved value once that exists;
    // "UNRESOLVED" is the honest default, not a silently-assumed level base.
    std::string baseOrientation = "UNRESOLVED";
};
static SessionMeta g_meta;

static void logOpen(const char* path) {
    if (!path) return;
    g_log = fopen(path, "a");
    if (!g_log) {
        printf("WARNING: could not open log '%s' -- continuing without logging.\n", path);
        return;
    }
    // Header only for a fresh/empty file, so repeated runs append cleanly.
    fseek(g_log, 0, SEEK_END);
    if (ftell(g_log) == 0) {
        fprintf(g_log,
            "tool,session_id,git_commit,device,base_orientation,unix_time,"
            "hold_attempt_id,"
            "pose_id,approach_dir,"
            "q1,q2,q3,qdot1,qdot2,qdot3,qddot1,qddot2,qddot3,"
            "qdot_valid,qddot_valid,"
            "tau_hold_nm_1,tau_hold_nm_2,tau_hold_nm_3,"
            "bias_1,bias_2,bias_3,"
            "pos_x,pos_y,pos_z,"
            "sag_m,max_wander_m,unsettled_frac,settled,open_loop_drift_rad,hold_ok,"
            "cartesian_controller_telemetry,dob_disturbance_estimate,"
            "commanded_joint_torque_from_controller,controller_decomposition\n");
    }
    fflush(g_log);
}

static void logClose() {
    if (g_log) { fflush(g_log); fclose(g_log); g_log = nullptr; }
}

// One row per accepted OR rejected pose-hold attempt. qdot_valid/qddot_valid
// are explicit rather than silently zero -- see the qdot/qddot computation
// in servoLoop, which only becomes valid after enough consecutive frames.
// The last four columns are ALWAYS the literal string "NA": this tool
// cannot produce them (see file header) and a numeric 0 there would be
// indistinguishable from a genuine zero reading -- "NA" cannot be
// mistaken for data the way a fallback zero could.
static void logPoseHoldRow(int poseId, const char* approachDir,
                           const double q[3], const double qdot[3], const double qddot[3],
                           bool qdotValid, bool qddotValid,
                           const double tauHoldNm[3], const double bias[3],
                           const double pos[3], double sagM, double maxWanderM,
                           double unsettledFrac, bool settled, double openLoopDriftRad,
                           bool holdOk) {
    if (!g_log) return;
    // base_orientation is CSV-quoted: its recommended format
    // ("roll=0deg,pitch=0deg,...") contains internal commas, which
    // otherwise silently shift every column after it -- caught during the
    // 2026-09-08 pilot session (all 16 rows had to be repaired offline).
    // session_id/device are quoted too, defensively, for the same reason.
    fprintf(g_log,
        "pose_hold_sweep,\"%s\",%s,\"%s\",\"%s\",%ld,"
        "%d,"
        "%d,%s,"
        "%.9f,%.9f,%.9f,%.9f,%.9f,%.9f,%.9f,%.9f,%.9f,"
        "%d,%d,"
        "%.9f,%.9f,%.9f,"
        "%.9f,%.9f,%.9f,"
        "%.9f,%.9f,%.9f,"
        "%.9f,%.9f,%.9f,%d,%.9f,%d,"
        "NA,NA,NA,NA\n",
        g_meta.sessionId.c_str(), g_meta.gitCommit.c_str(), g_meta.device.c_str(),
        g_meta.baseOrientation.c_str(), (long)time(nullptr),
        g_holdAttemptId,
        poseId, approachDir,
        q[0], q[1], q[2], qdot[0], qdot[1], qdot[2], qddot[0], qddot[1], qddot[2],
        qdotValid ? 1 : 0, qddotValid ? 1 : 0,
        tauHoldNm[0], tauHoldNm[1], tauHoldNm[2],
        bias[0], bias[1], bias[2],
        pos[0], pos[1], pos[2],
        sagM, maxWanderM, unsettledFrac, settled ? 1 : 0, openLoopDriftRad, holdOk ? 1 : 0);
    fflush(g_log);   // flush per row: a hardware session can end abruptly
}

static const char* errName(HDerror code) {
    switch (code) {
        case HD_SUCCESS: return "HD_SUCCESS";
        case HD_INVALID_ENUM: return "HD_INVALID_ENUM";
        case HD_INVALID_VALUE: return "HD_INVALID_VALUE";
        case HD_INVALID_OPERATION: return "HD_INVALID_OPERATION";
        case HD_BAD_HANDLE: return "HD_BAD_HANDLE";
        case HD_WARM_MOTORS: return "HD_WARM_MOTORS";
        case HD_EXCEEDED_MAX_FORCE: return "HD_EXCEEDED_MAX_FORCE";
        case HD_EXCEEDED_MAX_FORCE_IMPULSE: return "HD_EXCEEDED_MAX_FORCE_IMPULSE";
        case HD_EXCEEDED_MAX_VELOCITY: return "HD_EXCEEDED_MAX_VELOCITY";
        case HD_FORCE_ERROR: return "HD_FORCE_ERROR";
        case HD_DEVICE_FAULT: return "HD_DEVICE_FAULT";
        case HD_COMM_ERROR: return "HD_COMM_ERROR";
        case HD_TIMER_ERROR: return "HD_TIMER_ERROR";
        case HD_ILLEGAL_BEGIN: return "HD_ILLEGAL_BEGIN";
        case HD_ILLEGAL_END: return "HD_ILLEGAL_END";
        case HD_FRAME_ERROR: return "HD_FRAME_ERROR";
        default: return "UNKNOWN";
    }
}

struct Ctx {
    HHD hHD;
    volatile double cmd[3];            // RAMP force only
    volatile double bias[3];           // constant known support force, see below
    volatile double q[3];              // latest joint angles
    volatile double lastJointTorque[3];
    volatile double pos_m[3];          // tip position, metres (device frame)
    volatile double vel_mps[3];        // tip velocity, m/s
    volatile unsigned long frames;

    // qdot/qddot: added for this tool. Single/double backward difference of
    // joint angles at the servo rate, same convention as DobDiagnostics.msg's
    // qdot_rad_s ("quantisation-noisy by construction" -- that caveat applies
    // here too, unchanged). dtServoS is measured from wall-clock deltas
    // between frames rather than assumed, since HDAPI's actual callback rate
    // is not guaranteed to be exactly the nominal 1 kHz.
    volatile double qdot[3];
    volatile double qddot[3];
    double qPrev[3];                   // previous-frame q, main-thread-only use pattern (see note below)
    double qdotPrev[3];
    double tPrevS;                     // previous frame's wall time, seconds
    bool haveQPrev;
    bool haveQdotPrev;

    // AUTO-HOLD. While holdMode is 1 the servo loop runs a Cartesian PD toward
    // xTarget and writes the result into bias[]. The main thread averages it,
    // then clears holdMode and freezes that average as a CONSTANT bias.
    //
    // Why find-then-freeze rather than just leaving the controller running:
    // the +/- decomposition requires the support force to be IDENTICAL in the
    // plus and minus ramps so that it cancels from friction and enters gravity
    // as one known offset. A live controller would react to each ramp, making
    // the support force a different unknown function of time in each direction
    // -- which is precisely the defect that makes a supporting hand unusable.
    // Freezing it keeps the force exactly known and exactly equal in both.
    volatile int holdMode;
    volatile double xTarget[3];
};

// ---------------------------------------------------------------------------
// KNOWN SUPPORT BIAS  (--bias-n, adjustable per pose)
// ---------------------------------------------------------------------------
// The problem this solves: this arm only self-holds where gravity torque is
// almost zero (shoulder hanging low). Everywhere else it sags. Measured
// 2026-08-24 -- of 12 poses, the 5 that held unaided spanned just 0.040 rad of
// shoulder angle, giving a design condition number of 12085 versus 15 for the
// full set. The poses carrying the information are exactly the ones that will
// not hold themselves.
//
// Holding the arm by hand does not fix it. The +/- decomposition assumes only
// gravity and friction resist, so an unknown hand force h lands directly in the
// gravity estimate: g_est = g - (h_plus + h_minus)/2.
//
// A COMMANDED bias does fix it, because we know its value exactly:
//
//     tau_plus  =  c - (g + J^T B)        tau_minus = -c - (g + J^T B)
//  => c_est = (tau_plus - tau_minus)/2 = c              bias cancels entirely
//     g_est = -(tau_plus + tau_minus)/2 = g + J^T B     known, subtracted offline
//
// So friction is untouched and gravity picks up an offset we can remove exactly.
// The bias is logged per row so the Python side can do that subtraction.
//
// The bias is applied continuously -- including while positioning between poses
// -- because that is the point: the arm needs support to stay where it is put.
// RAISED 0.40 -> 0.55 N on 2026-08-24: at 0.40 N the arm could not be lifted
// clear of the table, so every -Y ramp bottomed out and returned an EXCEEDED
// error instead of a breakaway. 0.55 N is 63% of the 0.88 N continuous rating
// and is applied continuously, so it stays inside the continuous envelope.
static const double kMaxBiasN = 0.55;

// Hard ceiling on bias + ramp together, applied in the servo loop.
// RAISED 0.80 -> 1.00 N the same day. This total is only reached transiently,
// during a ramp of at most ~3 s, and 1.00 N is 30% of the 3.3 N peak rating.
//
// CAVEAT, stated because it cannot be monitored: HD_MOTOR_TEMPERATURE is dead
// on this unit -- measured flat at ~1e-10 through 12 s of sustained 0.5 N, so
// it is neither sensed nor modelled. There is therefore NO thermal feedback
// available. The margin below the peak rating and the short ramp duration are
// the only protection, which is why the total is held to 1.00 N rather than
// pushed nearer the 3.3 N the hardware would nominally accept.
static const double kMaxTotalN = 1.00;

// Auto-hold controller. Cartesian PD toward the captured tip position.
static const double kHoldKp = 60.0;      // N/m
static const double kHoldKd = 0.4;       // N*s/m
static const double kHoldSettleErrM = 0.004;   // 4 mm
static const double kHoldSettleVelMps = 0.006; // 6 mm/s

static HDerror drainLast() {
    HDerror last = HD_SUCCESS;
    HDErrorInfo e;
    while (HD_DEVICE_ERROR(e = hdGetError())) last = e.errorCode;
    return last;
}

static HDCallbackCode HDCALLBACK servoLoop(void* data) {
    Ctx* c = static_cast<Ctx*>(data);

    hdBeginFrame(c->hHD);

    // Abort wins over any commanded force, checked before the write so an
    // aborted frame can never emit a non-zero force. The bias is suppressed on
    // abort too: Ctrl-C must leave the device fully passive, not merely
    // un-ramped, even though that means the arm will sag as it goes limp.
    if (g_abort) { c->cmd[0] = c->cmd[1] = c->cmd[2] = 0.0; }

    // Tip position/velocity, needed by the auto-hold PD. Read every frame so
    // the status display and the settle test see the same data the PD does.
    HDdouble p_mm[3], v_mm_s[3];
    hdGetDoublev(HD_CURRENT_POSITION, p_mm);
    drainLast();
    hdGetDoublev(HD_CURRENT_VELOCITY, v_mm_s);
    drainLast();
    for (int i = 0; i < 3; ++i) {
        c->pos_m[i] = p_mm[i] * 0.001;
        c->vel_mps[i] = v_mm_s[i] * 0.001;
    }

    // AUTO-HOLD: closed-loop only while searching for the holding force. The
    // result is written into bias[] so that freezing it later is simply a
    // matter of clearing holdMode and leaving the value in place.
    if (c->holdMode && !g_abort) {
        double h[3];
        double hm = 0.0;
        for (int i = 0; i < 3; ++i) {
            h[i] = kHoldKp * (c->xTarget[i] - c->pos_m[i]) - kHoldKd * c->vel_mps[i];
            hm += h[i] * h[i];
        }
        hm = std::sqrt(hm);
        if (hm > kMaxBiasN && hm > 1e-12) {
            const double s = kMaxBiasN / hm;
            h[0] *= s; h[1] *= s; h[2] *= s;
        }
        for (int i = 0; i < 3; ++i) c->bias[i] = h[i];
    }

    double ftmp[3];
    for (int i = 0; i < 3; ++i) {
        ftmp[i] = g_abort ? 0.0 : (c->cmd[i] + c->bias[i]);
    }
    // Hard clamp on bias + ramp combined, downstream of both. Neither a large
    // bias nor a long ramp can push the total past the ceiling.
    const double mag = std::sqrt(ftmp[0]*ftmp[0] + ftmp[1]*ftmp[1] + ftmp[2]*ftmp[2]);
    if (mag > kMaxTotalN && mag > 1e-12) {
        const double s = kMaxTotalN / mag;
        ftmp[0] *= s; ftmp[1] *= s; ftmp[2] *= s;
    }

    HDdouble f[3] = { ftmp[0], ftmp[1], ftmp[2] };
    hdSetDoublev(HD_CURRENT_FORCE, f);

    HDdouble q[3], t[3];
    // Sampled before the frame ends; HD_LAST_JOINT_TORQUE lags one frame, which
    // is fine here because the ramp is quasi-static by construction.
    hdGetDoublev(HD_CURRENT_JOINT_ANGLES, q);
    drainLast();
    hdGetDoublev(HD_LAST_JOINT_TORQUE, t);
    drainLast();

    for (int i = 0; i < 3; ++i) { c->q[i] = q[i]; c->lastJointTorque[i] = t[i]; }

    // qdot/qddot: backward difference against the previous frame, using a
    // MEASURED wall-clock dt (not an assumed 1 kHz). Single writer (this
    // callback thread) for qPrev/qdotPrev/tPrevS/haveQPrev/haveQdotPrev, so
    // these do not need to be volatile -- only the published q/qdot/qddot
    // arrays (read by the main thread) are.
    {
        struct timespec ts;
        clock_gettime(CLOCK_MONOTONIC, &ts);
        const double tNow = ts.tv_sec + ts.tv_nsec * 1e-9;
        if (c->haveQPrev) {
            const double dt = tNow - c->tPrevS;
            if (dt > 1e-6) {
                double qd[3];
                for (int i = 0; i < 3; ++i) {
                    qd[i] = (q[i] - c->qPrev[i]) / dt;
                    c->qdot[i] = qd[i];
                }
                if (c->haveQdotPrev) {
                    for (int i = 0; i < 3; ++i) c->qddot[i] = (qd[i] - c->qdotPrev[i]) / dt;
                } else {
                    for (int i = 0; i < 3; ++i) c->qddot[i] = 0.0;
                }
                for (int i = 0; i < 3; ++i) c->qdotPrev[i] = qd[i];
                c->haveQdotPrev = true;
            }
        }
        for (int i = 0; i < 3; ++i) c->qPrev[i] = q[i];
        c->tPrevS = tNow;
        c->haveQPrev = true;
    }

    hdEndFrame(c->hHD);

    c->frames++;
    return HD_CALLBACK_CONTINUE;
}

static void zeroForce(Ctx& c) { c.cmd[0] = c.cmd[1] = c.cmd[2] = 0.0; }

static void readQ(Ctx& c, touch_haptics::Vec3& out) {
    for (int i = 0; i < 3; ++i) out[i] = c.q[i];
}

// ---------------------------------------------------------------------------
// LIVE STATUS DISPLAY  (--status <path>)
// ---------------------------------------------------------------------------
// Only one process may own an OpenHaptics device, so an external monitor cannot
// read angles while this tool is running. The live view therefore has to come
// from inside this process.
//
// A background thread rewrites a small status file at 10 Hz. The operator
// watches it from a second terminal. Deliberately NOT in servoLoop: file I/O at
// HD_MAX_SCHEDULER_PRIORITY could stall a servo frame, which is the one thing
// that must never be delayed while force is live.
//
// The thread reads ctx.q without a lock. That is a benign race for a DISPLAY
// value -- a torn read would show one stale angle for 100 ms and nothing else.
// This value never feeds the control path, the breakaway test, or the logged
// data, all of which read ctx.q through readQ() on the main thread as before.
static std::atomic<bool> g_statusRun{false};
static std::thread g_statusThread;

static void statusLoop(Ctx* c, std::string path) {
    touch_haptics::Vec3 ref{c->q[0], c->q[1], c->q[2]};
    int settle = 0;
    while (g_statusRun.load()) {
        touch_haptics::Vec3 q{c->q[0], c->q[1], c->q[2]};
        double drift = 0.0;
        for (int j = 0; j < 3; ++j) drift = std::max(drift, std::fabs(q[j] - ref[j]));
        // Re-baseline once the arm has been quiet for ~1.5 s, so "drift" always
        // means "since it last settled" rather than "since the tool started".
        if (drift < 0.002) { if (++settle > 15) { ref = q; settle = 0; } }
        else settle = 0;

        const touch_haptics::Vec3 p = touch_haptics::fkFromJacobianConvention(q);
        const touch_haptics::SingularityMetrics sm = touch_haptics::singularityMetrics(q);

        std::string tmp = path + ".tmp";
        if (FILE* f = fopen(tmp.c_str(), "w")) {
            fprintf(f, "  LIVE  (updates 10 Hz)\n\n");
            fprintf(f, "  joint angles   q1 waist    %+8.4f rad  %+7.1f deg\n", q[0], q[0] * 180.0 / M_PI);
            fprintf(f, "                 q2 shoulder %+8.4f rad  %+7.1f deg\n", q[1], q[1] * 180.0 / M_PI);
            fprintf(f, "                 q3 elbow    %+8.4f rad  %+7.1f deg\n", q[2], q[2] * 180.0 / M_PI);
            fprintf(f, "\n  tip position   x %+8.4f   y %+8.4f   z %+8.4f  m\n", p[0], p[1], p[2]);
            fprintf(f, "                 (FK frame, base offset not removed)\n");
            fprintf(f, "\n  reach r        %7.4f m      cos(q2-q3) %+6.3f%s\n",
                    sm.radial_r, sm.align_cos, sm.near_singular ? "   <-- NEAR SINGULAR" : "");
            fprintf(f, "\n  drift          %7.4f rad   %s\n", drift,
                    drift < 0.010 ? "STEADY - safe to ramp"
                                  : "MOVING - wait, the 0.010 rad hold check will fail");
            fprintf(f, "\n  support bias   %+6.3f %+6.3f %+6.3f  N  (known, subtracted offline)\n",
                    (double)c->bias[0], (double)c->bias[1], (double)c->bias[2]);
            fprintf(f, "  ramp force     %+6.3f %+6.3f %+6.3f  N\n",
                    (double)c->cmd[0], (double)c->cmd[1], (double)c->cmd[2]);
            fprintf(f, "  total to motor %+6.3f %+6.3f %+6.3f  N\n",
                    (double)(c->cmd[0] + c->bias[0]), (double)(c->cmd[1] + c->bias[1]),
                    (double)(c->cmd[2] + c->bias[2]));

            const int matchPoseId = g_activeMatchPoseId;   // benign racy read, display only
            if (matchPoseId >= 0) {
                auto it = g_poseRefs.find(matchPoseId);
                if (it != g_poseRefs.end()) {
                    const touch_haptics::Vec3& qref = it->second;
                    double dq[3], maxAbs = 0.0;
                    for (int i = 0; i < 3; ++i) {
                        dq[i] = q[i] - qref[i];
                        maxAbs = std::max(maxAbs, std::fabs(dq[i]));
                    }
                    double speed = 0.0;
                    for (int i = 0; i < 3; ++i) speed += c->vel_mps[i] * c->vel_mps[i];
                    speed = std::sqrt(speed);
                    const char* label = maxAbs <= kMatchGoodRad ? "GOOD"
                                       : maxAbs <= kMatchAcceptableRad ? "ACCEPTABLE" : "NOT MATCHED";
                    fprintf(f, "\n  === POSE MATCH: pose %d, approaching '%c' ===\n",
                            matchPoseId, g_activeMatchDir);
                    fprintf(f, "  reference q:  [%+7.4f, %+7.4f, %+7.4f] rad\n",
                            qref[0], qref[1], qref[2]);
                    fprintf(f, "  current q:    [%+7.4f, %+7.4f, %+7.4f] rad\n", q[0], q[1], q[2]);
                    fprintf(f, "  delta:        [%+7.4f, %+7.4f, %+7.4f] rad\n", dq[0], dq[1], dq[2]);
                    fprintf(f, "  max |delta| = %.4f rad -- %s (GOOD<=%.2f, ACCEPTABLE<=%.2f)\n",
                            maxAbs, label, kMatchGoodRad, kMatchAcceptableRad);
                    fprintf(f, "  Cartesian speed = %.4f m/s -- %s\n", speed,
                            speed < kMatchSettleVelMps ? "SETTLED" : "NOT SETTLED, still moving");
                    fprintf(f, "  %s\n",
                            (maxAbs <= kMatchAcceptableRad && speed < kMatchSettleVelMps)
                                ? "Ready -- operator may say go."
                                : "NOT ready -- adjust position/settle before saying go.");
                }
            }
            fclose(f);
            rename(tmp.c_str(), path.c_str());   // atomic swap; never a half-written file
        }
        usleep(100000);
    }
}

static void statusStart(Ctx& c, const char* path) {
    if (!path) return;
    g_statusRun = true;
    g_statusThread = std::thread(statusLoop, &c, std::string(path));
}

static void statusStop() {
    if (g_statusRun.load()) {
        g_statusRun = false;
        if (g_statusThread.joinable()) g_statusThread.join();
    }
}

// ---------------------------------------------------------------------------
// AUTO-HOLD: find the constant force that keeps the arm at the pose, then
// freeze it. Returns true if a stable hold was achieved.
//
// Sequence, and why it is in this order:
//   1. Capture the tip position the operator is holding. This is the target;
//      it is captured while they still have the arm, so it is the pose they
//      chose rather than wherever it sags to.
//   2. Enable the PD and tell them to release. The PD then does the work of
//      discovering how much force this pose needs -- which is exactly the
//      thing that cannot be guessed, because it varies with configuration.
//   3. Let the transient die, then AVERAGE the commanded force. Averaging
//      matters: the instantaneous value at any one frame includes the
//      derivative term reacting to residual micro-motion, and freezing that
//      would bake a velocity artefact into the constant.
//   4. Freeze: clear holdMode, leave the averaged value in bias[].
//   5. Verify open-loop for a second. If it sags now, the averaged force was
//      wrong and the pose is rejected rather than measured badly.
// Additive diagnostic out-params (all optional, default nullptr): none of
// them are read by the control/decision logic below, which is otherwise
// byte-for-byte identical to hdapi_static_sweep.cpp's autoHold(). They only
// let the caller log what this function already computes internally.
// ---------------------------------------------------------------------------
// SETTLE-TRACE LOG (--settle-log <path>), added 2026-09-08.
//
// WHY: the dedicated stillness calibration (calibrate_stillness_noise.py)
// measured an encoder noise floor of EXACTLY ZERO at true rest -- real,
// but useless as a qdot accept/reject threshold (k * 0 = 0 rejects every
// real sample). A single v/L Cartesian-to-joint conversion was also
// rejected (operator direction, 2026-09-08): x_dot = J(q) @ qdot is
// configuration-dependent, not a fixed scalar. The correct empirical
// approach is to record qdot alongside Cartesian speed through every
// hold's natural post-release settling transient, then -- offline, in
// analyze_settle_transients.py -- find the qdot value AT the moment
// Cartesian speed first sustains below the already-safety-reviewed
// kHoldSettleVelMps gate, across many real holds. This block records
// exactly that raw trace; it computes no threshold itself and changes no
// control behavior.
static void settleLogOpen(const char* path) {
    if (!path) return;
    g_settleLog = fopen(path, "a");
    if (!g_settleLog) {
        printf("WARNING: could not open settle-trace log '%s' -- continuing without it.\n", path);
        return;
    }
    fseek(g_settleLog, 0, SEEK_END);
    if (ftell(g_settleLog) == 0) {
        fprintf(g_settleLog, "tool,session_id,hold_attempt_id,t_since_release_s,"
                             "cart_speed_mps,qdot1,qdot2,qdot3,q1,q2,q3\n");
    }
    fflush(g_settleLog);
}

static void settleLogClose() {
    if (g_settleLog) { fflush(g_settleLog); fclose(g_settleLog); g_settleLog = nullptr; }
}

static bool autoHold(Ctx& c, touch_haptics::Vec3& biasOut,
                     double* outSagM = nullptr, double* outMaxWanderM = nullptr,
                     double* outUnsettledFrac = nullptr, double* outOpenLoopDriftRad = nullptr) {
    for (int i = 0; i < 3; ++i) c.xTarget[i] = c.pos_m[i];

    printf("    target captured. RELEASE THE ARM NOW.\n");
    fflush(stdout);
    c.holdMode = 1;
    ++g_holdAttemptId;

    // Let the operator let go and the transient settle. This IS the decay
    // this tool needs to observe -- record it if a settle-log is open.
    // Purely additive: no new timing, no new control action, same 200 x
    // 10ms loop as before.
    for (int k = 0; k < 200 && !g_abort; ++k) {
        if (g_settleLog) {
            double speed = 0.0;
            for (int i = 0; i < 3; ++i) speed += c.vel_mps[i] * c.vel_mps[i];
            speed = std::sqrt(speed);
            fprintf(g_settleLog, "settle_trace,%s,%d,%.3f,%.6f,%.9f,%.9f,%.9f,%.9f,%.9f,%.9f\n",
                    g_meta.sessionId.c_str(), g_holdAttemptId, k * 0.01, speed,
                    c.qdot[0], c.qdot[1], c.qdot[2], c.q[0], c.q[1], c.q[2]);
        }
        usleep(10000);
    }
    if (g_settleLog) fflush(g_settleLog);   // per-hold flush, same reasoning as the main log
    if (g_abort) { c.holdMode = 0; return false; }

    // Average over 1.5 s, and require it to be genuinely settled while doing so.
    //
    // "Settled" is tested as STABILITY, not as closeness to xTarget. A pure PD
    // needs a standing error to generate force at all: holding F newtons costs
    // F/kHoldKp of offset, so at 0.35 N and Kp=60 the arm rests ~5.8 mm below
    // where the operator held it. That is expected, not a fault -- and the pose
    // actually measured is whatever q0 is logged at ramp time, so the fit stays
    // self-consistent either way. Testing absolute error here flagged every
    // single pose as "still moving", which would have trained the operator to
    // ignore a warning that is supposed to mean something.
    double acc[3] = {0, 0, 0};
    int n = 0, unsettled = 0;
    double pStart[3] = {c.pos_m[0], c.pos_m[1], c.pos_m[2]};
    double maxWander = 0.0;
    for (int k = 0; k < 150 && !g_abort; ++k) {
        double wander = 0.0, vel = 0.0;
        for (int i = 0; i < 3; ++i) {
            const double d = c.pos_m[i] - pStart[i];
            wander += d * d;
            vel += c.vel_mps[i] * c.vel_mps[i];
        }
        wander = std::sqrt(wander);
        maxWander = std::max(maxWander, wander);
        if (wander > kHoldSettleErrM || std::sqrt(vel) > kHoldSettleVelMps) ++unsettled;
        for (int i = 0; i < 3; ++i) acc[i] += c.bias[i];
        ++n;
        usleep(10000);
    }
    if (g_abort) { c.holdMode = 0; return false; }

    // Standing offset from where the operator held it. Reported so the shift is
    // visible rather than being a silent few millimetres.
    double sag = 0.0;
    for (int i = 0; i < 3; ++i) {
        const double d = c.xTarget[i] - c.pos_m[i];
        sag += d * d;
    }
    sag = std::sqrt(sag);

    double mag = 0.0;
    for (int i = 0; i < 3; ++i) { acc[i] /= n; mag += acc[i] * acc[i]; }
    mag = std::sqrt(mag);

    // Freeze. holdMode is cleared FIRST so the servo loop stops writing bias[]
    // before the main thread does, leaving exactly one writer.
    c.holdMode = 0;
    usleep(5000);
    for (int i = 0; i < 3; ++i) { c.bias[i] = acc[i]; biasOut[i] = acc[i]; }

    if (outSagM) *outSagM = sag;
    if (outMaxWanderM) *outMaxWanderM = maxWander;
    if (outUnsettledFrac) *outUnsettledFrac = n > 0 ? (double)unsettled / n : 0.0;

    printf("    holding force found: [%+.3f %+.3f %+.3f] N  |F| = %.3f N",
           acc[0], acc[1], acc[2], mag);
    if (mag >= kMaxBiasN - 1e-6) printf("   <-- AT THE %.2f N CAP", kMaxBiasN);
    printf("\n");
    printf("    settled %.1f mm below where you held it (expected: |F|/Kp = %.1f mm)\n",
           sag * 1000.0, mag / kHoldKp * 1000.0);
    if (unsettled > n / 4) {
        printf("    NOTE: position wandered up to %.1f mm during averaging; the arm may be\n"
               "    drifting or touching something. The frozen value is less reliable.\n",
               maxWander * 1000.0);
    }

    // Open-loop verification: does the frozen constant actually hold it?
    touch_haptics::Vec3 q0;
    readQ(c, q0);
    double worst = 0.0;
    for (int k = 0; k < 100 && !g_abort; ++k) {
        usleep(10000);
        touch_haptics::Vec3 qn;
        readQ(c, qn);
        for (int j = 0; j < 3; ++j) worst = std::max(worst, std::fabs(qn[j] - q0[j]));
    }
    if (g_abort) return false;
    if (outOpenLoopDriftRad) *outOpenLoopDriftRad = worst;

    if (worst > kHoldDriftRad) {
        printf("    FROZEN HOLD FAILED: drifted %.4f rad open-loop (limit %.3f).\n",
               worst, kHoldDriftRad);
        printf("    This pose needs more than the %.2f N cap, or it is resting on\n"
               "    something. Reposition and retry, or skip it.\n", kMaxBiasN);
        return false;
    }
    printf("    frozen hold verified: %.4f rad drift over 1 s. Ramps can proceed.\n", worst);
    return true;
}

// verifySelfHold() / rampDirection() (probe-force breakaway ramping) removed
// in this fork -- see file header. This tool only ever calls autoHold(),
// unchanged from hdapi_static_sweep.cpp, to find and freeze the holding
// force at each operator-chosen approach.

int main(int argc, char** argv) {
    // Additive arg parsing: the first non-flag argument is still the device
    // name, so every existing invocation keeps working unchanged.
    const char* deviceName = HD_DEFAULT_DEVICE;
    const char* logPath = nullptr;
    const char* settleLogPath = nullptr;
    const char* statusPath = nullptr;
    double biasN = 0.0;     // support-force magnitude, N (startup only, see applyBias below)
    int    biasAxis = 1;    // 1 = y = up on a standard Touch
    bool haveDevice = false;
    const char* sessionId = nullptr;
    const char* baseOrientation = nullptr;   // e.g. "roll=+0.3deg,pitch=-4.1deg,method=digital_angle_gauge"
    for (int i = 1; i < argc; ++i) {
        if (std::strcmp(argv[i], "--log") == 0 && i + 1 < argc) {
            logPath = argv[++i];
        } else if (std::strncmp(argv[i], "--log=", 6) == 0) {
            logPath = argv[i] + 6;
        } else if (std::strcmp(argv[i], "--settle-log") == 0 && i + 1 < argc) {
            settleLogPath = argv[++i];
        } else if (std::strcmp(argv[i], "--status") == 0 && i + 1 < argc) {
            statusPath = argv[++i];
        } else if (std::strncmp(argv[i], "--status=", 9) == 0) {
            statusPath = argv[i] + 9;
        } else if (std::strcmp(argv[i], "--bias-n") == 0 && i + 1 < argc) {
            biasN = atof(argv[++i]);
        } else if (std::strcmp(argv[i], "--bias-axis") == 0 && i + 1 < argc) {
            biasAxis = atoi(argv[++i]);
        } else if (std::strcmp(argv[i], "--session") == 0 && i + 1 < argc) {
            sessionId = argv[++i];
        } else if (std::strcmp(argv[i], "--base-orientation") == 0 && i + 1 < argc) {
            baseOrientation = argv[++i];
        } else if (!haveDevice) {
            deviceName = argv[i];
            haveDevice = true;
        }
    }

    if (!sessionId) {
        printf("FATAL: --session <id> is required (e.g. --session pilot_20260909).\n"
               "This identifies every row in the log -- there is no safe default.\n");
        return 2;
    }
    g_meta.sessionId = sessionId;
    g_meta.device = haveDevice ? deviceName : "HD_DEFAULT_DEVICE";
    if (baseOrientation) {
        g_meta.baseOrientation = baseOrientation;
    } else {
        printf("WARNING: --base-orientation not given. Logging base_orientation=UNRESOLVED\n"
               "for every row. Per RIGHT_RIG_FRAME_VALIDATION.md, do not run a real\n"
               "identification session without first resolving this -- pass\n"
               "--base-orientation once it's measured, e.g.\n"
               "  --base-orientation \"roll=+0.3deg,pitch=-4.1deg,unc=0.2deg,method=digital_angle_gauge,adjustable=no\"\n\n");
    }
    {
        // Relies on the operator's cwd being inside the repo (true for every
        // invocation in the runbook) -- falls back to "NA" (the default)
        // rather than guessing a path if that's not the case.
        FILE* p = popen("git rev-parse HEAD 2>/dev/null", "r");
        if (p) {
            char buf[128] = {0};
            if (fgets(buf, sizeof(buf), p)) {
                size_t len = strlen(buf);
                if (len > 0 && buf[len - 1] == '\n') buf[len - 1] = '\0';
                if (buf[0]) g_meta.gitCommit = buf;
            }
            pclose(p);
        }
    }

    printf("=== Pose-hold sweep: gravity map + static friction via approach direction ===\n");
    printf("session: %s   commit: %s   base_orientation: %s\n",
           g_meta.sessionId.c_str(), g_meta.gitCommit.c_str(), g_meta.baseOrientation.c_str());
    printf("device: %s\n", haveDevice ? deviceName : "(default)");
    printf("bias cap %.2f N, total cap %.2f N (continuous rating 0.88 N) -- no active\n"
           "ramp/breakaway force in this tool, only the auto-hold bias\n",
           kMaxBiasN, kMaxTotalN);
    printf("L1 = %.4f m  L2 = %.4f m\n",
           touch_haptics::kL1Meters, touch_haptics::kL2Meters);
    printf("\nDO NOT GRIP THE STYLUS during a ramp -- position it, release, then\n"
           "let the ramp run. Gripping measures your arm, not the device.\n");
    printf("Run hdapi_fk_residual first: the N*m columns depend on J(q).\n");
    printf("\nABORT: Ctrl-C at any time. The servo loop zeroes force within ~1 ms,\n"
           "then the tool unwinds and disables force output. Safe to use mid-ramp.\n\n");

    logOpen(logPath);
    if (g_log) printf("logging rows to: %s\n\n", logPath);
    settleLogOpen(settleLogPath);
    if (g_settleLog) printf("logging settle-transient trace to: %s\n\n", settleLogPath);

    struct sigaction sa;
    memset(&sa, 0, sizeof(sa));
    sa.sa_handler = onSigint;
    sigaction(SIGINT, &sa, nullptr);
    sigaction(SIGTERM, &sa, nullptr);

    HHD hHD = hdInitDevice(deviceName);
    HDErrorInfo e;
    if (HD_DEVICE_ERROR(e = hdGetError())) {
        printf("FATAL: hdInitDevice failed: %s (0x%04X)\n", errName(e.errorCode), e.errorCode);
        return 1;
    }
    hdMakeCurrentDevice(hHD);
    hdEnable(HD_FORCE_OUTPUT);
    if (HD_DEVICE_ERROR(e = hdGetError())) {
        printf("FATAL: hdEnable(HD_FORCE_OUTPUT) failed: %s\n", errName(e.errorCode));
        hdDisableDevice(hHD);
        return 1;
    }

    Ctx ctx;
    memset(&ctx, 0, sizeof(ctx));
    ctx.hHD = hHD;

    HDSchedulerHandle handle =
        hdScheduleAsynchronous(servoLoop, &ctx, HD_MAX_SCHEDULER_PRIORITY);
    hdStartScheduler();
    if (HD_DEVICE_ERROR(e = hdGetError())) {
        printf("FATAL: hdStartScheduler failed: %s (0x%04X)\n", errName(e.errorCode), e.errorCode);
        hdDisable(HD_FORCE_OUTPUT); hdDisableDevice(hHD);
        return 1;
    }

    usleep(200000);
    if (ctx.frames == 0) {
        printf("FATAL: servo loop never ran. Scheduler is not alive.\n");
        zeroForce(ctx);
        hdStopScheduler(); hdUnschedule(handle);
        hdDisable(HD_FORCE_OUTPUT); hdDisableDevice(hHD);
        return 1;
    }

    // Support bias, ramped in gently. A step to 0.4 N would jolt an arm the
    // operator may already have a hand near; 1 s of ramp is imperceptible in
    // procedure terms and removes that.
    auto applyBias = [&](double target) {
        if (biasAxis < 0 || biasAxis > 2) biasAxis = 1;
        if (target > kMaxBiasN) {
            printf("  bias %.3f N exceeds the %.2f N bias cap; clamped.\n", target, kMaxBiasN);
            target = kMaxBiasN;
        }
        if (target < 0.0) target = 0.0;
        const double from = ctx.bias[biasAxis];
        for (int k = 0; k <= 50 && !g_abort; ++k) {
            const double v = from + (target - from) * (k / 50.0);
            ctx.bias[0] = ctx.bias[1] = ctx.bias[2] = 0.0;
            ctx.bias[biasAxis] = v;
            usleep(20000);
        }
        if (g_abort) { ctx.bias[0] = ctx.bias[1] = ctx.bias[2] = 0.0; }
    };

    if (biasN > 0.0) {
        printf("applying support bias %.3f N on %c ...\n", biasN, "xyz"[biasAxis < 0 || biasAxis > 2 ? 1 : biasAxis]);
        applyBias(biasN);
        printf("  bias live. The arm is now being held UP by a known force.\n");
        printf("  It is logged per row and subtracted from gravity offline.\n\n");
    }

    // Started only after the servo loop is confirmed alive, so it never shows
    // stale zeros as if they were real angles.
    statusStart(ctx, statusPath);
    if (statusPath) {
        printf("live status -> %s\n", statusPath);
        printf("  watch it from another terminal with:\n");
        printf("    watch -n 0.2 cat %s\n\n", statusPath);
    }

    // Pose-hold loop. Operator drives both the pose ID and the approach-
    // direction label -- this tool does not sequence "below then above"
    // itself, because only the operator can actually move the arm to
    // approach from one side or the other. A given pose_id is expected to
    // appear twice in the log (once '+' once '-'); the offline analysis
    // script pairs them by pose_id, not by call order.
    int holdCount = 0;
    int currentPoseId = 1;
    while (!g_abort) {
        printf("\n--- hold #%d --- pose_id [%d] ('r<N>' clears pose N's saved "
               "reference, e.g. a bad first hold slipped through): ", holdCount + 1, currentPoseId);
        fflush(stdout);
        char poseLine[64];
        if (!fgets(poseLine, sizeof(poseLine), stdin)) break;
        if (g_abort) break;
        if (poseLine[0] == 'q' || poseLine[0] == 'Q') break;
        if (poseLine[0] == 'r' || poseLine[0] == 'R') {
            const int toClear = atoi(poseLine + 1);
            if (g_poseRefs.erase(toClear) > 0) {
                printf("    cleared reference for pose %d -- its next successful hold "
                       "becomes the new reference.\n", toClear);
            } else {
                printf("    pose %d had no saved reference.\n", toClear);
            }
            continue;   // re-prompt, does not count as a hold attempt
        }
        if (poseLine[0] != '\n' && poseLine[0] != '\0') {
            const int typed = atoi(poseLine);
            if (typed > 0) currentPoseId = typed;
        }

        char dirLine[64];
        char approachDir[8] = "+";
        printf("    approach direction, '+' (from above/positive side) or '-' "
               "(from below/negative side) [+]: ");
        fflush(stdout);
        if (!fgets(dirLine, sizeof(dirLine), stdin)) break;
        if (g_abort) break;
        if (dirLine[0] == '-') { approachDir[0] = '-'; }

        // Live pose-matching gate: only engages if this pose_id already has
        // a stored reference (i.e. this is a second/later approach for it).
        // The FIRST approach to any pose_id has nothing to match against
        // and proceeds exactly as before.
        const bool haveRef = g_poseRefs.count(currentPoseId) > 0;
        if (haveRef) {
            g_activeMatchPoseId = currentPoseId;
            g_activeMatchDir = approachDir[0];
        }
        bool proceedToHold = false;
        while (!proceedToHold && !g_abort) {
            if (haveRef) {
                printf("    move the arm to pose %d, approaching from the %s side.\n"
                       "    Live match feedback is in the status file%s.\n"
                       "    Enter when matched+settled and ready ('q' to finish): ",
                       currentPoseId, approachDir,
                       statusPath ? "" : " (none configured with --status, watch printed values below)");
            } else {
                printf("    move the arm to pose %d, approaching from the %s side, clear\n"
                       "    of the table, then Enter to auto-find the holding force ('q' to\n"
                       "    finish the session, blank pose_id/direction repeats the last): ",
                       currentPoseId, approachDir);
            }
            fflush(stdout);
            char goLine[16];
            if (!fgets(goLine, sizeof(goLine), stdin)) { g_activeMatchPoseId = -1; goto sessionEnd; }
            if (g_abort) break;
            if (goLine[0] == 'q' || goLine[0] == 'Q') { g_activeMatchPoseId = -1; goto sessionEnd; }

            if (!haveRef) { proceedToHold = true; break; }

            // Operator said go -- verify match+settle NOW, per the approved
            // gate: pose matched + settled + operator says go = hold may
            // begin. Not auto-fired on MATCH alone; not skipped on operator
            // say-so alone either.
            const touch_haptics::Vec3& qref = g_poseRefs[currentPoseId];
            double maxAbs = 0.0, dq[3];
            for (int i = 0; i < 3; ++i) {
                dq[i] = ctx.q[i] - qref[i];
                maxAbs = std::max(maxAbs, std::fabs(dq[i]));
            }
            double speed = 0.0;
            for (int i = 0; i < 3; ++i) speed += ctx.vel_mps[i] * ctx.vel_mps[i];
            speed = std::sqrt(speed);
            if (maxAbs <= kMatchAcceptableRad && speed < kMatchSettleVelMps) {
                printf("    MATCHED (max|delta|=%.4f rad) and SETTLED (speed=%.4f m/s) -- proceeding.\n",
                       maxAbs, speed);
                proceedToHold = true;
            } else {
                printf("    NOT READY: max|delta|=%.4f rad (need <=%.3f), speed=%.4f m/s (need <%.3f).\n"
                       "    Adjust and press Enter again, or 'q' to finish.\n",
                       maxAbs, kMatchAcceptableRad, speed, kMatchSettleVelMps);
            }
        }
        if (g_abort) break;
        g_activeMatchPoseId = -1;

        touch_haptics::Vec3 found{};
        double sagM = 0.0, maxWanderM = 0.0, unsettledFrac = 0.0, openLoopDriftRad = 0.0;
        const bool holdOk = autoHold(ctx, found, &sagM, &maxWanderM, &unsettledFrac, &openLoopDriftRad);
        if (g_abort) break;

        touch_haptics::Vec3 qSettled;
        readQ(ctx, qSettled);
        const double qArr[3] = {qSettled[0], qSettled[1], qSettled[2]};
        const double qdotArr[3] = {ctx.qdot[0], ctx.qdot[1], ctx.qdot[2]};
        const double qddotArr[3] = {ctx.qddot[0], ctx.qddot[1], ctx.qddot[2]};
        // Valid once servoLoop has accumulated at least two consecutive
        // frames since this hold began -- true in practice by the time
        // autoHold (several seconds) returns, but checked explicitly rather
        // than assumed.
        const bool qdotValid = ctx.haveQdotPrev;
        const bool qddotValid = ctx.haveQdotPrev;   // qddot needs the same history

        touch_haptics::Vec3 tauHold = touch_haptics::jacobianTransposeTimes(
            touch_haptics::jacobian(qSettled), found);
        const double tauHoldArr[3] = {tauHold[0], tauHold[1], tauHold[2]};
        const double biasArr[3] = {found[0], found[1], found[2]};
        const double posArr[3] = {ctx.pos_m[0], ctx.pos_m[1], ctx.pos_m[2]};

        if (!holdOk) {
            printf("    HOLD REJECTED -- logged with hold_ok=0, exclude from fitting.\n");
            ctx.holdMode = 0;
            ctx.bias[0] = ctx.bias[1] = ctx.bias[2] = 0.0;
        } else {
            printf("    tau_hold = [%+8.5f %+8.5f %+8.5f] N*m at q=[%+7.4f %+7.4f %+7.4f] rad\n",
                   tauHoldArr[0], tauHoldArr[1], tauHoldArr[2], qArr[0], qArr[1], qArr[2]);
            // Save this pose's reference for live-matching the OPPOSITE
            // approach, but only from its first successful hold -- a
            // manually-invalidated/redone first attempt (like the pilot's
            // hold #1) should not poison the reference for later, correctly
            // executed approaches. Deliberately not overwritten on repeats.
            if (!g_poseRefs.count(currentPoseId)) {
                g_poseRefs[currentPoseId] = qSettled;
                printf("    (saved as reference q for pose %d's opposite approach)\n", currentPoseId);
            }
        }

        logPoseHoldRow(currentPoseId, approachDir, qArr, qdotArr, qddotArr,
                       qdotValid, qddotValid, tauHoldArr, biasArr, posArr,
                       sagM, maxWanderM, unsettledFrac, /*settled=*/unsettledFrac <= 0.25,
                       openLoopDriftRad, holdOk);
        ++holdCount;

        if (!holdOk) {
            printf("    Reposition and retry this pose_id/direction.\n");
        } else {
            // Release after logging: the operator repositions by hand for the
            // next hold, same "hands off only during the hold itself" pattern
            // as the original tool's per-direction ramp step.
            ctx.holdMode = 0;
            ctx.bias[0] = ctx.bias[1] = ctx.bias[2] = 0.0;
        }
    }
    sessionEnd:
    printf("\n%d hold(s) recorded across this session.\n", holdCount);

    // Zero force before anything else, whatever happened above. The status
    // thread is stopped only after that, so the display stays live right
    // through an abort rather than freezing on the last pre-abort frame.
    zeroForce(ctx);
    usleep(150000);
    statusStop();
    unsigned long totalFrames = ctx.frames;
    hdStopScheduler();
    hdUnschedule(handle);
    hdDisable(HD_FORCE_OUTPUT);
    hdDisableDevice(hHD);
    printf("\nForce zeroed, output disabled. Servo frames: %lu\n", totalFrames);
    if (g_abort) {
        printf("ABORTED by signal. Force was zeroed in the servo loop within ~1 ms\n"
               "of the signal, before this teardown ran. Partial data below.\n");
    }

    if (holdCount == 0) {
        printf("No holds recorded.\n");
        logClose();
        settleLogClose();
        return 1;
    }

    printf("\n=== How to use this output ===\n");
    printf("  * This tool does NOT compute the gravity/friction split itself -- unlike\n"
           "    hdapi_static_sweep.cpp, whose axis-pair decomposition doesn't apply\n"
           "    to approach-direction pairs. Run the offline analysis script instead:\n"
           "      python3 identification/analyze_pose_hold_sweep.py <log path>\n"
           "    It pairs rows by pose_id, computes tau_g/tau_c per the same\n"
           "    +/- averaging formula, and produces the data-quality summary\n"
           "    required before any fit (per RIGHT_GRAVITY_IDENTIFICATION_PROTOCOL.md\n"
           "    Phase 4 -- do not fit before reviewing that summary).\n");
    printf("  * Nothing is written anywhere by this tool. M_n, gravity, and the\n"
           "    commissioning flags remain untouched and unset.\n");
    if (g_log) printf("\nlog written: %s\n", logPath);
    if (g_settleLog) printf("settle-transient trace written: %s\n", settleLogPath);
    logClose();
    settleLogClose();
    printf("\n=== Done. %d hold(s) recorded. ===\n", holdCount);
    return 0;
}
