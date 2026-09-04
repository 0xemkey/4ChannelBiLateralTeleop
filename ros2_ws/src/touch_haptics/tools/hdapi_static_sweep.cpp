// hdapi_static_sweep.cpp
//
// STANDALONE static-pose sweep: gravity map + friction floor.
//
// Measures, per pose, the breakaway wrench needed to start the arm moving in
// each of six Cartesian directions (+/-x, +/-y, +/-z), and projects it through
// J^T(q) to get joint torques. From the +/- pairs it separates the two things
// the disturbance observer cannot currently distinguish:
//
//   * GRAVITY  -- the pose-dependent bias in tau_dis_hat. Open point 4. This
//                 is the term that otherwise dominates the observer's estimate
//                 on shoulder and elbow and makes a fixed threshold unsafe.
//   * FRICTION -- the Coulomb floor near qdot ~ 0. Open point 3. F_fric is
//                 identically zero in touch_dynamics terms today, which is why
//                 tau_dis_hat sits at a nonzero floor at rest. This measures
//                 that floor so threshold_nm can be set above it.
//
// The recommended empirical route for both, replacing the fabricated
// analytical gravity coefficients, which are not used anywhere.
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
static const double kMaxForceN = 0.60;
static const double kForceStepN = 0.004;
static const int    kStepUs = 20000;         // 20 ms per step -> ~3.0 s to cap

// Breakaway: joint deviation from the held pose, radians. Above encoder noise.
static const double kBreakawayRad = 0.015;

// Self-hold verification before each ramp.
static const double kHoldDriftRad = 0.010;
static const int    kHoldCheckMs = 800;

// The +/- pair must share a pose for the g/c split to mean anything.
static const double kPairPoseTolRad = 0.05;

static const int kMaxPoses = 16;
static const int kNumDirs = 6;   // +x -x +y -y +z -z

static const char* kDirName[kNumDirs] = {"+X", "-X", "+Y", "-Y", "+Z", "-Z"};
static const int   kDirAxis[kNumDirs] = {0, 0, 1, 1, 2, 2};
static const double kDirSign[kNumDirs] = {+1.0, -1.0, +1.0, -1.0, +1.0, -1.0};

// ---------------------------------------------------------------------------
// OPTIONAL CSV LOG  (--log <path>)
// ---------------------------------------------------------------------------
// Strictly additive: no safety behaviour depends on this, and nothing here runs
// inside servoLoop. All writes happen from the main thread between ramps, so a
// slow or full disk can never stall a servo frame or delay a force zeroing.
// If the file cannot be opened the tool says so and continues WITHOUT logging,
// because losing a log is not a reason to abandon a hands-on hardware session.
static FILE* g_log = nullptr;

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
            "tool,unix_time,pose_id,dir,broke,break_force_n,"
            "q0_1,q0_2,q0_3,qbreak_1,qbreak_2,qbreak_3,"
            "tau_nm_1,tau_nm_2,tau_nm_3,"
            "raw_lastjt_1,raw_lastjt_2,raw_lastjt_3,"
            "slipped_joint,max_dq_rad,"
            "bias_1,bias_2,bias_3\n");
    }
    fflush(g_log);
}

static void logClose() {
    if (g_log) { fflush(g_log); fclose(g_log); g_log = nullptr; }
}

// One row per ramp attempt, including non-breakaways (broke=0). A direction that
// refused to break is real data -- it bounds the breakaway torque from below --
// so it is recorded rather than dropped.
static void logRow(int poseId, const char* dirName, bool broke, double breakForceN,
                   const double q0[3], const double qBreak[3],
                   const double tauNm[3], const double rawJT[3],
                   int slippedJoint, double maxDq, const double bias[3]) {
    if (!g_log) return;
    static const char* jn[3] = {"waist", "shoulder", "elbow"};
    // tau_nm here is J^T applied to the RAMP force only, excluding the bias.
    // The bias is logged separately so the offline fit can subtract J^T(q0)*B
    // from the gravity estimate. Keeping them apart means an old log without a
    // bias column and a new one with bias=0 mean exactly the same thing.
    fprintf(g_log,
        "static_sweep,%ld,%d,%s,%d,%.6f,"
        "%.9f,%.9f,%.9f,%.9f,%.9f,%.9f,"
        "%.9f,%.9f,%.9f,"
        "%.9f,%.9f,%.9f,"
        "%s,%.9f,"
        "%.9f,%.9f,%.9f\n",
        (long)time(nullptr), poseId, dirName, broke ? 1 : 0, breakForceN,
        q0[0], q0[1], q0[2], qBreak[0], qBreak[1], qBreak[2],
        tauNm[0], tauNm[1], tauNm[2],
        rawJT[0], rawJT[1], rawJT[2],
        (broke && slippedJoint >= 0) ? jn[slippedJoint] : "-", maxDq,
        bias[0], bias[1], bias[2]);
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
static bool autoHold(Ctx& c, touch_haptics::Vec3& biasOut) {
    for (int i = 0; i < 3; ++i) c.xTarget[i] = c.pos_m[i];

    printf("    target captured. RELEASE THE ARM NOW.\n");
    fflush(stdout);
    c.holdMode = 1;

    // Let the operator let go and the transient settle.
    for (int k = 0; k < 200 && !g_abort; ++k) usleep(10000);   // 2.0 s
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

struct DirResult {
    bool broke = false;
    double breakForceN = 0.0;
    touch_haptics::Vec3 q0{};          // pose held before the ramp
    touch_haptics::Vec3 qBreak{};      // pose at breakaway
    touch_haptics::Vec3 tauNm{};       // J^T(q0) * F at breakaway
    touch_haptics::Vec3 rawLastJT{};   // HD_LAST_JOINT_TORQUE at breakaway
};

// Verify the arm holds the pose on its own. If it drifts, the +/- decomposition
// is meaningless at this pose -- there is no static equilibrium to break.
static bool verifySelfHold(Ctx& c, touch_haptics::Vec3& q0) {
    readQ(c, q0);
    for (int i = 0; i < kHoldCheckMs; ++i) {
        if (g_abort) return false;
        usleep(1000);
        touch_haptics::Vec3 q;
        readQ(c, q);
        for (int j = 0; j < 3; ++j) {
            if (std::fabs(q[j] - q0[j]) > kHoldDriftRad) return false;
        }
    }
    return true;
}

static DirResult rampDirection(Ctx& c, int dir) {
    DirResult r;

    if (!verifySelfHold(c, r.q0)) {
        printf("    pose does not hold on its own (drift > %.3f rad) -- skipped.\n",
               kHoldDriftRad);
        zeroForce(c);
        return r;
    }

    const int axis = kDirAxis[dir];
    const double sign = kDirSign[dir];

    for (double f = 0.0; f <= kMaxForceN + 1e-12; f += kForceStepN) {
        if (g_abort) { zeroForce(c); printf("    ABORTED mid-ramp at %.3f N -- force zeroed.\n", f); return r; }
        zeroForce(c);
        c.cmd[axis] = sign * f;
        usleep(kStepUs);

        touch_haptics::Vec3 q;
        readQ(c, q);
        double maxDev = 0.0;
        for (int j = 0; j < 3; ++j) {
            const double d = std::fabs(q[j] - r.q0[j]);
            if (d > maxDev) maxDev = d;
        }
        if (maxDev > kBreakawayRad) {
            r.broke = true;
            r.breakForceN = f;
            r.qBreak = q;
            for (int j = 0; j < 3; ++j) r.rawLastJT[j] = c.lastJointTorque[j];

            // tau = J^T(q0) * F. q0, not qBreak: we want the torque in the
            // configuration that was resisting, before it slipped.
            touch_haptics::Vec3 F{0.0, 0.0, 0.0};
            F[axis] = sign * f;
            r.tauNm = touch_haptics::jacobianTransposeTimes(
                touch_haptics::jacobian(r.q0), F);
            break;
        }
    }

    zeroForce(c);
    usleep(150000);   // let it settle before the next prompt
    return r;
}

int main(int argc, char** argv) {
    // Additive arg parsing: the first non-flag argument is still the device
    // name, so every existing invocation keeps working unchanged.
    const char* deviceName = HD_DEFAULT_DEVICE;
    const char* logPath = nullptr;
    const char* statusPath = nullptr;
    double biasN = 0.0;     // support-force magnitude, N (manual mode only)
    int    biasAxis = 1;    // 1 = y = up on a standard Touch
    bool   autoHoldEnabled = true;   // find the holding force per pose
    bool haveDevice = false;
    for (int i = 1; i < argc; ++i) {
        if (std::strcmp(argv[i], "--log") == 0 && i + 1 < argc) {
            logPath = argv[++i];
        } else if (std::strncmp(argv[i], "--log=", 6) == 0) {
            logPath = argv[i] + 6;
        } else if (std::strcmp(argv[i], "--status") == 0 && i + 1 < argc) {
            statusPath = argv[++i];
        } else if (std::strncmp(argv[i], "--status=", 9) == 0) {
            statusPath = argv[i] + 9;
        } else if (std::strcmp(argv[i], "--bias-n") == 0 && i + 1 < argc) {
            biasN = atof(argv[++i]);
        } else if (std::strcmp(argv[i], "--bias-axis") == 0 && i + 1 < argc) {
            biasAxis = atoi(argv[++i]);
        } else if (std::strcmp(argv[i], "--auto-hold") == 0) {
            autoHoldEnabled = true;
        } else if (std::strcmp(argv[i], "--no-auto-hold") == 0) {
            autoHoldEnabled = false;
        } else if (!haveDevice) {
            deviceName = argv[i];
            haveDevice = true;
        }
    }

    printf("=== Static-pose sweep: gravity map + friction floor ===\n");
    printf("device: %s\n", haveDevice ? deviceName : "(default)");
    printf("ramp cap %.2f N (continuous rating 0.88 N), step %.3f N, breakaway %.3f rad\n",
           kMaxForceN, kForceStepN, kBreakawayRad);
    printf("L1 = %.4f m  L2 = %.4f m\n",
           touch_haptics::kL1Meters, touch_haptics::kL2Meters);
    printf("\nDO NOT GRIP THE STYLUS during a ramp -- position it, release, then\n"
           "let the ramp run. Gripping measures your arm, not the device.\n");
    printf("Run hdapi_fk_residual first: the N*m columns depend on J(q).\n");
    printf("\nABORT: Ctrl-C at any time. The servo loop zeroes force within ~1 ms,\n"
           "then the tool unwinds and disables force output. Safe to use mid-ramp.\n\n");

    logOpen(logPath);
    if (g_log) printf("logging rows to: %s\n\n", logPath);

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

    static DirResult results[kMaxPoses][kNumDirs];
    int nPose = 0;

    while (nPose < kMaxPoses && !g_abort) {
        if (autoHoldEnabled) {
            printf("\n--- pose %d --- HOLD the arm where you want it, clear of the table,\n",
                   nPose + 1);
            printf("                then Enter to auto-find the holding force. 'q' finishes: ");
        } else {
            printf("\n--- pose %d --- Enter to begin", nPose + 1);
            printf(" | number = set support bias N (now %.3f)", ctx.bias[biasAxis]);
            printf(" | 'q' to finish: ");
        }
        fflush(stdout);
        char line[128];
        if (!fgets(line, sizeof(line), stdin)) break;
        if (g_abort) break;
        if (line[0] == 'q' || line[0] == 'Q') break;

        if (!autoHoldEnabled) {
            // Manual fallback: a bare number re-tunes the support bias and
            // re-prompts the same pose.
            char* endp = nullptr;
            const double typed = strtod(line, &endp);
            if (endp != line && endp != nullptr) {
                applyBias(typed);
                printf("  bias now %.3f N on %c. Let the arm settle, check STEADY, then Enter.\n",
                       ctx.bias[biasAxis], "xyz"[biasAxis]);
                continue;               // same pose number, re-prompt
            }
        } else {
            touch_haptics::Vec3 found{};
            if (!autoHold(ctx, found)) {
                if (g_abort) break;
                // Release the failed hold rather than leaving a wrong constant
                // pushing on an arm the operator is about to reposition.
                ctx.holdMode = 0;
                ctx.bias[0] = ctx.bias[1] = ctx.bias[2] = 0.0;
                printf("    pose skipped, bias released. Reposition and try again.\n");
                continue;               // same pose number, re-prompt
            }
        }

        for (int d = 0; d < kNumDirs && !g_abort; ++d) {
            // Deliberately NOT "restore the pose": with the hold frozen the arm
            // stays put on its own, and reaching in to adjust it between ramps
            // is what destroyed the +/- pairing on 2026-08-24 (q2 moved 1.5 rad
            // within a single pose). Hands off once the hold is verified.
            printf("  [%s] hands OFF. Enter to ramp: ", kDirName[d]);
            fflush(stdout);
            int c2 = getchar();
            if (g_abort) break;
            while (c2 != '\n' && c2 != EOF) c2 = getchar();
            if (c2 == EOF) break;

            DirResult r = rampDirection(ctx, d);
            results[nPose][d] = r;

            int mj = -1;
            double mv = 0.0;
            for (int j = 0; j < 3; ++j) {
                const double dv = std::fabs(r.qBreak[j] - r.q0[j]);
                if (dv > mv) { mv = dv; mj = j; }
            }

            if (r.broke) {
                static const char* jn[3] = {"waist", "shoulder", "elbow"};
                printf("    broke at %.3f N   tau=[%+8.5f %+8.5f %+8.5f] N*m   "
                       "slipped: %s (dq=%.4f rad)\n",
                       r.breakForceN, r.tauNm[0], r.tauNm[1], r.tauNm[2],
                       jn[mj < 0 ? 0 : mj], mv);
            } else if (r.q0[0] != 0.0 || r.q0[1] != 0.0 || r.q0[2] != 0.0) {
                printf("    NO BREAKAWAY by %.2f N cap -- excluded.\n", kMaxForceN);
            }

            const double biasSnapshot[3] = {ctx.bias[0], ctx.bias[1], ctx.bias[2]};
            logRow(nPose + 1, kDirName[d], r.broke, r.breakForceN,
                   r.q0.data(), r.qBreak.data(), r.tauNm.data(), r.rawLastJT.data(),
                   mj, mv, biasSnapshot);
        }
        ++nPose;
    }

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

    if (nPose == 0) {
        printf("No poses recorded.\n");
        logClose();
        return 1;
    }

    // -----------------------------------------------------------------
    // Per-pose decomposition.
    // -----------------------------------------------------------------
    static const char* jn[3] = {"waist   ", "shoulder", "elbow   "};

    for (int p = 0; p < nPose; ++p) {
        printf("\n=== pose %d ===\n", p + 1);

        const DirResult* rr = results[p];
        int nvalid = 0;
        for (int d = 0; d < kNumDirs; ++d) if (rr[d].broke) ++nvalid;
        printf("valid directions: %d of %d\n", nvalid, kNumDirs);
        if (nvalid == 0) { printf("  nothing usable at this pose.\n"); continue; }

        printf("\n%-4s %9s  %-32s %-32s\n", "dir", "F_break", "tau = J^T F [N*m]",
               "raw HD_LAST_JOINT_TORQUE");
        for (int d = 0; d < kNumDirs; ++d) {
            if (!rr[d].broke) { printf("%-4s %9s  (no breakaway)\n", kDirName[d], "-"); continue; }
            printf("%-4s %9.3f  [%+9.5f %+9.5f %+9.5f] [%+9.4f %+9.4f %+9.4f]\n",
                   kDirName[d], rr[d].breakForceN,
                   rr[d].tauNm[0], rr[d].tauNm[1], rr[d].tauNm[2],
                   rr[d].rawLastJT[0], rr[d].rawLastJT[1], rr[d].rawLastJT[2]);
        }

        // Three independent (g, c) estimates, one per axis pair.
        printf("\ngravity / friction split per axis pair:\n");
        printf("%-6s %-8s %-30s %-30s\n", "pair", "joint", "g = -(t+ + t-)/2 [N*m]",
               "c = (t+ - t-)/2 [N*m]");
        double gsum[3] = {0,0,0};
        int gcount = 0;
        double gvals[3][3];
        bool gok[3] = {false,false,false};

        for (int a = 0; a < 3; ++a) {
            const DirResult& rp = rr[2*a];
            const DirResult& rm = rr[2*a + 1];
            if (!rp.broke || !rm.broke) {
                printf("%-6s incomplete pair -- skipped\n", kDirName[2*a]);
                continue;
            }
            double poseDelta = 0.0;
            for (int j = 0; j < 3; ++j) {
                const double dv = std::fabs(rp.q0[j] - rm.q0[j]);
                if (dv > poseDelta) poseDelta = dv;
            }
            const char* warn = (poseDelta > kPairPoseTolRad) ? "  <-- POSE DRIFTED, pair unreliable" : "";

            for (int j = 0; j < 3; ++j) {
                const double g = -(rp.tauNm[j] + rm.tauNm[j]) / 2.0;
                const double c = (rp.tauNm[j] - rm.tauNm[j]) / 2.0;
                gvals[a][j] = g;
                printf("%-6s %-8s %+30.5f %+30.5f%s\n",
                       (j == 0) ? (a == 0 ? "X" : a == 1 ? "Y" : "Z") : "",
                       jn[j], g, c, (j == 0) ? warn : "");
                if (c < 0.0 && j == 0) {
                    // c < 0 is non-physical: it means the - direction needed more
                    // torque than the + direction in the same sense.
                    printf("       NOTE: negative Coulomb estimate -- the pose was not a\n"
                           "       true static equilibrium, or another joint slipped first.\n");
                }
            }
            if (poseDelta <= kPairPoseTolRad) {
                gok[a] = true;
                for (int j = 0; j < 3; ++j) gsum[j] += gvals[a][j];
                ++gcount;
            }
        }

        // Consistency: the axis pairs should agree on gravity at a given pose.
        if (gcount >= 2) {
            printf("\ngravity consistency across %d axis pair(s):\n", gcount);
            for (int j = 0; j < 3; ++j) {
                double mn = 1e9, mx = -1e9;
                for (int a = 0; a < 3; ++a) {
                    if (!gok[a]) continue;
                    if (gvals[a][j] < mn) mn = gvals[a][j];
                    if (gvals[a][j] > mx) mx = gvals[a][j];
                }
                printf("  %-8s mean %+9.5f   spread %9.5f N*m%s\n",
                       jn[j], gsum[j] / gcount, mx - mn,
                       (mx - mn) > 0.02 ? "   <-- wide, decomposition suspect here" : "");
            }
        } else {
            printf("\nfewer than 2 usable axis pairs -- no consistency check possible.\n");
        }

        // Independent scale-factor cross-check vs hdapi_torque_calib.cpp.
        printf("\nindependent HD_LAST_JOINT_TORQUE scale estimate (N*m per raw unit):\n");
        for (int d = 0; d < kNumDirs; ++d) {
            if (!rr[d].broke) continue;
            printf("  %-4s", kDirName[d]);
            for (int j = 0; j < 3; ++j) {
                if (std::fabs(rr[d].rawLastJT[j]) > 1e-6) {
                    printf("  j%d=%.6g", j + 1, rr[d].tauNm[j] / rr[d].rawLastJT[j]);
                } else {
                    printf("  j%d=-", j + 1);
                }
            }
            printf("\n");
        }
        printf("  Compare against hdapi_torque_calib.cpp. These are different data;\n"
               "  if they disagree, neither is trustworthy yet.\n");
    }

    printf("\n=== How to use this output ===\n");
    printf("  * GRAVITY (open point 4): tabulate mean g against q across poses.\n"
           "    That table IS the empirical G_n(q), no analytical formula needed.\n"
           "    Discard any pose flagged wide-spread or pose-drifted.\n");
    printf("  * FRICTION FLOOR (open point 3): the c column is the Coulomb floor\n"
           "    that tau_dis_hat will sit at near qdot ~ 0. threshold_nm must be\n"
           "    set above it -- and above the gravity term too, unless G_n(q) is\n"
           "    subtracted in the nominal model first.\n");
    printf("  * Neither value is written anywhere by this tool. M_n, gravity, and\n"
           "    the commissioning flags remain untouched and unset.\n");
    if (g_log) printf("\nlog written: %s\n", logPath);
    logClose();
    printf("\n=== Done. ===\n");
    return 0;
}
