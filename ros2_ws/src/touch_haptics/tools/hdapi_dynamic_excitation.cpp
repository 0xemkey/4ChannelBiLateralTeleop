// hdapi_dynamic_excitation.cpp
//
// STANDALONE bounded dynamic excitation: data for nominal-inertia (M_n) fitting.
//
// WHY THIS TOOL EXISTS
// ---------------------------------------------------------------------------
// hdapi_static_sweep measures breakaway at static equilibrium. That isolates
// gravity and the Coulomb friction floor, but it carries no inertial
// information at all: at breakaway qddot ~ 0, so M_n never enters the balance.
// Identifying M_n needs the arm actually accelerating under a force we know
// exactly, sampled fast enough to recover qddot. That is what this does.
//
// The regression this feeds (done offline, in Python) is, per joint:
//
//     tau_cmd(t) = M_n * qddot(t) + G(q) + F_fric(qdot) + residual
//
// with G(q) taken from the static sweep's gravity map and F_fric from its
// Coulomb/viscous estimates, both subtracted before the least-squares fit for
// M_n. This tool supplies tau_cmd, q, and t; Python supplies qdot and qddot by
// non-causal filtering, which is strictly better conditioned than anything the
// real-time observer can do -- see the Python suite's note on that.
//
// ---------------------------------------------------------------------------
// EXCITATION DESIGN
// ---------------------------------------------------------------------------
// A linear chirp, not a fixed sinusoid. A single frequency confounds inertia
// with stiffness and damping -- they all produce force in phase quadrature at
// one frequency and cannot be separated from one measurement. Sweeping a band
// gives the regression independent equations at different qddot/qdot ratios,
// which is what makes M_n identifiable rather than merely fitted.
//
// Amplitude is shaped by a raised-cosine envelope over the first and last 15%
// of the run. A force step at t=0 would be both a jolt to the hardware and a
// broadband transient that pollutes the very band being swept.
//
// Frequencies are deliberately low (default 0.5 -> 6 Hz). Above ~10 Hz the
// Omni's structural compliance and cable stretch start to dominate, and the
// rigid-body model this regression assumes stops being true.
//
// ---------------------------------------------------------------------------
// SAFETY  -- same discipline as hdapi_static_sweep.cpp, plus one addition
// ---------------------------------------------------------------------------
//   * Chirp amplitude capped at 0.50 N; support bias capped at 0.55 N; the two
//     together hard-clamped at 1.00 N in the servo loop. Device continuous
//     rating 0.88 N, peak 3.3 N. Raised from a flat 0.30 N on 2026-08-24 with
//     operator authorisation, in step with hdapi_static_sweep.cpp.
//   * The amplitude is REDUCED automatically to fit under the total ceiling
//     once the support bias is known, rather than being allowed to clip.
//     Clipping a chirp distorts it, and the harmonics that creates would be
//     identified as inertia -- a biased result, not merely a noisy one.
//   * NO THERMAL FEEDBACK EXISTS. HD_MOTOR_TEMPERATURE reads flat ~1e-10 on
//     this unit through 12 s of sustained 0.5 N, so it is neither sensed nor
//     modelled. Unlike the sweep's brief ramps, this tool oscillates for
//     seconds at a time; keep sessions short and let the motors rest.
//   * SIGINT/SIGTERM set a flag only. servoLoop checks it BEFORE the force
//     write, so an aborted frame can never emit a non-zero force. Zeroing
//     happens within one servo frame (~1 ms), ahead of any userspace teardown.
//   * ADDITIONAL to the sweep: a hard magnitude clamp is applied to the
//     computed force immediately before every hdSetDoublev, independent of the
//     chirp maths. Defence in depth -- an arithmetic bug in the envelope or a
//     bad --amp cannot exceed the ceiling, because the clamp is downstream of
//     all of it.
//   * Force zeroed and output disabled on every exit path, including aborts
//     and the scheduler-never-ran path.
//   * NO disk I/O inside servoLoop. Samples land in a preallocated buffer;
//     the CSV is written from the main thread only after force is zeroed.
//     Blocking I/O at HD_MAX_SCHEDULER_PRIORITY could stall a servo frame,
//     which is the one thing that must never be delayed while force is live.
//   * DO NOT GRIP THE STYLUS during a capture. Your hand's impedance is large
//     compared with the arm's and would be what gets identified. Position the
//     arm, release it, then start the capture.
//
// BUILD (from this tools/ directory):
//   g++ -std=gnu++17 -I../include hdapi_dynamic_excitation.cpp \
//       -o hdapi_dynamic_excitation -lHD -lHDU -lrt -lpthread
//
// RUN:
//   ./hdapi_dynamic_excitation "Left" --log dyn.csv
//   ./hdapi_dynamic_excitation "Left" --log dyn.csv --amp 0.20 --dur 6 \
//        --f0 0.5 --f1 5.0 --axes xyz
//
// Per pose the operator confirms, then each selected axis is chirped in turn.

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
// ABORT PATH -- identical contract to hdapi_static_sweep.cpp
// ---------------------------------------------------------------------------
static volatile sig_atomic_t g_abort = 0;
static void onSigint(int) { g_abort = 1; }

// Safety ceiling on the CHIRP amplitude alone. Device nominal continuous is
// 0.88 N. Raised 0.30 -> 0.50 on 2026-08-24 alongside the sweep, but see
// kMaxTotalN: the amplitude actually usable is whatever is left after the
// support bias, and the tool reduces it automatically rather than clipping.
static const double kMaxForceN = 0.50;

// Support bias, found automatically per pose. Same rationale as the sweep:
// this arm only self-holds where gravity torque is nearly zero, so the poses
// worth exciting are exactly the ones that will not stay put unaided.
static const double kMaxBiasN = 0.55;

// Hard ceiling on bias + chirp together, enforced in the servo loop.
//
// WHY CLIPPING MATTERS MORE HERE THAN IN THE SWEEP: the sweep's ramp is
// monotonic, so a clamp merely truncates it and the breakaway is either found
// or reported as "no breakaway" -- honest either way. A chirp that hits the
// clamp is DISTORTED: flattened peaks are no longer sinusoidal, which injects
// harmonics at frequencies the sweep never commanded, and the regression then
// attributes that energy to inertia and damping. A clipped capture is not
// merely noisier, it is biased. So the amplitude is reduced up front to fit
// under this ceiling, and any clipping that still occurs is counted and the
// capture flagged rather than silently used.
static const double kMaxTotalN = 1.00;

// Auto-hold controller, identical to hdapi_static_sweep.cpp.
static const double kHoldKp = 60.0;      // N/m
static const double kHoldKd = 0.4;       // N*s/m
static const double kHoldSettleErrM = 0.004;
static const double kHoldSettleVelMps = 0.006;
static const double kHoldDriftRad = 0.010;

// Excitation defaults.
static const double kDefaultAmpN = 0.25;
static const double kDefaultDurS = 8.0;
static const double kDefaultF0Hz = 0.5;
static const double kDefaultF1Hz = 6.0;
static const double kEnvelopeFrac = 0.15;   // raised-cosine fade in/out

// Capture buffer. 1 kHz servo * 20 s headroom. Preallocated, never grown.
static const int kMaxSamples = 20000;
static const int kMaxPoses = 16;

struct Sample {
    double t_s;
    double cmd[3];
    double q[3];
    double lastJT[3];
    double pos[3];
};

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

static double nowMonotonic() {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (double)ts.tv_sec + (double)ts.tv_nsec * 1e-9;
}

struct Ctx {
    HHD hHD;
    volatile double q[3];
    volatile double lastJointTorque[3];
    volatile unsigned long frames;

    // Support bias and auto-hold, ported from hdapi_static_sweep.cpp. The bias
    // is frozen before a capture starts and held constant throughout it, so the
    // chirp oscillates about a fixed offset. A live controller would fight the
    // excitation and its reaction would land in the identified dynamics.
    volatile double bias[3];
    volatile int holdMode;
    volatile double xTarget[3];
    volatile double pos_m[3];
    volatile double vel_mps[3];
    volatile unsigned long clipped;   // frames where bias+chirp hit kMaxTotalN

    // Excitation state, written by main thread before a capture starts and
    // read by the servo loop. `capturing` is the last field set, so the loop
    // never sees a half-configured excitation.
    volatile int    capturing;
    volatile int    axis;
    volatile double amp;
    volatile double durS;
    volatile double f0;
    volatile double f1;
    volatile double tStart;

    // Capture buffer. Written only by the servo loop while capturing, read by
    // the main thread only after capturing has been cleared.
    Sample* buf;
    volatile int nSamples;
    volatile int overflow;
};

static HDerror drainLast() {
    HDerror last = HD_SUCCESS;
    HDErrorInfo e;
    while (HD_DEVICE_ERROR(e = hdGetError())) last = e.errorCode;
    return last;
}

// Raised-cosine envelope: 0 -> 1 over the first kEnvelopeFrac of the run,
// 1 -> 0 over the last. Keeps the force continuous at both ends.
static double envelope(double u) {
    if (u <= 0.0 || u >= 1.0) return 0.0;
    if (u < kEnvelopeFrac)        return 0.5 * (1.0 - std::cos(M_PI * u / kEnvelopeFrac));
    if (u > 1.0 - kEnvelopeFrac)  return 0.5 * (1.0 - std::cos(M_PI * (1.0 - u) / kEnvelopeFrac));
    return 1.0;
}

static HDCallbackCode HDCALLBACK servoLoop(void* data) {
    Ctx* c = static_cast<Ctx*>(data);

    hdBeginFrame(c->hHD);

    double f[3] = {0.0, 0.0, 0.0};
    double tRel = 0.0;
    const int capturing = c->capturing;

    if (capturing && !g_abort) {
        const double t = nowMonotonic();
        tRel = t - c->tStart;
        const double dur = c->durS;
        if (tRel >= 0.0 && tRel <= dur) {
            const double u = tRel / dur;
            // Linear chirp: instantaneous frequency f0 + (f1-f0)*u. The phase
            // is its integral, which is why the u^2 term appears -- using
            // sin(2*pi*f(t)*t) instead is a classic error and produces a
            // different, wrong sweep.
            const double phase = 2.0 * M_PI * (c->f0 * tRel +
                                 0.5 * (c->f1 - c->f0) * tRel * tRel / dur);
            f[c->axis] = c->amp * envelope(u) * std::sin(phase);
        }
    }

    // Tip position/velocity for the auto-hold PD and the status display.
    HDdouble p_mm[3], v_mm_s[3];
    hdGetDoublev(HD_CURRENT_POSITION, p_mm);
    drainLast();
    hdGetDoublev(HD_CURRENT_VELOCITY, v_mm_s);
    drainLast();
    for (int i = 0; i < 3; ++i) {
        c->pos_m[i] = p_mm[i] * 0.001;
        c->vel_mps[i] = v_mm_s[i] * 0.001;
    }

    // AUTO-HOLD: closed loop only while searching. Never runs during a capture.
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

    // Chirp rides on top of the frozen support bias.
    for (int i = 0; i < 3; ++i) f[i] += c->bias[i];

    // Abort wins over any commanded force, checked before the write so an
    // aborted frame can never emit a non-zero force. The bias goes too: an
    // abort must leave the device fully passive, not merely un-excited.
    if (g_abort) { f[0] = f[1] = f[2] = 0.0; }

    // Hard ceiling, downstream of all excitation maths. Defence in depth: no
    // arithmetic error above can push past this. Clipping is COUNTED because a
    // clipped chirp is distorted, not just attenuated -- see kMaxTotalN.
    const double mag = std::sqrt(f[0]*f[0] + f[1]*f[1] + f[2]*f[2]);
    if (mag > kMaxTotalN && mag > 1e-12) {
        const double s = kMaxTotalN / mag;
        f[0] *= s; f[1] *= s; f[2] *= s;
        if (capturing) c->clipped++;
    }

    HDdouble hf[3] = { f[0], f[1], f[2] };
    hdSetDoublev(HD_CURRENT_FORCE, hf);

    HDdouble q[3], t3[3], p3[3];
    hdGetDoublev(HD_CURRENT_JOINT_ANGLES, q);
    drainLast();
    // HD_LAST_JOINT_TORQUE lags one servo frame (1 ms). At the default 6 Hz
    // top frequency that is ~2 degrees of phase; the Python side shifts by one
    // sample to remove it rather than ignoring it.
    hdGetDoublev(HD_LAST_JOINT_TORQUE, t3);
    drainLast();
    hdGetDoublev(HD_CURRENT_POSITION, p3);
    drainLast();

    for (int i = 0; i < 3; ++i) { c->q[i] = q[i]; c->lastJointTorque[i] = t3[i]; }

    // Buffer write only -- no I/O, no allocation, no locks.
    if (capturing && !g_abort) {
        const int n = c->nSamples;
        if (n < kMaxSamples) {
            Sample& s = c->buf[n];
            s.t_s = tRel;
            for (int i = 0; i < 3; ++i) {
                s.cmd[i] = f[i];
                s.q[i] = q[i];
                s.lastJT[i] = t3[i];
                s.pos[i] = p3[i];
            }
            c->nSamples = n + 1;
        } else {
            c->overflow = 1;
        }
    }

    hdEndFrame(c->hHD);

    c->frames++;
    return HD_CALLBACK_CONTINUE;
}

static void zeroForce(Ctx& c) {
    c.capturing = 0;
    c.holdMode = 0;
    c.bias[0] = c.bias[1] = c.bias[2] = 0.0;
}

// ---------------------------------------------------------------------------
// LIVE STATUS  (--status <path>)
// Same file format as hdapi_static_sweep.cpp, so watch_sweep_status.sh works
// for both tools unchanged. Never runs inside servoLoop -- file I/O at
// HD_MAX_SCHEDULER_PRIORITY could stall a frame while force is live.
// ---------------------------------------------------------------------------
static std::atomic<bool> g_statusRun{false};
static std::thread g_statusThread;

static void statusLoop(Ctx* c, std::string path) {
    double ref[3] = {c->q[0], c->q[1], c->q[2]};
    int settle = 0;
    while (g_statusRun.load()) {
        double q[3] = {c->q[0], c->q[1], c->q[2]};
        double drift = 0.0;
        for (int j = 0; j < 3; ++j) drift = std::max(drift, std::fabs(q[j] - ref[j]));
        if (drift < 0.002) { if (++settle > 15) { for (int j = 0; j < 3; ++j) ref[j] = q[j]; settle = 0; } }
        else settle = 0;

        const touch_haptics::Vec3 qq{q[0], q[1], q[2]};
        const touch_haptics::SingularityMetrics sm = touch_haptics::singularityMetrics(qq);

        std::string tmp = path + ".tmp";
        if (FILE* f = fopen(tmp.c_str(), "w")) {
            fprintf(f, "  LIVE - dynamic excitation  (10 Hz)\n\n");
            fprintf(f, "  joint angles   q1 waist    %+8.4f rad  %+7.1f deg\n", q[0], q[0]*180.0/M_PI);
            fprintf(f, "                 q2 shoulder %+8.4f rad  %+7.1f deg\n", q[1], q[1]*180.0/M_PI);
            fprintf(f, "                 q3 elbow    %+8.4f rad  %+7.1f deg\n", q[2], q[2]*180.0/M_PI);
            fprintf(f, "\n  tip position   x %+8.4f   y %+8.4f   z %+8.4f  m\n",
                    (double)c->pos_m[0], (double)c->pos_m[1], (double)c->pos_m[2]);
            fprintf(f, "\n  reach r        %7.4f m      cos(q2-q3) %+6.3f%s\n",
                    sm.radial_r, sm.align_cos, sm.near_singular ? "   <-- NEAR SINGULAR" : "");
            fprintf(f, "\n  drift          %7.4f rad   %s\n", drift,
                    drift < 0.010 ? "STEADY" : "MOVING - let it settle");
            fprintf(f, "\n  support bias   %+6.3f %+6.3f %+6.3f  N%s\n",
                    (double)c->bias[0], (double)c->bias[1], (double)c->bias[2],
                    c->holdMode ? "  (SEARCHING)" : "  (frozen)");
            fprintf(f, "  capturing      %s   clipped frames %lu%s\n",
                    c->capturing ? "YES" : "no", c->clipped,
                    c->clipped ? "   <-- CHIRP DISTORTED" : "");
            fclose(f);
            rename(tmp.c_str(), path.c_str());
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
// AUTO-HOLD -- ported from hdapi_static_sweep.cpp, same contract.
// Find the constant force that keeps the arm at the pose, then FREEZE it. The
// freeze is what makes it usable: during a capture the support force must be a
// known constant, or the controller's reaction to the chirp would be
// identified as part of the arm's dynamics.
// ---------------------------------------------------------------------------
static bool autoHold(Ctx& c, double biasOut[3]) {
    for (int i = 0; i < 3; ++i) c.xTarget[i] = c.pos_m[i];

    printf("    target captured. RELEASE THE ARM NOW.\n");
    fflush(stdout);
    c.holdMode = 1;

    for (int k = 0; k < 200 && !g_abort; ++k) usleep(10000);   // 2.0 s to release
    if (g_abort) { c.holdMode = 0; return false; }

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

    double mag = 0.0, sag = 0.0;
    for (int i = 0; i < 3; ++i) {
        acc[i] /= n;
        mag += acc[i] * acc[i];
        const double d = c.xTarget[i] - c.pos_m[i];
        sag += d * d;
    }
    mag = std::sqrt(mag);
    sag = std::sqrt(sag);

    c.holdMode = 0;          // cleared first: exactly one writer of bias[]
    usleep(5000);
    for (int i = 0; i < 3; ++i) { c.bias[i] = acc[i]; biasOut[i] = acc[i]; }

    printf("    holding force: [%+.3f %+.3f %+.3f] N  |F| = %.3f N", acc[0], acc[1], acc[2], mag);
    if (mag >= kMaxBiasN - 1e-6) printf("   <-- AT THE %.2f N CAP", kMaxBiasN);
    printf("\n    settled %.1f mm below where you held it (expected |F|/Kp = %.1f mm)\n",
           sag * 1000.0, mag / kHoldKp * 1000.0);
    if (unsettled > n / 4) {
        printf("    NOTE: wandered up to %.1f mm while averaging; frozen value less reliable.\n",
               maxWander * 1000.0);
    }

    // Open-loop verification.
    double q0[3] = {c.q[0], c.q[1], c.q[2]};
    double worst = 0.0;
    for (int k = 0; k < 100 && !g_abort; ++k) {
        usleep(10000);
        for (int j = 0; j < 3; ++j) worst = std::max(worst, std::fabs(c.q[j] - q0[j]));
    }
    if (g_abort) return false;

    if (worst > kHoldDriftRad) {
        printf("    FROZEN HOLD FAILED: drifted %.4f rad open-loop (limit %.3f).\n"
               "    Needs more than the %.2f N cap, or it is resting on something.\n",
               worst, kHoldDriftRad, kMaxBiasN);
        return false;
    }
    printf("    hold verified: %.4f rad drift over 1 s.\n", worst);
    return true;
}

// ---------------------------------------------------------------------------
// CSV LOG -- main thread only, after force is zeroed.
// ---------------------------------------------------------------------------
static FILE* g_log = nullptr;

static void logOpen(const char* path) {
    if (!path) return;
    g_log = fopen(path, "a");
    if (!g_log) {
        printf("WARNING: could not open log '%s' -- continuing without logging.\n", path);
        return;
    }
    fseek(g_log, 0, SEEK_END);
    if (ftell(g_log) == 0) {
        fprintf(g_log,
            "tool,unix_time,pose_id,axis,amp_n,f0_hz,f1_hz,dur_s,"
            "sample_idx,t_s,cmd_1,cmd_2,cmd_3,q_1,q_2,q_3,"
            "raw_lastjt_1,raw_lastjt_2,raw_lastjt_3,pos_x_mm,pos_y_mm,pos_z_mm,"
            "bias_1,bias_2,bias_3,clipped\n");
    }
    fflush(g_log);
}

static void logClose() {
    if (g_log) { fflush(g_log); fclose(g_log); g_log = nullptr; }
}

static void dumpCapture(Ctx& c, int poseId, const char* axisName) {
    if (!g_log) return;
    const long ts = (long)time(nullptr);
    const int n = c.nSamples;
    for (int i = 0; i < n; ++i) {
        const Sample& s = c.buf[i];
        // cmd_* is the TOTAL commanded force (support bias + chirp), post-clamp.
        // That is deliberate and is what the regression needs: the whole
        // commanded force is what accelerates the arm, so tau = J^T @ cmd is the
        // full driving torque. The bias is logged separately as well, purely so
        // its contribution can be inspected -- it must NOT be subtracted from
        // cmd, or the regression loses the term that balances gravity.
        fprintf(g_log,
            "dynamic_excitation,%ld,%d,%s,%.6f,%.6f,%.6f,%.6f,"
            "%d,%.9f,%.9f,%.9f,%.9f,%.9f,%.9f,%.9f,"
            "%.9f,%.9f,%.9f,%.6f,%.6f,%.6f,"
            "%.9f,%.9f,%.9f,%lu\n",
            ts, poseId, axisName, c.amp, c.f0, c.f1, c.durS,
            i, s.t_s, s.cmd[0], s.cmd[1], s.cmd[2], s.q[0], s.q[1], s.q[2],
            s.lastJT[0], s.lastJT[1], s.lastJT[2], s.pos[0], s.pos[1], s.pos[2],
            (double)c.bias[0], (double)c.bias[1], (double)c.bias[2], c.clipped);
    }
    fflush(g_log);
}

// Run one chirp on one axis. Returns false if aborted.
static bool runCapture(Ctx& c, int axis, double amp, double durS,
                       double f0, double f1) {
    c.nSamples = 0;
    c.overflow = 0;
    c.axis = axis;
    c.amp = amp;
    c.durS = durS;
    c.f0 = f0;
    c.f1 = f1;
    c.tStart = nowMonotonic();
    c.capturing = 1;          // set last: the loop never sees a partial config

    const double deadline = c.tStart + durS + 0.20;
    while (nowMonotonic() < deadline) {
        if (g_abort) {
            c.capturing = 0;
            printf("    ABORTED mid-capture -- force zeroed in the servo loop.\n");
            return false;
        }
        usleep(2000);
    }

    c.capturing = 0;
    usleep(150000);           // let the arm settle before the next prompt
    return true;
}

int main(int argc, char** argv) {
    const char* deviceName = HD_DEFAULT_DEVICE;
    const char* logPath = nullptr;
    const char* axesSpec = "xyz";
    const char* statusPath = nullptr;
    double amp = kDefaultAmpN;
    double durS = kDefaultDurS;
    double f0 = kDefaultF0Hz;
    double f1 = kDefaultF1Hz;
    bool haveDevice = false;

    for (int i = 1; i < argc; ++i) {
        if (!std::strcmp(argv[i], "--log")  && i + 1 < argc) logPath = argv[++i];
        else if (!std::strncmp(argv[i], "--log=", 6))        logPath = argv[i] + 6;
        else if (!std::strcmp(argv[i], "--amp")  && i + 1 < argc) amp  = atof(argv[++i]);
        else if (!std::strcmp(argv[i], "--dur")  && i + 1 < argc) durS = atof(argv[++i]);
        else if (!std::strcmp(argv[i], "--f0")   && i + 1 < argc) f0   = atof(argv[++i]);
        else if (!std::strcmp(argv[i], "--f1")   && i + 1 < argc) f1   = atof(argv[++i]);
        else if (!std::strcmp(argv[i], "--axes") && i + 1 < argc) axesSpec = argv[++i];
        else if (!std::strcmp(argv[i], "--status") && i + 1 < argc) statusPath = argv[++i];
        else if (!std::strncmp(argv[i], "--status=", 9))            statusPath = argv[i] + 9;
        else if (!haveDevice) { deviceName = argv[i]; haveDevice = true; }
    }

    // Clamp the request itself, and say so. The servo loop clamps again.
    if (amp > kMaxForceN) {
        printf("NOTE: --amp %.3f N exceeds the %.2f N ceiling; clamped.\n", amp, kMaxForceN);
        amp = kMaxForceN;
    }
    if (amp <= 0.0) { printf("FATAL: --amp must be positive.\n"); return 1; }
    if (durS <= 0.0 || durS > 20.0) {
        printf("FATAL: --dur must be in (0, 20] s (buffer holds %d samples at 1 kHz).\n",
               kMaxSamples);
        return 1;
    }
    if (f0 <= 0.0 || f1 <= 0.0 || f1 < f0) {
        printf("FATAL: need 0 < --f0 <= --f1.\n");
        return 1;
    }

    int axisList[3]; int nAxes = 0;
    for (const char* p = axesSpec; *p && nAxes < 3; ++p) {
        if (*p == 'x' || *p == 'X') axisList[nAxes++] = 0;
        else if (*p == 'y' || *p == 'Y') axisList[nAxes++] = 1;
        else if (*p == 'z' || *p == 'Z') axisList[nAxes++] = 2;
    }
    if (nAxes == 0) { printf("FATAL: --axes must name at least one of x,y,z.\n"); return 1; }
    static const char* axName[3] = {"X", "Y", "Z"};

    printf("=== Bounded dynamic excitation: data for M_n identification ===\n");
    printf("device: %s\n", haveDevice ? deviceName : "(default)");
    printf("chirp %.2f -> %.2f Hz over %.1f s, peak %.3f N (ceiling %.2f N, continuous rating 0.88 N)\n",
           f0, f1, durS, amp, kMaxForceN);
    printf("axes: ");
    for (int i = 0; i < nAxes; ++i) printf("%s ", axName[axisList[i]]);
    printf("\nL1 = %.4f m  L2 = %.4f m\n",
           touch_haptics::kL1Meters, touch_haptics::kL2Meters);
    printf("\nDO NOT GRIP THE STYLUS during a capture -- position the arm, release it,\n"
           "then start. Your hand's impedance would be what gets identified.\n");
    printf("Run hdapi_fk_residual first if you intend to use J(q)-derived columns.\n");
    printf("\nABORT: Ctrl-C at any time. The servo loop zeroes force within ~1 ms,\n"
           "then the tool unwinds and disables force output. Safe to use mid-capture.\n\n");

    struct sigaction sa;
    memset(&sa, 0, sizeof(sa));
    sa.sa_handler = onSigint;
    sigaction(SIGINT, &sa, nullptr);
    sigaction(SIGTERM, &sa, nullptr);

    logOpen(logPath);
    if (g_log) printf("logging samples to: %s\n\n", logPath);

    HHD hHD = hdInitDevice(deviceName);
    HDErrorInfo e;
    if (HD_DEVICE_ERROR(e = hdGetError())) {
        printf("FATAL: hdInitDevice failed: %s (0x%04X)\n", errName(e.errorCode), e.errorCode);
        logClose();
        return 1;
    }
    hdMakeCurrentDevice(hHD);
    hdEnable(HD_FORCE_OUTPUT);
    if (HD_DEVICE_ERROR(e = hdGetError())) {
        printf("FATAL: hdEnable(HD_FORCE_OUTPUT) failed: %s\n", errName(e.errorCode));
        hdDisableDevice(hHD);
        logClose();
        return 1;
    }

    static Sample buffer[kMaxSamples];
    Ctx ctx;
    memset(&ctx, 0, sizeof(ctx));
    ctx.hHD = hHD;
    ctx.buf = buffer;

    HDSchedulerHandle handle =
        hdScheduleAsynchronous(servoLoop, &ctx, HD_MAX_SCHEDULER_PRIORITY);
    hdStartScheduler();
    if (HD_DEVICE_ERROR(e = hdGetError())) {
        printf("FATAL: hdStartScheduler failed: %s (0x%04X)\n", errName(e.errorCode), e.errorCode);
        hdDisable(HD_FORCE_OUTPUT); hdDisableDevice(hHD);
        logClose();
        return 1;
    }

    usleep(200000);
    if (ctx.frames == 0) {
        printf("FATAL: servo loop never ran. Scheduler is not alive.\n");
        zeroForce(ctx);
        hdStopScheduler(); hdUnschedule(handle);
        hdDisable(HD_FORCE_OUTPUT); hdDisableDevice(hHD);
        logClose();
        return 1;
    }

    statusStart(ctx, statusPath);
    if (statusPath) {
        printf("live status -> %s\n", statusPath);
        printf("  watch from another terminal:  watch -n 0.2 cat %s\n\n", statusPath);
    }

    int nPose = 0;
    int nCaptures = 0;

    while (nPose < kMaxPoses && !g_abort) {
        printf("\n--- pose %d --- HOLD the arm where you want it, clear of the table,\n",
               nPose + 1);
        printf("                then Enter to auto-find the holding force. 'q' finishes: ");
        fflush(stdout);
        int ch = getchar();
        if (g_abort) break;
        if (ch == 'q' || ch == 'Q') { while (ch != '\n' && ch != EOF) ch = getchar(); break; }
        while (ch != '\n' && ch != EOF) ch = getchar();
        if (ch == EOF) break;

        double heldBias[3] = {0, 0, 0};
        if (!autoHold(ctx, heldBias)) {
            if (g_abort) break;
            ctx.holdMode = 0;
            ctx.bias[0] = ctx.bias[1] = ctx.bias[2] = 0.0;
            printf("    pose skipped, hold released. Reposition and try again.\n");
            continue;                 // same pose number
        }

        // Fit the chirp under the total ceiling. Reducing the amplitude is far
        // better than letting the servo loop clamp it: a clamped chirp is
        // distorted, and the harmonics that creates would be identified as
        // inertia. Shrinking it keeps the excitation a clean sinusoid, just a
        // smaller one.
        const double biasMag = std::sqrt(heldBias[0]*heldBias[0] +
                                         heldBias[1]*heldBias[1] +
                                         heldBias[2]*heldBias[2]);
        double poseAmp = amp;
        // 5% margin rather than filling the headroom exactly. Sizing to the cap
        // precisely leaves the chirp peak sitting on the clamp threshold, where
        // floating-point noise alone decides whether a frame clips -- and a
        // clipped frame distorts the excitation. Cheap insurance.
        const double headroom = 0.95 * (kMaxTotalN - biasMag);
        if (poseAmp > headroom) {
            poseAmp = headroom > 0.0 ? headroom : 0.0;
            printf("    amplitude reduced %.3f -> %.3f N so bias+chirp stays under %.2f N.\n",
                   amp, poseAmp, kMaxTotalN);
        }
        if (poseAmp < 0.05) {
            printf("    only %.3f N of headroom left after the hold -- too little to\n"
                   "    excite anything useful. Pose skipped.\n", poseAmp);
            ctx.bias[0] = ctx.bias[1] = ctx.bias[2] = 0.0;
            continue;
        }

        for (int a = 0; a < nAxes && !g_abort; ++a) {
            const int axis = axisList[a];
            // The hold is already frozen and verified; touching the arm now
            // would both corrupt the capture and invalidate the frozen bias.
            printf("  [%s] hands OFF. Enter to chirp: ", axName[axis]);
            fflush(stdout);
            int c2 = getchar();
            if (g_abort) break;
            while (c2 != '\n' && c2 != EOF) c2 = getchar();
            if (c2 == EOF) break;

            touch_haptics::Vec3 q0{ ctx.q[0], ctx.q[1], ctx.q[2] };
            touch_haptics::SingularityMetrics sm = touch_haptics::singularityMetrics(q0);
            if (sm.near_singular) {
                printf("    NOTE: pose is near-singular (r=%.4f m, cos(q2-q3)=%.3f).\n"
                       "    J(q)-derived quantities will be ill-conditioned here.\n",
                       sm.radial_r, sm.align_cos);
            }

            ctx.clipped = 0;
            const bool ok = runCapture(ctx, axis, poseAmp, durS, f0, f1);
            const int n = ctx.nSamples;

            if (ctx.overflow) {
                printf("    WARNING: sample buffer overflowed; capture truncated at %d.\n",
                       kMaxSamples);
            }
            if (ctx.clipped) {
                printf("    WARNING: force clamp hit on %lu frame(s). The chirp was\n"
                       "    DISTORTED, not merely attenuated, so this capture is biased\n"
                       "    toward higher apparent inertia. Treat it as suspect.\n",
                       ctx.clipped);
            }
            if (n > 0) {
                const double rate = (ctx.buf[n-1].t_s > 1e-6) ? (n - 1) / ctx.buf[n-1].t_s : 0.0;
                double maxdq = 0.0;
                for (int i = 0; i < n; ++i) {
                    for (int j = 0; j < 3; ++j) {
                        const double d = std::fabs(ctx.buf[i].q[j] - q0[j]);
                        if (d > maxdq) maxdq = d;
                    }
                }
                printf("    %d samples, %.0f Hz effective, peak |dq| = %.4f rad%s\n",
                       n, rate, maxdq,
                       maxdq < 0.005 ? "   <-- barely moved; raise --amp or --dur" : "");
                dumpCapture(ctx, nPose + 1, axName[axis]);
                ++nCaptures;
            } else {
                printf("    no samples captured.\n");
            }
            if (!ok) break;
        }
        ++nPose;
    }

    // Zero force before anything else, whatever happened above.
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
               "of the signal, before this teardown ran. Partial data was written.\n");
    }

    if (g_log) printf("log written: %s  (%d capture(s))\n", logPath, nCaptures);
    logClose();

    if (nCaptures == 0) {
        printf("No captures recorded.\n");
        return 1;
    }

    printf("\n=== Next step ===\n");
    printf("  Feed this CSV to identification/fit_inertia.py together with the\n"
           "  static-sweep CSV. M_n is NOT written anywhere by this tool; the\n"
           "  DOB's JointDobParams remain unset and uncommissioned.\n");
    printf("\n=== Done. ===\n");
    return 0;
}
