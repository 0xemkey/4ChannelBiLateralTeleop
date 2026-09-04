// hdapi_fk_residual.cpp
//
// STANDALONE commissioning check for the J(q) / FK convention question.
//
// This is the gate that must pass before the joint-space DOB path in
// touch_slave.cpp may ever be armed. It answers, empirically:
//
//   Does the Jacobian's angle convention (which joint is q1/q2/q3, what the
//   zero reference is, and what the signs are) and its Cartesian frame match
//   what HDAPI actually reports on THIS device, with L1 = L2 = 0.135 m?
//
// The forward kinematics in touch_dob.hpp was RECOVERED from the supplied
// Jacobian by integration, not assumed. So if FK(q) tracks the measured
// Cartesian position across several distinct poses, that simultaneously
// confirms the angle convention, the frame convention, and both link lengths.
// If it does not, the per-axis breakdown below localizes which part is wrong.
//
// PASSIVE. This tool never enables force output and never commands a force.
// It only reads. The stylus is moved by hand.
//
// ---------------------------------------------------------------------------
// WHY IT FITS AND REMOVES AN OFFSET  (do not skip -- this is not a detail)
// ---------------------------------------------------------------------------
// A Jacobian determines position only up to an additive constant: J = dx/dq is
// unchanged by x -> x + c. The FK in touch_dob.hpp took all integration
// constants as zero, so it is correct in SHAPE but carries no base offset,
// while HD_CURRENT_TRANSFORM is measured from the device's own origin.
//
// Comparing them directly would therefore report a large residual even if
// everything is perfectly right -- the difference would just be the unknown
// base translation. So this tool:
//
//   1. samples several poses,
//   2. computes d_i = measured_i - FK(q_i) for each,
//   3. takes c = mean(d_i) as the estimated base offset,
//   4. reports the residual d_i - c, i.e. the part NOT explained by a constant.
//
// It is the residual AFTER offset removal that must be small. A single pose
// would fit trivially and prove nothing, which is why >= 3 poses are required
// and more is better. Spread them widely: a cluster of nearby poses can be
// fitted by an offset even when the shape is wrong.
//
// The fitted offset c is itself reported, and is useful in its own right -- it
// should be a fixed translation, consistent run to run.
//
// BUILD (from this tools/ directory):
//   g++ -std=gnu++17 -I../include hdapi_fk_residual.cpp -o hdapi_fk_residual -lHD -lHDU -lrt -lpthread
//
// RUN:
//   ./hdapi_fk_residual                 # HD_DEFAULT_DEVICE
//   ./hdapi_fk_residual "Left"          # named device
//
// Move the stylus to a distinct pose, press Enter, repeat. Press q then Enter
// to finish and print the analysis. Use the full workspace: high/low,
// left/right, near/far. Hold still at the instant you press Enter.

#include <HD/hd.h>

#include "touch_haptics/touch_dob.hpp"

#include <cstdio>
#include <cstring>
#include <cstdlib>
#include <cmath>
#include <unistd.h>

static const int kMaxPoses = 64;
static const int kMinPoses = 3;

// HD_CURRENT_TRANSFORM translation is in mm; touch_slave.cpp uses the same
// 0.001 scale (pos_scale_) to reach metres.
static const double kMmToM = 0.001;

static const char* errName(HDerror code) {
    switch (code) {
        case HD_SUCCESS: return "HD_SUCCESS";
        case HD_INVALID_ENUM: return "HD_INVALID_ENUM";
        case HD_INVALID_VALUE: return "HD_INVALID_VALUE";
        case HD_INVALID_OPERATION: return "HD_INVALID_OPERATION";
        case HD_BAD_HANDLE: return "HD_BAD_HANDLE";
        case HD_DEVICE_FAULT: return "HD_DEVICE_FAULT";
        case HD_COMM_ERROR: return "HD_COMM_ERROR";
        case HD_TIMER_ERROR: return "HD_TIMER_ERROR";
        case HD_ILLEGAL_BEGIN: return "HD_ILLEGAL_BEGIN";
        case HD_ILLEGAL_END: return "HD_ILLEGAL_END";
        case HD_FRAME_ERROR: return "HD_FRAME_ERROR";
        default: return "UNKNOWN";
    }
}

struct Sample {
    HDdouble jointAngles[3];
    HDdouble transform[16];
    HDerror  errAngles;
    HDerror  errTransform;
};

struct Ctx {
    HHD hHD;
    volatile int snapRequest;
    volatile int snapReady;
    volatile unsigned long frames;
    Sample snap;
};

static HDerror drainLast() {
    HDerror last = HD_SUCCESS;
    HDErrorInfo e;
    while (HD_DEVICE_ERROR(e = hdGetError())) last = e.errorCode;
    return last;
}

// Read-only servo loop. No hdSetDoublev anywhere in this file.
static HDCallbackCode HDCALLBACK servoLoop(void* data) {
    Ctx* c = static_cast<Ctx*>(data);

    hdBeginFrame(c->hHD);

    if (c->snapRequest && !c->snapReady) {
        drainLast();
        hdGetDoublev(HD_CURRENT_JOINT_ANGLES, c->snap.jointAngles);
        c->snap.errAngles = drainLast();
        hdGetDoublev(HD_CURRENT_TRANSFORM, c->snap.transform);
        c->snap.errTransform = drainLast();

        c->snapReady   = 1;
        c->snapRequest = 0;
    }

    hdEndFrame(c->hHD);

    c->frames++;
    return HD_CALLBACK_CONTINUE;
}

static bool takeSnapshot(Ctx& c, Sample& out, int timeoutMs = 2000) {
    c.snapReady   = 0;
    c.snapRequest = 1;
    for (int i = 0; i < timeoutMs; ++i) {
        if (c.snapReady) { memcpy(&out, &c.snap, sizeof(Sample)); return true; }
        usleep(1000);
    }
    c.snapRequest = 0;
    return false;
}

// The 6 permutations of three axes.
static const int kPerm[6][3] = {
    {0,1,2},{0,2,1},{1,0,2},{1,2,0},{2,0,1},{2,1,0}
};

struct Hypothesis {
    int perm;          // index into kPerm
    int sx, sy, sz;    // output sign per axis, +1/-1
    int qs0, qs1, qs2; // joint-angle sign per joint, +1/-1
    double rms;
    touch_haptics::Vec3 offset;
};

// RMS residual, after fitting a constant offset, for one candidate convention.
static Hypothesis evaluate(const touch_haptics::Vec3* q,
                           const touch_haptics::Vec3* meas,
                           int n, int perm, int sx, int sy, int sz,
                           int qs0, int qs1, int qs2) {
    Hypothesis h;
    h.perm = perm; h.sx = sx; h.sy = sy; h.sz = sz;
    h.qs0 = qs0; h.qs1 = qs1; h.qs2 = qs2;

    const int sgn[3] = {sx, sy, sz};
    touch_haptics::Vec3 sum{0.0, 0.0, 0.0};
    static touch_haptics::Vec3 diff[kMaxPoses];

    for (int i = 0; i < n; ++i) {
        const touch_haptics::Vec3 qq{q[i][0] * qs0, q[i][1] * qs1, q[i][2] * qs2};
        const touch_haptics::Vec3 fk = touch_haptics::fkFromJacobianConvention(qq);
        for (int a = 0; a < 3; ++a) {
            const double mapped = sgn[a] * fk[kPerm[perm][a]];
            diff[i][a] = meas[i][a] - mapped;
            sum[a] += diff[i][a];
        }
    }
    for (int a = 0; a < 3; ++a) h.offset[a] = sum[a] / n;

    double acc = 0.0;
    for (int i = 0; i < n; ++i) {
        for (int a = 0; a < 3; ++a) {
            const double r = diff[i][a] - h.offset[a];
            acc += r * r;
        }
    }
    h.rms = std::sqrt(acc / n);   // metres, per-pose vector RMS
    return h;
}

int main(int argc, char** argv) {
    const char* deviceName = (argc > 1) ? argv[1] : HD_DEFAULT_DEVICE;

    printf("=== FK / Jacobian convention commissioning check ===\n");
    printf("device: %s\n", (argc > 1) ? deviceName : "(default)");
    printf("L1 = %.4f m   L2 = %.4f m  (touch_dob.hpp, SUPPLIED values)\n",
           touch_haptics::kL1Meters, touch_haptics::kL2Meters);
    printf("PASSIVE: force output is never enabled. Move the stylus by hand.\n\n");

    HHD hHD = hdInitDevice(deviceName);
    HDErrorInfo e;
    if (HD_DEVICE_ERROR(e = hdGetError())) {
        printf("FATAL: hdInitDevice failed: %s (0x%04X)\n", errName(e.errorCode), e.errorCode);
        return 1;
    }
    hdMakeCurrentDevice(hHD);

    Ctx ctx;
    memset(&ctx, 0, sizeof(ctx));
    ctx.hHD = hHD;

    HDSchedulerHandle handle =
        hdScheduleAsynchronous(servoLoop, &ctx, HD_MAX_SCHEDULER_PRIORITY);
    hdStartScheduler();
    if (HD_DEVICE_ERROR(e = hdGetError())) {
        printf("FATAL: hdStartScheduler failed: %s (0x%04X)\n", errName(e.errorCode), e.errorCode);
        hdDisableDevice(hHD);
        return 1;
    }

    usleep(200000);
    if (ctx.frames == 0) {
        printf("FATAL: servo loop never ran. Scheduler is not alive.\n");
        hdStopScheduler(); hdUnschedule(handle); hdDisableDevice(hHD);
        return 1;
    }

    static touch_haptics::Vec3 q[kMaxPoses];
    static touch_haptics::Vec3 meas[kMaxPoses];
    int n = 0;

    printf("Move to a pose, hold still, press Enter to sample. 'q'+Enter to finish.\n");
    printf("Spread the poses widely -- nearby poses can be fitted by an offset\n");
    printf("even when the shape is wrong.\n\n");

    while (n < kMaxPoses) {
        printf("  [pose %2d] Enter to sample (q to finish): ", n + 1);
        fflush(stdout);
        int ch = getchar();
        if (ch == 'q' || ch == 'Q') { while (ch != '\n' && ch != EOF) ch = getchar(); break; }
        while (ch != '\n' && ch != EOF) ch = getchar();
        if (ch == EOF) break;

        Sample s;
        if (!takeSnapshot(ctx, s)) { printf("    TIMED OUT -- servo loop stalled.\n"); break; }
        if (s.errAngles != HD_SUCCESS || s.errTransform != HD_SUCCESS) {
            printf("    read error: angles=%s transform=%s\n",
                   errName(s.errAngles), errName(s.errTransform));
            continue;
        }

        q[n] = touch_haptics::Vec3{s.jointAngles[0], s.jointAngles[1], s.jointAngles[2]};
        meas[n] = touch_haptics::Vec3{s.transform[12] * kMmToM,
                                      s.transform[13] * kMmToM,
                                      s.transform[14] * kMmToM};
        printf("    q = [%8.5f %8.5f %8.5f] rad   measured = [%8.5f %8.5f %8.5f] m\n",
               q[n][0], q[n][1], q[n][2], meas[n][0], meas[n][1], meas[n][2]);
        ++n;
    }

    unsigned long totalFrames = ctx.frames;
    hdStopScheduler();
    hdUnschedule(handle);
    hdDisableDevice(hHD);

    printf("\nCollected %d pose(s). Servo frames: %lu\n", n, totalFrames);
    if (n < kMinPoses) {
        printf("\nNeed at least %d poses -- a constant offset is fitted from the data,\n"
               "so fewer than that cannot distinguish a correct FK from a wrong one.\n",
               kMinPoses);
        return 1;
    }

    // ---------------------------------------------------------------------
    // As-implemented convention: identity mapping, no sign flips.
    // ---------------------------------------------------------------------
    const Hypothesis id = evaluate(q, meas, n, 0, 1, 1, 1, 1, 1, 1);

    printf("\n=== AS IMPLEMENTED (identity mapping, offset fitted) ===\n");
    printf("fitted base offset c = [%9.6f %9.6f %9.6f] m\n",
           id.offset[0], id.offset[1], id.offset[2]);
    printf("\n%4s  %-32s %-32s %10s\n", "pose", "residual per axis [m]", "raw FK(q) [m]", "|resid|");
    double worst = 0.0;
    for (int i = 0; i < n; ++i) {
        const touch_haptics::Vec3 fk = touch_haptics::fkFromJacobianConvention(q[i]);
        double r[3], nr = 0.0;
        for (int a = 0; a < 3; ++a) {
            r[a] = meas[i][a] - fk[a] - id.offset[a];
            nr += r[a] * r[a];
        }
        nr = std::sqrt(nr);
        if (nr > worst) worst = nr;
        printf("%4d  [%9.6f %9.6f %9.6f] [%9.6f %9.6f %9.6f] %10.6f\n",
               i + 1, r[0], r[1], r[2], fk[0], fk[1], fk[2], nr);
    }
    printf("\nRMS residual after offset removal: %.6f m   worst pose: %.6f m\n", id.rms, worst);

    // Scale reference: how much did the measured position actually move? A small
    // residual is only meaningful relative to the spread of the data.
    touch_haptics::Vec3 mmin = meas[0], mmax = meas[0];
    for (int i = 1; i < n; ++i) {
        for (int a = 0; a < 3; ++a) {
            if (meas[i][a] < mmin[a]) mmin[a] = meas[i][a];
            if (meas[i][a] > mmax[a]) mmax[a] = meas[i][a];
        }
    }
    double span = 0.0;
    for (int a = 0; a < 3; ++a) span += (mmax[a] - mmin[a]) * (mmax[a] - mmin[a]);
    span = std::sqrt(span);
    printf("measured workspace span covered: %.6f m\n", span);
    if (span < 0.02) {
        printf("  <-- WARNING: poses are too tightly clustered to be conclusive.\n"
               "      Re-run with poses spread across the workspace.\n");
    }
    printf("relative residual (RMS / span): %.4f\n", span > 1e-9 ? id.rms / span : 0.0);

    printf("\nVERDICT: ");
    if (id.rms < 0.002 && span >= 0.02) {
        printf("PASS as implemented (RMS < 2 mm).\n");
        printf("  Angle convention, frame convention and both link lengths are\n"
               "  consistent with this device. The J(q) convention gate is satisfied.\n");
    } else if (span < 0.02) {
        printf("INCONCLUSIVE -- insufficient workspace coverage.\n");
    } else {
        printf("FAIL as implemented (RMS %.4f m).\n", id.rms);
        printf("  Do NOT arm the joint-space path. See the search below for which\n"
               "  convention difference, if any, would explain the discrepancy.\n");
    }

    // ---------------------------------------------------------------------
    // Diagnostic search. Hypothesis generation only -- NOT a licence to
    // silently adopt the winner. If a permuted/flipped convention fits far
    // better than the identity, that is evidence about how the paper's frame
    // and angle signs relate to HDAPI's, and the fix belongs in touch_dob.hpp
    // as a deliberate, reviewed change.
    // ---------------------------------------------------------------------
    printf("\n=== DIAGNOSTIC: best-fitting alternative conventions ===\n");
    printf("Searching 6 axis permutations x 8 output sign flips x 8 joint sign\n");
    printf("flips = 384 candidates. Reported for diagnosis only.\n\n");

    Hypothesis best[5];
    int nbest = 0;
    for (int p = 0; p < 6; ++p)
    for (int sx = -1; sx <= 1; sx += 2)
    for (int sy = -1; sy <= 1; sy += 2)
    for (int sz = -1; sz <= 1; sz += 2)
    for (int a0 = -1; a0 <= 1; a0 += 2)
    for (int a1 = -1; a1 <= 1; a1 += 2)
    for (int a2 = -1; a2 <= 1; a2 += 2) {
        const Hypothesis h = evaluate(q, meas, n, p, sx, sy, sz, a0, a1, a2);
        int pos = nbest;
        while (pos > 0 && best[pos-1].rms > h.rms) --pos;
        if (pos < 5) {
            for (int k = (nbest < 5 ? nbest : 4); k > pos; --k) best[k] = best[k-1];
            best[pos] = h;
            if (nbest < 5) ++nbest;
        }
    }

    printf("%5s  %-14s %-12s %-12s %10s\n", "rank", "axis map", "out signs", "q signs", "RMS [m]");
    static const char* axName = "xyz";
    for (int i = 0; i < nbest; ++i) {
        const Hypothesis& h = best[i];
        char mapbuf[16];
        snprintf(mapbuf, sizeof(mapbuf), "%c%c%c<-fk",
                 axName[kPerm[h.perm][0]], axName[kPerm[h.perm][1]], axName[kPerm[h.perm][2]]);
        printf("%5d  %-14s [%+d %+d %+d]   [%+d %+d %+d]   %10.6f%s\n",
               i + 1, mapbuf, h.sx, h.sy, h.sz, h.qs0, h.qs1, h.qs2, h.rms,
               (h.perm == 0 && h.sx == 1 && h.sy == 1 && h.sz == 1 &&
                h.qs0 == 1 && h.qs1 == 1 && h.qs2 == 1) ? "   <-- as implemented" : "");
    }

    printf("\nHow to read this:\n");
    printf("  * If rank 1 IS 'as implemented', the convention is right.\n");
    printf("  * If rank 1 beats it by a large margin, that mapping is the likely\n");
    printf("    relationship between the paper's convention and HDAPI's. Treat it\n");
    printf("    as a hypothesis to confirm, not a value to paste in.\n");
    printf("  * If even rank 1 has a large RMS, no sign/permutation fix works and\n");
    printf("    the problem is elsewhere: link lengths, a joint-angle offset\n");
    printf("    (this search covers signs, not offsets), or the Jacobian is simply\n");
    printf("    not this device's.\n");

    printf("\n=== Done. Nothing was commanded; no state was changed. ===\n");
    return 0;
}
