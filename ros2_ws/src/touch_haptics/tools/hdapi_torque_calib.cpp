// hdapi_torque_calib.cpp
//
// STANDALONE calibration sweep -- run only if hdapi_param_probe_v2 reports
// HD_LAST_JOINT_TORQUE (or HD_CURRENT_JOINT_TORQUE) as LIVE.
//
// Commands a series of known force magnitudes along one axis, one at a time,
// and logs the reported joint torque at each level. The question it answers:
// is reported_torque / commanded_force a consistent scale factor across test
// points, or does it vary with configuration in a way a simple J(q)^T scaling
// would not explain?
//
// Joint angles are snapshotted at every level so drift can be detected -- if
// the arm moved between levels, a changing ratio tells you nothing about
// linearity, so the run warns when angles shift beyond a threshold.
//
// SAFETY: max 0.3 N (device nominal max continuous is 0.88 N per the .config).
// Each level is held ~0.5 s. Force is zeroed and output disabled on every
// exit path. HOLD THE STYLUS STILL for the whole run, or the data is noise.
//
// BUILD:
//   g++ -std=gnu++17 hdapi_torque_calib.cpp -o hdapi_torque_calib -lHD -lHDU -lrt -lpthread
//
// RUN:
//   ./hdapi_torque_calib                 # HD_DEFAULT_DEVICE
//   ./hdapi_torque_calib "Left"          # named device
//   ./hdapi_torque_calib "Left" y        # sweep along +Y instead of +X

#include <HD/hd.h>
#include <cstdio>
#include <cstring>
#include <cstdlib>
#include <cmath>
#include <csignal>
#include <ctime>
#include <unistd.h>

// ---------------------------------------------------------------------------
// ABORT PATH  (added 2026-08-25)
// ---------------------------------------------------------------------------
// This tool commands up to 0.30 N and, until now, had NO signal handler at all.
// Ctrl-C terminated the process with a force still latched, leaving whether the
// device went passive up to the driver's watchdog rather than to us -- not a
// guarantee worth relying on with a hand near a loaded arm. Every other
// force-commanding tool here already closes that gap; this one was missed.
//
// Same contract as hdapi_static_sweep.cpp, deliberately identical so there is
// one abort story across the toolset: the handler only sets a flag (the sole
// async-signal-safe action available), and the real zeroing happens in
// servoLoop, which sees the flag and commands zero force within one servo frame
// (~1 ms) -- faster than any userspace teardown could manage. The level loop
// then unwinds and runs normal teardown.
static volatile sig_atomic_t g_abort = 0;
static void onSigint(int) { g_abort = 1; }

static const double kLevels[] = { 0.0, 0.1, 0.2, 0.3 };
static const int NLEVEL = sizeof(kLevels) / sizeof(kLevels[0]);

// Angle drift beyond this between the first and last level makes the ratio
// comparison meaningless, since J(q) itself changed.
static const double kAngleDriftWarn = 0.02;  // radians

// ---------------------------------------------------------------------------
// OPTIONAL CSV LOG  (--log <path>)
// ---------------------------------------------------------------------------
// Strictly additive. Nothing here runs inside servoLoop; rows are written from
// the main thread after all sampling and after force has been zeroed, so disk
// I/O can never delay a servo frame. Failure to open is a warning, not a fault.
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
            "tool,unix_time,axis,level_idx,commanded_n,"
            "cmd_1,cmd_2,cmd_3,"
            "last_jt_1,last_jt_2,last_jt_3,"
            "cur_jt_1,cur_jt_2,cur_jt_3,"
            "last_force_1,last_force_2,last_force_3,"
            "cur_force_1,cur_force_2,cur_force_3,"
            "q_1,q_2,q_3,err_last_jt,err_cur_jt\n");
    }
    fflush(g_log);
}

static void logClose() {
    if (g_log) { fflush(g_log); fclose(g_log); g_log = nullptr; }
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

struct Sample {
    HDdouble commanded[3];
    HDdouble lastJointTorque[3];
    HDdouble curJointTorque[3];
    HDdouble lastForce[3];
    HDdouble curForce[3];
    HDdouble jointAngles[3];
    HDerror  errLastJT;
    HDerror  errCurJT;
};

struct Ctx {
    HHD hHD;
    volatile double cmd[3];
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

static HDCallbackCode HDCALLBACK servoLoop(void* data) {
    Ctx* c = static_cast<Ctx*>(data);

    hdBeginFrame(c->hHD);

    // Abort wins over any commanded force, checked BEFORE the write so an
    // aborted frame can never emit a non-zero force.
    if (g_abort) { c->cmd[0] = c->cmd[1] = c->cmd[2] = 0.0; }

    HDdouble f[3] = { c->cmd[0], c->cmd[1], c->cmd[2] };
    hdSetDoublev(HD_CURRENT_FORCE, f);

    if (c->snapRequest && !c->snapReady) {
        drainLast();

        hdGetDoublev(HD_LAST_JOINT_TORQUE, c->snap.lastJointTorque);
        c->snap.errLastJT = drainLast();

        hdGetDoublev(HD_CURRENT_JOINT_TORQUE, c->snap.curJointTorque);
        c->snap.errCurJT = drainLast();

        hdGetDoublev(HD_LAST_FORCE, c->snap.lastForce);
        drainLast();
        hdGetDoublev(HD_CURRENT_FORCE, c->snap.curForce);
        drainLast();
        hdGetDoublev(HD_CURRENT_JOINT_ANGLES, c->snap.jointAngles);
        drainLast();

        c->snap.commanded[0] = f[0];
        c->snap.commanded[1] = f[1];
        c->snap.commanded[2] = f[2];

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
        // Bail immediately on abort rather than spinning out the full 2 s
        // timeout. The force is already zeroed by the servo loop at this point;
        // this only stops the operator waiting for a teardown that has, from
        // their point of view, already happened.
        if (g_abort) { c.snapRequest = 0; return false; }
        usleep(1000);
    }
    c.snapRequest = 0;
    return false;
}

static double norm3(const HDdouble v[3]) {
    return std::sqrt(v[0]*v[0] + v[1]*v[1] + v[2]*v[2]);
}

int main(int argc, char** argv) {
    // Additive arg parsing: positional device then axis, exactly as before;
    // --log is recognised anywhere and does not consume a positional slot.
    const char* deviceName = HD_DEFAULT_DEVICE;
    const char* logPath = nullptr;
    int axis = 0;
    int positional = 0;
    bool haveDevice = false;
    for (int i = 1; i < argc; ++i) {
        if (std::strcmp(argv[i], "--log") == 0 && i + 1 < argc) {
            logPath = argv[++i];
        } else if (std::strncmp(argv[i], "--log=", 6) == 0) {
            logPath = argv[i] + 6;
        } else if (positional == 0) {
            deviceName = argv[i]; haveDevice = true; ++positional;
        } else if (positional == 1) {
            char a = argv[i][0];
            if (a == 'y' || a == 'Y') axis = 1;
            else if (a == 'z' || a == 'Z') axis = 2;
            ++positional;
        }
    }
    const char* axisName = (axis == 0) ? "X" : (axis == 1) ? "Y" : "Z";

    printf("=== HDAPI joint-torque calibration sweep ===\n");
    printf("device: %s   axis: +%s   levels:",
           haveDevice ? deviceName : "(default)", axisName);
    for (int i = 0; i < NLEVEL; ++i) printf(" %.2f", kLevels[i]);
    printf(" N\n");
    printf("HOLD THE STYLUS STILL for the whole run.\n");

    struct sigaction sa;
    memset(&sa, 0, sizeof(sa));
    sa.sa_handler = onSigint;
    sigaction(SIGINT, &sa, nullptr);
    sigaction(SIGTERM, &sa, nullptr);
    printf("\nABORT: Ctrl-C at any time. The servo loop zeroes force within ~1 ms,\n"
           "then the tool unwinds and disables force output. Safe mid-sweep.\n\n");

    logOpen(logPath);
    if (g_log) printf("logging rows to: %s\n", logPath);

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
        hdDisable(HD_FORCE_OUTPUT);
        hdDisableDevice(hHD);
        return 1;
    }

    usleep(200000);
    if (ctx.frames == 0) {
        printf("FATAL: servo loop never ran. Scheduler is not alive.\n");
        hdStopScheduler(); hdUnschedule(handle);
        hdDisable(HD_FORCE_OUTPUT); hdDisableDevice(hHD);
        return 1;
    }

    Sample samples[NLEVEL];
    bool ok = true;

    for (int i = 0; i < NLEVEL && ok && !g_abort; ++i) {
        ctx.cmd[0] = ctx.cmd[1] = ctx.cmd[2] = 0.0;
        ctx.cmd[axis] = kLevels[i];
        printf("  level %d: %.2f N on +%s ... ", i, kLevels[i], axisName);
        fflush(stdout);
        usleep(500000);
        ok = takeSnapshot(ctx, samples[i]);
        printf("%s\n", ok ? "sampled" : "TIMED OUT");
    }

    // Zero force before anything else, whatever happened above.
    ctx.cmd[0] = ctx.cmd[1] = ctx.cmd[2] = 0.0;
    usleep(100000);
    unsigned long totalFrames = ctx.frames;
    hdStopScheduler();
    hdUnschedule(handle);
    hdDisable(HD_FORCE_OUTPUT);
    hdDisableDevice(hHD);

    // Rows written only after force is zeroed and the device is released.
    if (g_log) {
        static const char* axn[3] = {"X", "Y", "Z"};
        for (int i = 0; i < NLEVEL; ++i) {
            const Sample& sm = samples[i];
            double cmd[3] = {0.0, 0.0, 0.0};
            cmd[axis] = kLevels[i];
            fprintf(g_log,
                "torque_calib,%ld,%s,%d,%.6f,"
                "%.9f,%.9f,%.9f,%.9f,%.9f,%.9f,%.9f,%.9f,%.9f,"
                "%.9f,%.9f,%.9f,%.9f,%.9f,%.9f,"
                "%.9f,%.9f,%.9f,%s,%s\n",
                (long)time(nullptr), axn[axis], i, kLevels[i],
                cmd[0], cmd[1], cmd[2],
                sm.lastJointTorque[0], sm.lastJointTorque[1], sm.lastJointTorque[2],
                sm.curJointTorque[0], sm.curJointTorque[1], sm.curJointTorque[2],
                sm.lastForce[0], sm.lastForce[1], sm.lastForce[2],
                sm.curForce[0], sm.curForce[1], sm.curForce[2],
                sm.jointAngles[0], sm.jointAngles[1], sm.jointAngles[2],
                errName(sm.errLastJT), errName(sm.errCurJT));
        }
        fflush(g_log);
        printf("\nlog written: %s\n", logPath);
    }

    if (g_abort) {
        printf("\nABORTED by signal. Force was zeroed in the servo loop within ~1 ms\n"
               "of the signal, before this teardown ran. Partial data below.\n");
        logClose();
        return 1;
    }

    if (!ok) {
        printf("FATAL: a snapshot timed out -- servo loop stalled. Force zeroed.\n");
        logClose();
        return 1;
    }

    printf("\n=== HD_LAST_JOINT_TORQUE vs commanded force ===\n");
    printf("%6s  %-34s %10s %10s\n", "F [N]", "HD_LAST_JOINT_TORQUE", "|tau|", "|tau|/F");
    for (int i = 0; i < NLEVEL; ++i) {
        const Sample& s = samples[i];
        if (s.errLastJT != HD_SUCCESS) {
            printf("%6.2f  ERRORED: %s\n", kLevels[i], errName(s.errLastJT));
            continue;
        }
        double n = norm3(s.lastJointTorque);
        printf("%6.2f  [%9.6f %9.6f %9.6f] %10.6f ", kLevels[i],
               s.lastJointTorque[0], s.lastJointTorque[1], s.lastJointTorque[2], n);
        if (kLevels[i] > 1e-9) printf("%10.6f\n", n / kLevels[i]);
        else                   printf("%10s\n", "-");
    }

    printf("\n=== HD_CURRENT_JOINT_TORQUE vs commanded force ===\n");
    printf("%6s  %-34s %10s\n", "F [N]", "HD_CURRENT_JOINT_TORQUE", "|tau|");
    for (int i = 0; i < NLEVEL; ++i) {
        const Sample& s = samples[i];
        if (s.errCurJT != HD_SUCCESS) {
            printf("%6.2f  ERRORED: %s\n", kLevels[i], errName(s.errCurJT));
            continue;
        }
        printf("%6.2f  [%9.6f %9.6f %9.6f] %10.6f\n", kLevels[i],
               s.curJointTorque[0], s.curJointTorque[1], s.curJointTorque[2],
               norm3(s.curJointTorque));
    }

    printf("\n=== force read-back (sanity: did the command land?) ===\n");
    printf("%6s  %-34s %-34s\n", "F [N]", "HD_CURRENT_FORCE", "HD_LAST_FORCE");
    for (int i = 0; i < NLEVEL; ++i) {
        const Sample& s = samples[i];
        printf("%6.2f  [%9.6f %9.6f %9.6f] [%9.6f %9.6f %9.6f]\n", kLevels[i],
               s.curForce[0], s.curForce[1], s.curForce[2],
               s.lastForce[0], s.lastForce[1], s.lastForce[2]);
    }

    printf("\n=== joint angles at each level (configuration drift check) ===\n");
    for (int i = 0; i < NLEVEL; ++i) {
        printf("%6.2f  [%9.6f %9.6f %9.6f]\n", kLevels[i],
               samples[i].jointAngles[0], samples[i].jointAngles[1], samples[i].jointAngles[2]);
    }

    double maxDrift = 0.0;
    for (int j = 0; j < 3; ++j) {
        double d = std::fabs(samples[NLEVEL-1].jointAngles[j] - samples[0].jointAngles[j]);
        if (d > maxDrift) maxDrift = d;
    }
    printf("max joint-angle drift first->last: %.6f rad", maxDrift);
    if (maxDrift > kAngleDriftWarn) {
        printf("  <-- WARNING: arm moved, J(q) changed; ratios above are not comparable\n");
    } else {
        printf("  (ok, configuration held)\n");
    }

    printf("\nTotal servo frames: %lu (force zeroed, output disabled)\n", totalFrames);
    logClose();
    printf("=== Done. ===\n");
    return 0;
}
