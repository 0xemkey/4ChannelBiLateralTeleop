// hdapi_param_probe_v2.cpp
//
// STANDALONE diagnostic tool -- no ROS dependency.
//
// Purpose: empirically determine, on this actual hardware, whether
// HD_LAST_JOINT_TORQUE / HD_LAST_FORCE / HD_LAST_TORQUE /
// HD_LAST_GIMBAL_TORQUE are usable get-parameters. They exist in
// hdDefines.h but are absent from the OpenHaptics API Reference Guide's
// "Get Parameters" tables (A-1..A-6), so "compiles" != "works".
//
// Difference from v1: v1 called hdBeginFrame/hdSetDoublev from the main
// thread without ever calling hdStartScheduler(). In HDAPI, force commands
// only reach the motors from inside the servo loop, and HD_CURRENT_* state
// is refreshed per servo tick. Without a running scheduler the commanded
// force is never applied and both passes read stale state -- which would
// make every parameter look dead regardless of whether it is supported.
// This version does all frame work inside an asynchronous servo callback,
// exactly like touch_slave.cpp does.
//
// SAFETY: commands a fixed 0.3 N on +X for ~0.5 s (device nominal max
// continuous force is 0.88 N per the .config). Force is zeroed and output
// disabled on every exit path. Hold the stylus or clear the workspace.
//
// BUILD:
//   g++ -std=gnu++17 hdapi_param_probe_v2.cpp -o hdapi_param_probe_v2 -lHD -lHDU -lrt -lpthread
//   (headers live in /usr/include/HD, libHD in /lib -- no -I needed here)
//
// RUN:
//   ./hdapi_param_probe_v2                 # HD_DEFAULT_DEVICE
//   ./hdapi_param_probe_v2 "Left"          # named device from ~/.3dsystems

#include <HD/hd.h>
#include <cstdio>
#include <cstring>
#include <unistd.h>

static const int NPARAM = 8;

static const HDenum kParams[NPARAM] = {
    HD_CURRENT_FORCE,
    HD_CURRENT_TORQUE,
    HD_CURRENT_JOINT_TORQUE,
    HD_CURRENT_GIMBAL_TORQUE,
    HD_LAST_FORCE,
    HD_LAST_TORQUE,
    HD_LAST_JOINT_TORQUE,
    HD_LAST_GIMBAL_TORQUE,
};

static const char* kNames[NPARAM] = {
    "HD_CURRENT_FORCE",
    "HD_CURRENT_TORQUE",
    "HD_CURRENT_JOINT_TORQUE",
    "HD_CURRENT_GIMBAL_TORQUE",
    "HD_LAST_FORCE",
    "HD_LAST_TORQUE",          // deprecated
    "HD_LAST_JOINT_TORQUE",    // NOT in official tables
    "HD_LAST_GIMBAL_TORQUE",   // NOT in official tables
};

// Whether the official Get Parameters tables document this one.
static const bool kDocumented[NPARAM] = {
    true, true, true, true, false, false, false, false,
};

static const char* errName(HDerror code) {
    switch (code) {
        case HD_SUCCESS: return "HD_SUCCESS";
        case HD_INVALID_ENUM: return "HD_INVALID_ENUM";
        case HD_INVALID_VALUE: return "HD_INVALID_VALUE";
        case HD_INVALID_OPERATION: return "HD_INVALID_OPERATION";
        case HD_INVALID_INPUT_TYPE: return "HD_INVALID_INPUT_TYPE";
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

struct Snapshot {
    HDdouble vals[NPARAM][3];
    HDerror  errs[NPARAM];
    HDdouble jointAngles[3];
    HDdouble commanded[3];
};

struct Ctx {
    HHD hHD;
    volatile double cmd[3];
    volatile int    snapRequest;
    volatile int    snapReady;
    volatile unsigned long frames;
    Snapshot snap;
};

// Pops every queued error and returns the last one seen (HD_SUCCESS if none).
// Called only from the servo thread; never prints.
static HDerror drainLast() {
    HDerror last = HD_SUCCESS;
    HDErrorInfo e;
    while (HD_DEVICE_ERROR(e = hdGetError())) last = e.errorCode;
    return last;
}

// The real servo loop. Everything -- force command and every read -- happens
// here, inside one hdBeginFrame/hdEndFrame pair at the scheduler's rate.
static HDCallbackCode HDCALLBACK servoLoop(void* data) {
    Ctx* c = static_cast<Ctx*>(data);

    hdBeginFrame(c->hHD);

    HDdouble f[3] = { c->cmd[0], c->cmd[1], c->cmd[2] };
    hdSetDoublev(HD_CURRENT_FORCE, f);

    if (c->snapRequest && !c->snapReady) {
        // Clear anything the set above queued, so per-parameter error
        // attribution below is not contaminated by it.
        drainLast();

        for (int i = 0; i < NPARAM; ++i) {
            // Sentinel: if the call is a no-op we still see -999 rather than
            // a plausible-looking zero.
            c->snap.vals[i][0] = -999.0;
            c->snap.vals[i][1] = -999.0;
            c->snap.vals[i][2] = -999.0;
            hdGetDoublev(kParams[i], c->snap.vals[i]);
            c->snap.errs[i] = drainLast();
        }

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

// Asks the servo thread for a snapshot and waits for it.
static bool takeSnapshot(Ctx& c, Snapshot& out, int timeoutMs = 2000) {
    c.snapReady   = 0;
    c.snapRequest = 1;
    for (int i = 0; i < timeoutMs; ++i) {
        if (c.snapReady) {
            memcpy(&out, &c.snap, sizeof(Snapshot));
            return true;
        }
        usleep(1000);
    }
    c.snapRequest = 0;
    return false;
}

static void printSnapshot(const char* title, const Snapshot& s) {
    printf("\n--- %s ---\n", title);
    printf("commanded force this frame: [%.3f, %.3f, %.3f] N\n",
           s.commanded[0], s.commanded[1], s.commanded[2]);
    for (int i = 0; i < NPARAM; ++i) {
        printf("  %-26s ", kNames[i]);
        if (s.errs[i] != HD_SUCCESS) {
            printf("-> ERRORED: %s (0x%04X)\n", errName(s.errs[i]), s.errs[i]);
        } else {
            printf("-> [%10.6f, %10.6f, %10.6f]%s\n",
                   s.vals[i][0], s.vals[i][1], s.vals[i][2],
                   kDocumented[i] ? "" : "   (undocumented)");
        }
    }
    printf("  %-26s -> [%10.6f, %10.6f, %10.6f]  (reference)\n",
           "HD_CURRENT_JOINT_ANGLES",
           s.jointAngles[0], s.jointAngles[1], s.jointAngles[2]);
}

static bool isZero3(const HDdouble v[3]) {
    return v[0] == 0.0 && v[1] == 0.0 && v[2] == 0.0;
}

static bool same3(const HDdouble a[3], const HDdouble b[3]) {
    for (int i = 0; i < 3; ++i) {
        double d = a[i] - b[i];
        if (d < 0) d = -d;
        if (d > 1e-9) return false;
    }
    return true;
}

static void printVerdict(const Snapshot& idle, const Snapshot& loaded) {
    printf("\n=== VERDICT ===\n");
    printf("%-26s %-12s %s\n", "PARAMETER", "STATUS", "NOTE");
    for (int i = 0; i < NPARAM; ++i) {
        const char* status;
        const char* note;
        if (loaded.errs[i] != HD_SUCCESS || idle.errs[i] != HD_SUCCESS) {
            status = "UNUSABLE";
            note   = "call errored -- do not read this in the observer";
        } else if (loaded.vals[i][0] == -999.0 && loaded.vals[i][1] == -999.0) {
            status = "UNUSABLE";
            note   = "no error, but buffer untouched -- silent no-op";
        } else if (isZero3(loaded.vals[i]) && isZero3(idle.vals[i])) {
            status = "DEAD";
            note   = "reads zero both idle and under load -- carries no signal";
        } else if (same3(idle.vals[i], loaded.vals[i])) {
            status = "STATIC";
            note   = "nonzero but unchanged under load -- not force-dependent";
        } else {
            status = "LIVE";
            note   = "changed when force was commanded";
        }
        printf("%-26s %-12s %s\n", kNames[i], status, note);
    }
    printf("\nLIVE + undocumented means the symbol works on this firmware but is\n");
    printf("not contractually supported -- usable, but pin the OpenHaptics version.\n");
}

int main(int argc, char** argv) {
    const char* deviceName = (argc > 1) ? argv[1] : HD_DEFAULT_DEVICE;

    printf("=== HDAPI parameter probe v2 (scheduler-driven) ===\n");
    printf("Initializing device: %s\n", (argc > 1) ? deviceName : "(default)");

    HHD hHD = hdInitDevice(deviceName);
    HDErrorInfo e;
    if (HD_DEVICE_ERROR(e = hdGetError())) {
        printf("FATAL: hdInitDevice failed: %s (0x%04X)\n",
               errName(e.errorCode), e.errorCode);
        printf("Check the device name exists in ~/.3dsystems (GTDD.xml + config/)\n");
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
    ctx.cmd[0] = ctx.cmd[1] = ctx.cmd[2] = 0.0;

    HDSchedulerHandle handle =
        hdScheduleAsynchronous(servoLoop, &ctx, HD_MAX_SCHEDULER_PRIORITY);

    hdStartScheduler();
    if (HD_DEVICE_ERROR(e = hdGetError())) {
        printf("FATAL: hdStartScheduler failed: %s (0x%04X)\n",
               errName(e.errorCode), e.errorCode);
        hdDisable(HD_FORCE_OUTPUT);
        hdDisableDevice(hHD);
        return 1;
    }

    // Let the servo loop spin up so pass 1 reads live state, not startup state.
    usleep(200000);
    unsigned long framesAtStart = ctx.frames;
    if (framesAtStart == 0) {
        printf("FATAL: servo loop never ran (0 frames). Scheduler is not alive.\n");
        hdStopScheduler();
        hdUnschedule(handle);
        hdDisable(HD_FORCE_OUTPUT);
        hdDisableDevice(hHD);
        return 1;
    }
    printf("servo loop alive: %lu frames in first 200 ms (~%lu Hz)\n",
           framesAtStart, framesAtStart * 5);

    Snapshot idle;
    if (!takeSnapshot(ctx, idle)) {
        printf("FATAL: snapshot 1 timed out -- servo loop stalled.\n");
        hdStopScheduler(); hdUnschedule(handle);
        hdDisable(HD_FORCE_OUTPUT); hdDisableDevice(hHD);
        return 1;
    }
    printSnapshot("Pass 1: idle, commanding [0, 0, 0] N", idle);

    printf("\n--- Now commanding [0.3, 0, 0] N for ~0.5 s ---\n");
    printf("(hold the stylus, or make sure the workspace is clear)\n");
    ctx.cmd[0] = 0.3;
    usleep(500000);

    Snapshot loaded;
    bool gotLoaded = takeSnapshot(ctx, loaded);

    // Zero the force before anything else, whatever happened above.
    ctx.cmd[0] = ctx.cmd[1] = ctx.cmd[2] = 0.0;
    usleep(100000);

    unsigned long totalFrames = ctx.frames;

    hdStopScheduler();
    hdUnschedule(handle);
    hdDisable(HD_FORCE_OUTPUT);
    hdDisableDevice(hHD);

    if (!gotLoaded) {
        printf("FATAL: snapshot 2 timed out -- servo loop stalled. Force zeroed.\n");
        return 1;
    }

    printSnapshot("Pass 2: under load, commanding [0.3, 0, 0] N", loaded);
    printVerdict(idle, loaded);

    printf("\nTotal servo frames: %lu (force zeroed, output disabled)\n", totalFrames);
    printf("=== Done. ===\n");
    return 0;
}
