// hdapi_passive_read.cpp
//
// ZERO-FORCE passive read. Commands [0,0,0] N for the whole run and samples
// the torque parameters while YOU push on the stylus by hand.
//
// This is the decisive test for whether HD_LAST_JOINT_TORQUE carries any
// sensed load at all. The sweep showed it is exactly linear in the COMMANDED
// force at fixed q, which is consistent with the driver simply computing
// J(q)^T * F_commanded. If that is all it is, then pushing hard on the stylus
// while commanding zero force must leave it at exactly [0,0,0] -- an external
// load the device cannot sense cannot appear in a computed quantity.
//
// If it stays zero: the parameter is a readback of your own command, carries
// no external-load information, and is useless as a momentum-observer input.
// If it moves: something is actually being estimated from the encoders/motors
// and is worth characterising further.
//
// SAFETY: commands zero force for the entire run. Motors are never driven.
//
// BUILD:
//   g++ -std=gnu++17 hdapi_passive_read.cpp -o hdapi_passive_read -lHD -lHDU -lrt -lpthread
//
// RUN:
//   ./hdapi_passive_read            # 6 s, HD_DEFAULT_DEVICE
//   ./hdapi_passive_read "Left" 10  # named device, 10 s

#include <HD/hd.h>
#include <cstdio>
#include <cstring>
#include <cstdlib>
#include <cmath>
#include <unistd.h>

struct Sample {
    HDdouble lastJointTorque[3];
    HDdouble curJointTorque[3];
    HDdouble lastForce[3];
    HDdouble jointAngles[3];
    HDdouble velocity[3];
};

struct Ctx {
    HHD hHD;
    volatile int snapRequest;
    volatile int snapReady;
    volatile unsigned long frames;
    Sample snap;
};

static void drain() { HDErrorInfo e; while (HD_DEVICE_ERROR(e = hdGetError())) {} }

static HDCallbackCode HDCALLBACK servoLoop(void* data) {
    Ctx* c = static_cast<Ctx*>(data);

    hdBeginFrame(c->hHD);

    // Explicitly command zero every frame -- never drive the motors.
    HDdouble zero[3] = { 0.0, 0.0, 0.0 };
    hdSetDoublev(HD_CURRENT_FORCE, zero);

    if (c->snapRequest && !c->snapReady) {
        drain();
        hdGetDoublev(HD_LAST_JOINT_TORQUE,    c->snap.lastJointTorque);
        hdGetDoublev(HD_CURRENT_JOINT_TORQUE, c->snap.curJointTorque);
        hdGetDoublev(HD_LAST_FORCE,           c->snap.lastForce);
        hdGetDoublev(HD_CURRENT_JOINT_ANGLES, c->snap.jointAngles);
        hdGetDoublev(HD_CURRENT_VELOCITY,     c->snap.velocity);
        drain();
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

static double norm3(const HDdouble v[3]) {
    return std::sqrt(v[0]*v[0] + v[1]*v[1] + v[2]*v[2]);
}

int main(int argc, char** argv) {
    const char* deviceName = (argc > 1) ? argv[1] : HD_DEFAULT_DEVICE;
    int seconds = (argc > 2) ? atoi(argv[2]) : 6;
    if (seconds < 1) seconds = 6;

    printf("=== HDAPI passive read (ZERO commanded force) ===\n");
    printf("device: %s   duration: %d s\n", (argc > 1) ? deviceName : "(default)", seconds);
    printf("\n>>> PUSH AND WIGGLE THE STYLUS HARD for the whole run. <<<\n");
    printf(">>> The motors are never driven; you are fighting nothing but friction. <<<\n\n");

    HHD hHD = hdInitDevice(deviceName);
    HDErrorInfo e;
    if (HD_DEVICE_ERROR(e = hdGetError())) {
        printf("FATAL: hdInitDevice failed (0x%04X)\n", e.errorCode);
        return 1;
    }

    hdMakeCurrentDevice(hHD);
    hdEnable(HD_FORCE_OUTPUT);   // enabled, but we only ever command zero
    drain();

    Ctx ctx;
    memset(&ctx, 0, sizeof(ctx));
    ctx.hHD = hHD;

    HDSchedulerHandle handle =
        hdScheduleAsynchronous(servoLoop, &ctx, HD_MAX_SCHEDULER_PRIORITY);
    hdStartScheduler();
    if (HD_DEVICE_ERROR(e = hdGetError())) {
        printf("FATAL: hdStartScheduler failed (0x%04X)\n", e.errorCode);
        hdDisable(HD_FORCE_OUTPUT); hdDisableDevice(hHD);
        return 1;
    }

    usleep(200000);

    const int nSamples = seconds * 4;   // 4 Hz
    double maxLastJT = 0.0, maxCurJT = 0.0, maxVel = 0.0;

    printf("%5s  %-32s %-32s %10s\n",
           "t[s]", "HD_LAST_JOINT_TORQUE", "HD_CURRENT_JOINT_ANGLES", "|vel|");
    for (int i = 0; i < nSamples; ++i) {
        Sample s;
        if (!takeSnapshot(ctx, s)) { printf("snapshot timed out\n"); break; }
        double nJT = norm3(s.lastJointTorque);
        double nV  = norm3(s.velocity);
        if (nJT > maxLastJT) maxLastJT = nJT;
        if (norm3(s.curJointTorque) > maxCurJT) maxCurJT = norm3(s.curJointTorque);
        if (nV > maxVel) maxVel = nV;
        printf("%5.2f  [%9.5f %9.5f %9.5f] [%9.5f %9.5f %9.5f] %10.3f\n",
               i * 0.25,
               s.lastJointTorque[0], s.lastJointTorque[1], s.lastJointTorque[2],
               s.jointAngles[0], s.jointAngles[1], s.jointAngles[2], nV);
        usleep(250000);
    }

    unsigned long totalFrames = ctx.frames;
    hdStopScheduler();
    hdUnschedule(handle);
    hdDisable(HD_FORCE_OUTPUT);
    hdDisableDevice(hHD);

    printf("\n=== RESULT ===\n");
    printf("max |HD_LAST_JOINT_TORQUE|    : %.6f\n", maxLastJT);
    printf("max |HD_CURRENT_JOINT_TORQUE| : %.6f\n", maxCurJT);
    printf("max |HD_CURRENT_VELOCITY|     : %.3f  (proves you actually moved it)\n", maxVel);
    if (maxVel < 1.0) {
        printf("\nWARNING: velocity stayed near zero -- the arm was not moved,\n");
        printf("so this run does not test anything. Re-run and push the stylus.\n");
    } else if (maxLastJT == 0.0) {
        printf("\nCONCLUSION: exactly zero under real external load.\n");
        printf("HD_LAST_JOINT_TORQUE is a readback of the COMMANDED force mapped\n");
        printf("through J(q)^T -- it senses nothing. Useless as a momentum-observer\n");
        printf("input: it can only ever tell you what you already told it.\n");
    } else {
        printf("\nCONCLUSION: nonzero under external load with zero commanded force.\n");
        printf("Something is being estimated -- worth characterising further.\n");
    }
    printf("\nTotal servo frames: %lu (no force was ever commanded)\n", totalFrames);
    return 0;
}
