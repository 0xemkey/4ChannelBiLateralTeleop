// hdapi_param_probe.cpp
//
// STANDALONE diagnostic tool -- no ROS dependency. Run this directly on
// the machine physically connected to the device.
//
// NOTE: this is the as-written v1. It does NOT start the HD scheduler, so
// the commanded force never reaches the motors and the reads may be stale.
// Prefer hdapi_param_probe_v2.cpp for a trustworthy answer.

#include <HD/hd.h>
#include <cstdio>
#include <cstring>
#include <unistd.h>

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

static bool drainErrors(const char* context) {
    bool anyError = false;
    HDErrorInfo error;
    while (HD_DEVICE_ERROR(error = hdGetError())) {
        anyError = true;
        printf("    [ERROR after %s] %s (code 0x%04X)\n",
               context, errName(error.errorCode), error.errorCode);
    }
    return anyError;
}

static void probeParam3(const char* name, HDenum pname) {
    HDdouble val[3] = {-999.0, -999.0, -999.0};
    hdGetDoublev(pname, val);
    bool errored = drainErrors(name);
    if (errored) {
        printf("  %-28s -> ERRORED (see above)\n", name);
    } else {
        printf("  %-28s -> [%.6f, %.6f, %.6f]\n", name, val[0], val[1], val[2]);
    }
}

int main(int argc, char** argv) {
    const char* deviceName = (argc > 1) ? argv[1] : HD_DEFAULT_DEVICE;

    printf("=== HDAPI parameter probe (v1, NO scheduler) ===\n");
    printf("Initializing device: %s\n", (argc > 1) ? deviceName : "(default)");

    HHD hHD = hdInitDevice(deviceName);
    if (HD_DEVICE_ERROR(hdGetError())) {
        printf("FATAL: could not init device. Check name/connection.\n");
        return 1;
    }

    hdMakeCurrentDevice(hHD);
    hdEnable(HD_FORCE_OUTPUT);

    printf("\n--- Pass 1: BEFORE commanding any force ---\n");
    hdBeginFrame(hHD);
    probeParam3("HD_CURRENT_FORCE",        HD_CURRENT_FORCE);
    probeParam3("HD_CURRENT_TORQUE",       HD_CURRENT_TORQUE);
    probeParam3("HD_CURRENT_JOINT_TORQUE", HD_CURRENT_JOINT_TORQUE);
    probeParam3("HD_CURRENT_GIMBAL_TORQUE",HD_CURRENT_GIMBAL_TORQUE);
    probeParam3("HD_LAST_FORCE",           HD_LAST_FORCE);
    probeParam3("HD_LAST_TORQUE",          HD_LAST_TORQUE);
    probeParam3("HD_LAST_JOINT_TORQUE",    HD_LAST_JOINT_TORQUE);
    probeParam3("HD_LAST_GIMBAL_TORQUE",   HD_LAST_GIMBAL_TORQUE);
    hdEndFrame(hHD);

    printf("\n--- Commanding a fixed force [0.3, 0, 0] N for 500 frames (~0.5s at 1kHz) ---\n");
    printf("(hold the stylus, or make sure the workspace is clear)\n");

    for (int i = 0; i < 500; ++i) {
        hdBeginFrame(hHD);
        HDdouble force[3] = {0.3, 0.0, 0.0};
        hdSetDoublev(HD_CURRENT_FORCE, force);
        hdEndFrame(hHD);
        drainErrors("hdSetDoublev(HD_CURRENT_FORCE)");
        usleep(1000);
    }

    printf("\n--- Pass 2: AFTER commanding [0.3, 0, 0] N ---\n");
    hdBeginFrame(hHD);
    probeParam3("HD_CURRENT_FORCE",        HD_CURRENT_FORCE);
    probeParam3("HD_CURRENT_TORQUE",       HD_CURRENT_TORQUE);
    probeParam3("HD_CURRENT_JOINT_TORQUE", HD_CURRENT_JOINT_TORQUE);
    probeParam3("HD_CURRENT_GIMBAL_TORQUE",HD_CURRENT_GIMBAL_TORQUE);
    probeParam3("HD_LAST_FORCE",           HD_LAST_FORCE);
    probeParam3("HD_LAST_TORQUE",          HD_LAST_TORQUE);
    probeParam3("HD_LAST_JOINT_TORQUE",    HD_LAST_JOINT_TORQUE);
    probeParam3("HD_LAST_GIMBAL_TORQUE",   HD_LAST_GIMBAL_TORQUE);

    HDdouble jointAngles[3];
    hdGetDoublev(HD_CURRENT_JOINT_ANGLES, jointAngles);
    printf("  %-28s -> [%.6f, %.6f, %.6f]  (for reference)\n",
           "HD_CURRENT_JOINT_ANGLES", jointAngles[0], jointAngles[1], jointAngles[2]);

    hdEndFrame(hHD);

    printf("\n--- Cleanup: zeroing force, disabling output ---\n");
    hdBeginFrame(hHD);
    HDdouble zero[3] = {0, 0, 0};
    hdSetDoublev(HD_CURRENT_FORCE, zero);
    hdEndFrame(hHD);
    hdDisable(HD_FORCE_OUTPUT);
    hdDisableDevice(hHD);

    printf("\n=== Done. ===\n");
    return 0;
}
