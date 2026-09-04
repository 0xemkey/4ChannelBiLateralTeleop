#include <HD/hd.h>
#include <HDU/hduError.h>

#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <iostream>
#include <string>
#include <thread>

static void rpyFromRotationMatrix(const double R[3][3], double &roll, double &pitch, double &yaw) {
    // XYZ (roll around X, pitch around Y, yaw around Z)
    // Handles standard case, assumes R is proper rotation matrix.
    pitch = std::asin(-R[2][0]);
    const double cp = std::cos(pitch);

    if (std::fabs(cp) > 1e-9) {
        roll = std::atan2(R[2][1], R[2][2]);
        yaw  = std::atan2(R[1][0], R[0][0]);
    } else {
        // Gimbal lock fallback
        roll = 0.0;
        yaw  = std::atan2(-R[0][1], R[1][1]);
    }
}

static void checkHdError(const char* where) {
    HDErrorInfo error;
    if (HD_DEVICE_ERROR(error = hdGetError())) {
        std::cerr << "[HDAPI ERROR] " << where << "\n";
        std::cerr << "  Error Code: " << error.errorCode << "\n";
        std::cerr << "  Internal: " << error.internalErrorCode << "\n";
        std::cerr << "  Message: " << hdGetErrorString(error.errorCode) << "\n";
        std::exit(1);
    }
}

int main(int argc, char** argv) {
    std::string deviceName = "Default Device";
    std::string outPath = "./touch_log.csv";
    int hz = 200;

    for (int i = 1; i < argc; i++) {
        if (std::strcmp(argv[i], "--device") == 0 && i + 1 < argc) {
            deviceName = argv[++i];
        } else if (std::strcmp(argv[i], "--out") == 0 && i + 1 < argc) {
            outPath = argv[++i];
        } else if (std::strcmp(argv[i], "--hz") == 0 && i + 1 < argc) {
            hz = std::atoi(argv[++i]);
        }
    }

    HHD hHD = hdInitDevice(deviceName.c_str());
    checkHdError("hdInitDevice");

    hdStartScheduler();
    checkHdError("hdStartScheduler");

    std::ofstream csv(outPath);
    if (!csv.is_open()) {
        std::cerr << "Cannot open output file: " << outPath << "\n";
        return 1;
    }

    csv << "t,x,y,z,roll,pitch,yaw\n";
    csv.flush();

    auto t0 = std::chrono::steady_clock::now();
    const int sleep_us = (hz > 0) ? (int)std::llround(1e6 / (double)hz) : 5000;

    std::cout << "Logging TouchX from device=\"" << deviceName << "\" to " << outPath
              << " at ~" << hz << " Hz\n";
    std::cout << "Press Ctrl+C to stop.\n";

    while (true) {
        hdBeginFrame(hHD);

        double pos_mm[3] = {0,0,0};
        double T[16] = {0};

        hdGetDoublev(HD_CURRENT_POSITION, pos_mm);
        hdGetDoublev(HD_CURRENT_TRANSFORM, T);

        hdEndFrame(hHD);
        checkHdError("hdGetDoublev / frame");

        // Convert position mm -> m
        const double x = pos_mm[0] / 1000.0;
        const double y = pos_mm[1] / 1000.0;
        const double z = pos_mm[2] / 1000.0;

        // T is column-major 4x4 (OpenHaptics style). Rotation is upper-left 3x3.
        double R[3][3];
        R[0][0] = T[0];  R[0][1] = T[4];  R[0][2] = T[8];
        R[1][0] = T[1];  R[1][1] = T[5];  R[1][2] = T[9];
        R[2][0] = T[2];  R[2][1] = T[6];  R[2][2] = T[10];

        double roll=0.0, pitch=0.0, yaw=0.0;
        rpyFromRotationMatrix(R, roll, pitch, yaw);

        auto now = std::chrono::steady_clock::now();
        double t = std::chrono::duration<double>(now - t0).count();

        csv << t << "," << x << "," << y << "," << z << ","
            << roll << "," << pitch << "," << yaw << "\n";

        // Light console print (optional)
        std::cout << "\r"
                  << "t=" << t
                  << " x=" << x << " y=" << y << " z=" << z
                  << " r=" << roll << " p=" << pitch << " y=" << yaw
                  << "      " << std::flush;

        if (sleep_us > 0) std::this_thread::sleep_for(std::chrono::microseconds(sleep_us));
    }

    // Unreachable with Ctrl+C, but keeping clean shutdown pattern:
    csv.close();
    hdStopScheduler();
    hdDisableDevice(hHD);
    return 0;
}
