#include <HD/hd.h>
#include <HDU/hduError.h>

#include <chrono>
#include <cmath>
#include <cstdlib>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

static void checkHdError(const char* where) {
    HDErrorInfo error;
    if (HD_DEVICE_ERROR(error = hdGetError())) {
        std::string msg = std::string("[HDAPI ERROR] ") + where +
                          " code=" + std::to_string((int)error.errorCode) +
                          " internal=" + std::to_string((int)error.internalErrorCode);
        throw std::runtime_error(msg);
    }
}

static void quatFromRotationMatrix(const double R[3][3], double &qx, double &qy, double &qz, double &qw) {
    const double tr = R[0][0] + R[1][1] + R[2][2];
    if (tr > 0.0) {
        const double S = std::sqrt(tr + 1.0) * 2.0;
        qw = 0.25 * S;
        qx = (R[2][1] - R[1][2]) / S;
        qy = (R[0][2] - R[2][0]) / S;
        qz = (R[1][0] - R[0][1]) / S;
    } else if ((R[0][0] > R[1][1]) && (R[0][0] > R[2][2])) {
        const double S = std::sqrt(1.0 + R[0][0] - R[1][1] - R[2][2]) * 2.0;
        qw = (R[2][1] - R[1][2]) / S;
        qx = 0.25 * S;
        qy = (R[0][1] + R[1][0]) / S;
        qz = (R[0][2] + R[2][0]) / S;
    } else if (R[1][1] > R[2][2]) {
        const double S = std::sqrt(1.0 + R[1][1] - R[0][0] - R[2][2]) * 2.0;
        qw = (R[0][2] - R[2][0]) / S;
        qx = (R[0][1] + R[1][0]) / S;
        qy = 0.25 * S;
        qz = (R[1][2] + R[2][1]) / S;
    } else {
        const double S = std::sqrt(1.0 + R[2][2] - R[0][0] - R[1][1]) * 2.0;
        qw = (R[1][0] - R[0][1]) / S;
        qx = (R[0][2] + R[2][0]) / S;
        qy = (R[1][2] + R[2][1]) / S;
        qz = 0.25 * S;
    }
}

class TouchPosePublisher : public rclcpp::Node {
public:
    TouchPosePublisher() : Node("touch_pose_publisher") {
        device_name_ = this->declare_parameter<std::string>("device", "Default Device");
        topic_ = this->declare_parameter<std::string>("topic", "touch/pose");
        frame_id_ = this->declare_parameter<std::string>("frame_id", "touch");
        publish_hz_ = this->declare_parameter<double>("publish_hz", 200.0);

        pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(topic_, rclcpp::SensorDataQoS());

        initDevice();

        const auto period = std::chrono::duration<double>(1.0 / std::max(1.0, publish_hz_));
        timer_ = this->create_wall_timer(
            std::chrono::duration_cast<std::chrono::nanoseconds>(period),
            std::bind(&TouchPosePublisher::tick, this)
        );

        RCLCPP_INFO(this->get_logger(), "Publishing TouchX pose from device='%s' on topic='%s' @ %.1f Hz",
                    device_name_.c_str(), topic_.c_str(), publish_hz_);
    }

    ~TouchPosePublisher() override {
        try {
            hdStopScheduler();
        } catch (...) {}
        try {
            if (hhd_ != HD_INVALID_HANDLE) hdDisableDevice(hhd_);
        } catch (...) {}
    }

private:
    void initDevice() {
        hhd_ = hdInitDevice(device_name_.c_str());
        checkHdError("hdInitDevice");
        hdStartScheduler();
        checkHdError("hdStartScheduler");
    }

    void tick() {
        double pos_mm[3] = {0, 0, 0};
        double T[16] = {0};

        hdBeginFrame(hhd_);
        hdGetDoublev(HD_CURRENT_POSITION, pos_mm);
        hdGetDoublev(HD_CURRENT_TRANSFORM, T);
        hdEndFrame(hhd_);
        checkHdError("HD_CURRENT_*");

        // OpenHaptics: T is column-major
        double R[3][3];
        R[0][0] = T[0];  R[0][1] = T[4];  R[0][2] = T[8];
        R[1][0] = T[1];  R[1][1] = T[5];  R[1][2] = T[9];
        R[2][0] = T[2];  R[2][1] = T[6];  R[2][2] = T[10];

        double qx=0, qy=0, qz=0, qw=1;
        quatFromRotationMatrix(R, qx, qy, qz, qw);

        geometry_msgs::msg::PoseStamped msg;
        msg.header.stamp = this->get_clock()->now();
        msg.header.frame_id = frame_id_;

        // mm -> m
        msg.pose.position.x = pos_mm[0] / 1000.0;
        msg.pose.position.y = pos_mm[1] / 1000.0;
        msg.pose.position.z = pos_mm[2] / 1000.0;

        msg.pose.orientation.x = qx;
        msg.pose.orientation.y = qy;
        msg.pose.orientation.z = qz;
        msg.pose.orientation.w = qw;

        pub_->publish(msg);
    }

    std::string device_name_;
    std::string topic_;
    std::string frame_id_;
    double publish_hz_;

    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    HHD hhd_{HD_INVALID_HANDLE};
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    try {
        auto node = std::make_shared<TouchPosePublisher>();
        rclcpp::spin(node);
    } catch (const std::exception &e) {
        std::fprintf(stderr, "%s\n", e.what());
    }
    rclcpp::shutdown();
    return 0;
}
