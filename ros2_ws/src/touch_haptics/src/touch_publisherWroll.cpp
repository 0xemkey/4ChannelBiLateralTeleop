#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <std_msgs/msg/bool.hpp>

#include <HD/hd.h>
#include <HDU/hduVector.h>
#include <HDU/hduMatrix.h> 

#include <string>
#include <cmath>
#include <vector>

class TouchPublisher : public rclcpp::Node {
public:
  TouchPublisher()
  : Node("touch_publisher"),
    hhd_(HD_INVALID_HANDLE)
  {
    this->declare_parameter<std::string>("device", "Default Device");
    this->declare_parameter<std::string>("ns_prefix", "touch");

    device_ = this->get_parameter("device").as_string();
    ns_prefix_ = this->get_parameter("ns_prefix").as_string();

    pose_pub_ = this->create_publisher<geometry_msgs::msg::Pose>("/" + ns_prefix_ + "/pose", 10);
    button_pub_ = this->create_publisher<std_msgs::msg::Bool>("/" + ns_prefix_ + "/button_0", 10);

    init_device_or_die();

    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(10),
      std::bind(&TouchPublisher::tick, this)
    );

    RCLCPP_INFO(this->get_logger(), "Initialized device: '%s' namespace: '/%s'", device_.c_str(), ns_prefix_.c_str());
  }

  ~TouchPublisher() override {
    shutdown_device();
  }

private:
  void init_device_or_die() {
    HDErrorInfo error;

    hhd_ = hdInitDevice(device_.c_str());
    if (HD_DEVICE_ERROR(error = hdGetError())) {
      RCLCPP_FATAL(
        this->get_logger(),
        "hdInitDevice failed: errorCode=%d internalErrorCode=%d",
        error.errorCode, error.internalErrorCode
      );
      throw std::runtime_error("hdInitDevice failed");
    }

    hdEnable(HD_FORCE_OUTPUT);
    hdStartScheduler();

    if (HD_DEVICE_ERROR(error = hdGetError())) {
      RCLCPP_FATAL(
        this->get_logger(),
        "hdStartScheduler failed: errorCode=%d internalErrorCode=%d",
        error.errorCode, error.internalErrorCode
      );
      throw std::runtime_error("hdStartScheduler failed");
    }
  }

  void shutdown_device() {
    if (hhd_ != HD_INVALID_HANDLE) {
      hdStopScheduler();
      hdDisableDevice(hhd_);
      hhd_ = HD_INVALID_HANDLE;
    }
  }

  void tick() {
    hdMakeCurrentDevice(hhd_);

    HDErrorInfo error;

    hdBeginFrame(hhd_);
    if (HD_DEVICE_ERROR(error = hdGetError())) {
      RCLCPP_ERROR(this->get_logger(), "hdBeginFrame error: %d / %d", error.errorCode, error.internalErrorCode);
      return;
    }

    // 1. Pozisyonu Al
    hduVector3Dd pos;
    hdGetDoublev(HD_CURRENT_POSITION, pos);

    // 2. Transform Matrisini Al (Rotasyon icin gerekli)
    hduMatrix transform;
    hdGetDoublev(HD_CURRENT_TRANSFORM, transform);

    // 3. Butonlari Al
    int buttons = 0;
    hdGetIntegerv(HD_CURRENT_BUTTONS, &buttons);

    hdEndFrame(hhd_);
    if (HD_DEVICE_ERROR(error = hdGetError())) {
      RCLCPP_ERROR(this->get_logger(), "hdEndFrame error: %d / %d", error.errorCode, error.internalErrorCode);
      return;
    }

    geometry_msgs::msg::Pose pose_msg;
    // Pozisyon: mm -> metre dönüşümü
    pose_msg.position.x = pos[0] / 1000.0;
    pose_msg.position.y = pos[1] / 1000.0;
    pose_msg.position.z = pos[2] / 1000.0;

    // --- Rotasyon Matrisinden Quaternion Hesabı ---
    // Matris elemanlarini cikaralim
    double m00 = transform[0][0]; double m01 = transform[1][0]; double m02 = transform[2][0];
    double m10 = transform[0][1]; double m11 = transform[1][1]; double m12 = transform[2][1];
    double m20 = transform[0][2]; double m21 = transform[1][2]; double m22 = transform[2][2];

    double tr = m00 + m11 + m22;
    double qw, qx, qy, qz;

    if (tr > 0) {
      double S = sqrt(tr + 1.0) * 2; 
      qw = 0.25 * S;
      qx = (m21 - m12) / S;
      qy = (m02 - m20) / S;
      qz = (m10 - m01) / S;
    } else if ((m00 > m11) && (m00 > m22)) {
      double S = sqrt(1.0 + m00 - m11 - m22) * 2;
      qw = (m21 - m12) / S;
      qx = 0.25 * S;
      qy = (m01 + m10) / S;
      qz = (m02 + m20) / S;
    } else if (m11 > m22) {
      double S = sqrt(1.0 + m11 - m00 - m22) * 2;
      qw = (m02 - m20) / S;
      qx = (m01 + m10) / S;
      qy = 0.25 * S;
      qz = (m12 + m21) / S;
    } else {
      double S = sqrt(1.0 + m22 - m00 - m11) * 2;
      qw = (m10 - m01) / S;
      qx = (m02 + m20) / S;
      qy = (m12 + m21) / S;
      qz = 0.25 * S;
    }

    pose_msg.orientation.x = qx;
    pose_msg.orientation.y = qy;
    pose_msg.orientation.z = qz;
    pose_msg.orientation.w = qw;

    pose_pub_->publish(pose_msg);

    std_msgs::msg::Bool b;
    b.data = (buttons & HD_DEVICE_BUTTON_1) ? true : false;
    button_pub_->publish(b);
  }

  std::string device_;
  std::string ns_prefix_;

  rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr pose_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr button_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  HHD hhd_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  try {
    auto node = std::make_shared<TouchPublisher>();
    rclcpp::spin(node);
  } catch (const std::exception &e) {
    fprintf(stderr, "Fatal: %s\n", e.what());
  }
  rclcpp::shutdown();
  return 0;
}
