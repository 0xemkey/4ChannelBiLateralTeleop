#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <std_msgs/msg/bool.hpp>

#include <HD/hd.h>
#include <HDU/hduVector.h>
#include <tf2/LinearMath/Quaternion.h>

#include <string>
#include <cmath>
#include <stdexcept>

class TouchPublisher2Buttons : public rclcpp::Node {
public:
  TouchPublisher2Buttons()
  : Node("touch_publisher_2buttons"),
    hhd_(HD_INVALID_HANDLE)
  {
    this->declare_parameter<std::string>("device", "Default Device");
    this->declare_parameter<std::string>("ns_prefix", "touch_left");

    device_ = this->get_parameter("device").as_string();
    ns_prefix_ = this->get_parameter("ns_prefix").as_string();

    pose_pub_ = this->create_publisher<geometry_msgs::msg::Pose>("/" + ns_prefix_ + "/pose", 10);
    button0_pub_ = this->create_publisher<std_msgs::msg::Bool>("/" + ns_prefix_ + "/button_0", 10);
    button1_pub_ = this->create_publisher<std_msgs::msg::Bool>("/" + ns_prefix_ + "/button_1", 10);

    init_device_or_die();

    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(10),
      std::bind(&TouchPublisher2Buttons::tick, this)
    );

    RCLCPP_INFO(this->get_logger(), "Initialized device: '%s' namespace: '/%s'", device_.c_str(), ns_prefix_.c_str());
  }

  ~TouchPublisher2Buttons() override {
    shutdown_device();
  }

private:
  void init_device_or_die() {
    HDErrorInfo error;
    hhd_ = hdInitDevice(device_.c_str());
    if (HD_DEVICE_ERROR(error = hdGetError())) {
      RCLCPP_FATAL(this->get_logger(), "hdInitDevice failed: errorCode=%d internalErrorCode=%d",
                   error.errorCode, error.internalErrorCode);
      throw std::runtime_error("hdInitDevice failed");
    }

    hdEnable(HD_FORCE_OUTPUT);
    hdStartScheduler();

    if (HD_DEVICE_ERROR(error = hdGetError())) {
      RCLCPP_FATAL(this->get_logger(), "hdStartScheduler failed: errorCode=%d internalErrorCode=%d",
                   error.errorCode, error.internalErrorCode);
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

    hduVector3Dd pos;
    hdGetDoublev(HD_CURRENT_POSITION, pos);

    int buttons = 0;
    hdGetIntegerv(HD_CURRENT_BUTTONS, &buttons);

    HDdouble T[16];
    hdGetDoublev(HD_CURRENT_TRANSFORM, T);

    hdEndFrame(hhd_);
    if (HD_DEVICE_ERROR(error = hdGetError())) {
      RCLCPP_ERROR(this->get_logger(), "hdEndFrame error: %d / %d", error.errorCode, error.internalErrorCode);
      return;
    }

    double r00 = T[0],  r01 = T[1],  r02 = T[2];
    double r10 = T[4],  r11 = T[5],  r12 = T[6];
    double r20 = T[8],  r21 = T[9],  r22 = T[10];

    double trace = r00 + r11 + r22;
    double qw, qx, qy, qz;

    if (trace > 0.0) {
      double s = std::sqrt(trace + 1.0) * 2.0;
      qw = 0.25 * s;
      qx = (r21 - r12) / s;
      qy = (r02 - r20) / s;
      qz = (r10 - r01) / s;
    } else if ((r00 > r11) && (r00 > r22)) {
      double s = std::sqrt(1.0 + r00 - r11 - r22) * 2.0;
      qw = (r21 - r12) / s;
      qx = 0.25 * s;
      qy = (r01 + r10) / s;
      qz = (r02 + r20) / s;
    } else if (r11 > r22) {
      double s = std::sqrt(1.0 + r11 - r00 - r22) * 2.0;
      qw = (r02 - r20) / s;
      qx = (r01 + r10) / s;
      qy = 0.25 * s;
      qz = (r12 + r21) / s;
    } else {
      double s = std::sqrt(1.0 + r22 - r00 - r11) * 2.0;
      qw = (r10 - r01) / s;
      qx = (r02 + r20) / s;
      qy = (r12 + r21) / s;
      qz = 0.25 * s;
    }

    tf2::Quaternion q(qx, qy, qz, qw);
    q.normalize();

    geometry_msgs::msg::Pose pose_msg;
    pose_msg.position.x = pos[0] / 1000.0;
    pose_msg.position.y = pos[1] / 1000.0;
    pose_msg.position.z = pos[2] / 1000.0;

    pose_msg.orientation.x = q.x();
    pose_msg.orientation.y = q.y();
    pose_msg.orientation.z = q.z();
    pose_msg.orientation.w = q.w();

    pose_pub_->publish(pose_msg);

    std_msgs::msg::Bool b0;
    b0.data = (buttons & HD_DEVICE_BUTTON_1) != 0;
    button0_pub_->publish(b0);

    std_msgs::msg::Bool b1;
    b1.data = (buttons & HD_DEVICE_BUTTON_2) != 0;
    button1_pub_->publish(b1);
  }

  std::string device_;
  std::string ns_prefix_;

  rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr pose_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr button0_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr button1_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  HHD hhd_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  try {
    auto node = std::make_shared<TouchPublisher2Buttons>();
    rclcpp::spin(node);
  } catch (const std::exception &e) {
    fprintf(stderr, "Fatal: %s\n", e.what());
  }
  rclcpp::shutdown();
  return 0;
}
