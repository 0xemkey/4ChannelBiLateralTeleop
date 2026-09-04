#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/bool.hpp>

#include <HD/hd.h>

#include <chrono>
#include <cmath>
#include <mutex>
#include <string>

using namespace std::chrono_literals;

struct TouchState
{
  double px{0.0}, py{0.0}, pz{0.0};
  double qx{0.0}, qy{0.0}, qz{0.0}, qw{1.0};
  double rr{0.0}, rp{0.0}, ry{0.0};
  int buttons{0};
  bool clutch{false};
  rclcpp::Time stamp{0, 0, RCL_ROS_TIME};
};

static inline void quatFromRotMat(
  double r00, double r01, double r02,
  double r10, double r11, double r12,
  double r20, double r21, double r22,
  double &qx, double &qy, double &qz, double &qw)
{
  double tr = r00 + r11 + r22;
  if (tr > 0.0) {
    double S = std::sqrt(tr + 1.0) * 2.0;
    qw = 0.25 * S;
    qx = (r21 - r12) / S;
    qy = (r02 - r20) / S;
    qz = (r10 - r01) / S;
  } else if ((r00 > r11) && (r00 > r22)) {
    double S = std::sqrt(1.0 + r00 - r11 - r22) * 2.0;
    qw = (r21 - r12) / S;
    qx = 0.25 * S;
    qy = (r01 + r10) / S;
    qz = (r02 + r20) / S;
  } else if (r11 > r22) {
    double S = std::sqrt(1.0 + r11 - r00 - r22) * 2.0;
    qw = (r02 - r20) / S;
    qx = (r01 + r10) / S;
    qy = 0.25 * S;
    qz = (r12 + r21) / S;
  } else {
    double S = std::sqrt(1.0 + r22 - r00 - r11) * 2.0;
    qw = (r10 - r01) / S;
    qx = (r02 + r20) / S;
    qy = (r12 + r21) / S;
    qz = 0.25 * S;
  }

  double n = std::sqrt(qx*qx + qy*qy + qz*qz + qw*qw);
  if (n > 0.0) { qx/=n; qy/=n; qz/=n; qw/=n; }
  else { qx=0.0; qy=0.0; qz=0.0; qw=1.0; }
}

static inline void rpyFromQuat(double qx, double qy, double qz, double qw, double &roll, double &pitch, double &yaw)
{
  double sinr_cosp = 2.0 * (qw*qx + qy*qz);
  double cosr_cosp = 1.0 - 2.0 * (qx*qx + qy*qy);
  roll = std::atan2(sinr_cosp, cosr_cosp);

  double sinp = 2.0 * (qw*qy - qz*qx);
  if (std::abs(sinp) >= 1.0) pitch = std::copysign(M_PI/2.0, sinp);
  else pitch = std::asin(sinp);

  double siny_cosp = 2.0 * (qw*qz + qx*qy);
  double cosy_cosp = 1.0 - 2.0 * (qy*qy + qz*qz);
  yaw = std::atan2(siny_cosp, cosy_cosp);
}

class TouchPublisher : public rclcpp::Node
{
public:
  TouchPublisher() : Node("touch_publisher")
  {
    device_name_ = this->declare_parameter<std::string>("device", "DEFAULT");
    frame_id_ = this->declare_parameter<std::string>("frame_id", "touch_base");
    publish_hz_ = this->declare_parameter<int>("publish_hz", 200);

    pos_scale_ = this->declare_parameter<double>("pos_scale", 0.001);
    invert_x_ = this->declare_parameter<bool>("invert_x", false);
    invert_y_ = this->declare_parameter<bool>("invert_y", false);
    invert_z_ = this->declare_parameter<bool>("invert_z", false);
    swap_xy_ = this->declare_parameter<bool>("swap_xy", false);

    clutch_button_mask_ = this->declare_parameter<int>("clutch_button_mask", 2);
    publish_while_clutched_ = this->declare_parameter<bool>("publish_while_clutched", true);

    // IMPORTANT: fixed topic names so we can remap exactly like your "With RPY" script
    pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("pose", 10);
    rpy_pub_ = this->create_publisher<geometry_msgs::msg::Vector3Stamped>("rpy", 10);
    buttons_pub_ = this->create_publisher<std_msgs::msg::Int32>("buttons", 10);
    clutch_pub_ = this->create_publisher<std_msgs::msg::Bool>("clutch", 10);

    initDevice();

    auto period = std::chrono::duration<double>(1.0 / std::max(1, publish_hz_));
    timer_ = this->create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      std::bind(&TouchPublisher::publishTick, this));
  }

  ~TouchPublisher() override { shutdownDevice(); }

private:
  void initDevice()
  {
    if (device_name_ == "DEFAULT") hHD_ = hdInitDevice(HD_DEFAULT_DEVICE);
    else hHD_ = hdInitDevice(device_name_.c_str());

    HDErrorInfo error = hdGetError();
    if (HD_DEVICE_ERROR(error)) {
      throw std::runtime_error(std::string("hdInitDevice failed: ") + hdGetErrorString(error.errorCode));
    }

    hdEnable(HD_FORCE_OUTPUT);
    hdStartScheduler();

    error = hdGetError();
    if (HD_DEVICE_ERROR(error)) {
      throw std::runtime_error(std::string("hdStartScheduler failed: ") + hdGetErrorString(error.errorCode));
    }

    callback_handle_ = hdScheduleAsynchronous(&TouchPublisher::hdCallback, this, HD_MAX_SCHEDULER_PRIORITY);
  }

  void shutdownDevice()
  {
    if (callback_handle_ != HD_INVALID_HANDLE) {
      hdUnschedule(callback_handle_);
      callback_handle_ = HD_INVALID_HANDLE;
    }
    hdStopScheduler();
    if (hHD_ != HD_INVALID_HANDLE) {
      hdDisableDevice(hHD_);
      hHD_ = HD_INVALID_HANDLE;
    }
  }

  static HDCallbackCode HDCALLBACK hdCallback(void *user_data)
  {
    auto *self = static_cast<TouchPublisher *>(user_data);

    hdBeginFrame(self->hHD_);

    double transform[16];
    hdGetDoublev(HD_CURRENT_TRANSFORM, transform);

    double x = transform[12];
    double y = transform[13];
    double z = transform[14];

    int buttons = 0;
    hdGetIntegerv(HD_CURRENT_BUTTONS, &buttons);

    hdEndFrame(self->hHD_);

    HDErrorInfo error = hdGetError();
    if (HD_DEVICE_ERROR(error)) return HD_CALLBACK_CONTINUE;

    double r00 = transform[0],  r01 = transform[1],  r02 = transform[2];
    double r10 = transform[4],  r11 = transform[5],  r12 = transform[6];
    double r20 = transform[8],  r21 = transform[9],  r22 = transform[10];

    double qx, qy, qz, qw;
    quatFromRotMat(r00, r01, r02, r10, r11, r12, r20, r21, r22, qx, qy, qz, qw);

    double roll, pitch, yaw;
    rpyFromQuat(qx, qy, qz, qw, roll, pitch, yaw);

    bool clutch = ((buttons & self->clutch_button_mask_) != 0);

    if (self->swap_xy_) std::swap(x, y);
    if (self->invert_x_) x = -x;
    if (self->invert_y_) y = -y;
    if (self->invert_z_) z = -z;

    x *= self->pos_scale_;
    y *= self->pos_scale_;
    z *= self->pos_scale_;

    {
      std::lock_guard<std::mutex> lock(self->mtx_);
      self->state_.px = x;
      self->state_.py = y;
      self->state_.pz = z;
      self->state_.qx = qx;
      self->state_.qy = qy;
      self->state_.qz = qz;
      self->state_.qw = qw;
      self->state_.rr = roll;
      self->state_.rp = pitch;
      self->state_.ry = yaw;
      self->state_.buttons = buttons;
      self->state_.clutch = clutch;
      self->state_.stamp = self->now();
    }

    return HD_CALLBACK_CONTINUE;
  }

  void publishTick()
  {
    TouchState s;
    {
      std::lock_guard<std::mutex> lock(mtx_);
      s = state_;
    }

    if (!publish_while_clutched_ && s.clutch) {
      std_msgs::msg::Bool clutch_msg;
      clutch_msg.data = true;
      clutch_pub_->publish(clutch_msg);

      std_msgs::msg::Int32 btn_msg;
      btn_msg.data = s.buttons;
      buttons_pub_->publish(btn_msg);
      return;
    }

    geometry_msgs::msg::PoseStamped pose;
    pose.header.stamp = s.stamp;
    pose.header.frame_id = frame_id_;
    pose.pose.position.x = s.px;
    pose.pose.position.y = s.py;
    pose.pose.position.z = s.pz;
    pose.pose.orientation.x = s.qx;
    pose.pose.orientation.y = s.qy;
    pose.pose.orientation.z = s.qz;
    pose.pose.orientation.w = s.qw;
    pose_pub_->publish(pose);

    geometry_msgs::msg::Vector3Stamped rpy;
    rpy.header = pose.header;
    rpy.vector.x = s.rr;
    rpy.vector.y = s.rp;
    rpy.vector.z = s.ry;
    rpy_pub_->publish(rpy);

    std_msgs::msg::Int32 btn_msg;
    btn_msg.data = s.buttons;
    buttons_pub_->publish(btn_msg);

    std_msgs::msg::Bool clutch_msg;
    clutch_msg.data = s.clutch;
    clutch_pub_->publish(clutch_msg);
  }

private:
  std::string device_name_;
  std::string frame_id_;
  int publish_hz_{200};

  double pos_scale_{0.001};
  bool invert_x_{false}, invert_y_{false}, invert_z_{false};
  bool swap_xy_{false};

  int clutch_button_mask_{2};
  bool publish_while_clutched_{true};

  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr rpy_pub_;
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr buttons_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr clutch_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  HHD hHD_{HD_INVALID_HANDLE};
  HDSchedulerHandle callback_handle_{HD_INVALID_HANDLE};

  std::mutex mtx_;
  TouchState state_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  try {
    auto node = std::make_shared<TouchPublisher>();
    rclcpp::spin(node);
  } catch (const std::exception &e) {
    fprintf(stderr, "%s\n", e.what());
  }
  rclcpp::shutdown();
  return 0;
}
