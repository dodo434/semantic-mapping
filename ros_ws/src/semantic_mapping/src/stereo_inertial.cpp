#include <algorithm>
#include <chrono>
#include <cmath>
#include <deque>
#include <fstream>
#include <iomanip>
#include <limits>
#include <iterator>
#include <memory>
#include <stdexcept>
#include <vector>

#include <Eigen/Geometry>
#include <opencv2/core.hpp>
#include <System.h>
#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <std_msgs/msg/header.hpp>
#include <std_msgs/msg/int32.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

namespace semantic_mapping
{
using Image = sensor_msgs::msg::Image;
using Imu = sensor_msgs::msg::Imu;
using namespace std::chrono_literals;

template<typename Msg>
int64_t stamp(const Msg & message)
{
  return rclcpp::Time(message.header.stamp).nanoseconds();
}

Sophus::SE3f as_pose(const geometry_msgs::msg::Transform & transform)
{
  const auto & r = transform.rotation;
  const auto & t = transform.translation;
  return {Eigen::Quaternionf(r.w, r.x, r.y, r.z).normalized(), Eigen::Vector3f(t.x, t.y, t.z)};
}

class StereoInertial : public rclcpp::Node
{
public:
  StereoInertial() : Node("stereo_inertial"), tf_buffer_(get_clock()), listener_(tf_buffer_),
    broadcaster_(*this)
  {
    const auto vocabulary = declare_parameter<std::string>("vocabulary", "");
    const auto settings = declare_parameter<std::string>("settings", "");
    world_ = declare_parameter<std::string>("world_frame", "map");
    base_ = declare_parameter<std::string>("base_frame", "base_link");
    odom_ = declare_parameter<std::string>("odom_frame", "odom");
    use_wheel_odom_ = declare_parameter("use_wheel_odom", true);
    publish_tf_ = declare_parameter("publish_tf", true);
    use_masks_ = declare_parameter("use_masks", false);
    mask_wait_ = seconds("mask_wait_sec", 0.12);
    mask_slop_ = seconds("mask_slop_sec", 0.050);
    stereo_slop_ = seconds("stereo_slop_sec", 0.003);
    if (!std::ifstream(vocabulary).good()) {
      throw std::runtime_error("Set vocabulary to the extracted ORBvoc.txt");
    }
    cv::FileStorage file(settings, cv::FileStorage::READ);
    if (!file.isOpened() || file["Calibration.CameraToImuTimeShiftSec"].empty()) {
      throw std::runtime_error("Use the measured Stage 2 ORB-SLAM3 settings");
    }
    const double shift = file["Calibration.CameraToImuTimeShiftSec"].real();
    if (!std::isfinite(shift) || std::abs(shift) > 1.0) {
      throw std::runtime_error("Invalid camera-to-IMU time shift");
    }
    shift_ns_ = std::llround(shift * 1e9);
    slam_ = std::make_unique<ORB_SLAM3::System>(
      vocabulary, settings, ORB_SLAM3::System::IMU_STEREO, false);
    const auto trajectory = declare_parameter<std::string>("trajectory", "");
    if (!trajectory.empty()) {
      if (std::ifstream(trajectory).good()) {
        throw std::runtime_error("Use a new trajectory output path");
      }
      trajectory_.open(trajectory);
      if (!trajectory_) {
        throw std::runtime_error("Could not create trajectory output");
      }
    }
    left_sub_ = subscribe_image("left_image", left_, 15);
    right_sub_ = subscribe_image("right_image", right_, 15);
    if (use_masks_) {
      left_mask_sub_ = subscribe_image("left_mask", left_masks_, 14);
      right_mask_sub_ = subscribe_image("right_mask", right_masks_, 14);
    }

    imu_sub_ = create_subscription<Imu>(
      "imu", rclcpp::SensorDataQoS().keep_last(300), [this](const Imu::ConstSharedPtr msg) {
        const auto & a = msg->linear_acceleration;
        const auto & w = msg->angular_velocity;
        if (!std::isfinite(a.x) || !std::isfinite(a.y) || !std::isfinite(a.z) ||
          !std::isfinite(w.x) || !std::isfinite(w.y) || !std::isfinite(w.z))
        {
          return;
        }
        if (!imu_.empty() && stamp(*msg) <= stamp(*imu_.back())) {
          return;
        }
        imu_.push_back(msg);
        if (imu_.size() > 500) {
          imu_.pop_front();
        }
      });
    pose_ = create_publisher<geometry_msgs::msg::PoseStamped>("body_pose", 10);
    keyframe_ = create_publisher<geometry_msgs::msg::PoseStamped>("keyframe", 10);
    revision_ = create_publisher<std_msgs::msg::Header>(
      "map_revision", rclcpp::QoS(1).transient_local());
    state_ = create_publisher<std_msgs::msg::Int32>("tracking_state", 10);
    timer_ = create_wall_timer(3ms, [this] { process(); });
  }

  ~StereoInertial() override
  {
    if (slam_) {
      slam_->Shutdown();
    }
  }

private:
  using Images = std::deque<Image::ConstSharedPtr>;
  using ImuQueue = std::deque<Imu::ConstSharedPtr>;

  int64_t seconds(const char * name, double initial)
  {
    const auto value = declare_parameter(name, initial);
    if (!std::isfinite(value) || value < 0 || value > 1) {
      throw std::runtime_error(std::string("Invalid seconds parameter: ") + name);
    }
    return std::llround(value * 1e9);
  }

  rclcpp::Subscription<Image>::SharedPtr subscribe_image(
    const char * topic, Images & queue, size_t capacity)
  {
    return create_subscription<Image>(
      topic, rclcpp::SensorDataQoS().keep_last(capacity),
      [&queue, capacity](Image::ConstSharedPtr msg) {
        if (queue.empty() || stamp(*msg) > stamp(*queue.back())) {
          queue.push_back(std::move(msg));
          if (queue.size() > capacity) {
            queue.pop_front();
          }
        }
      });
  }

  Image::ConstSharedPtr closest(const Images & frames, int64_t target) const
  {
    Image::ConstSharedPtr match;
    auto best = mask_slop_;
    for (const auto & frame : frames) {
      const auto difference = std::abs(stamp(*frame) - target);
      if (difference <= best) {
        best = difference;
        match = frame;
      }
    }
    return match;
  }

  void process()
  {
    while (!left_.empty() && !right_.empty()) {
      const auto left = left_.front();
      const auto right = right_.front();
      const auto image_ns = stamp(*left);
      const auto right_ns = stamp(*right);
      if (image_ns + stereo_slop_ < right_ns) {
        left_.pop_front();
        continue;
      }
      if (right_ns + stereo_slop_ < image_ns) {
        right_.pop_front();
        continue;
      }
      const auto imu_ns = image_ns + shift_ns_;
      if (imu_.empty() || stamp(*imu_.back()) < imu_ns) {
        return;
      }
      auto left_mask = use_masks_ ? closest(left_masks_, image_ns) : nullptr;
      auto right_mask = use_masks_ ? closest(right_masks_, right_ns) : nullptr;
      if (use_masks_ && (!left_mask || !right_mask) && stamp(*imu_.back()) < imu_ns + mask_wait_) {
        return;
      }
      left_.pop_front();
      right_.pop_front();
      if (imu_ns <= previous_image_ || stamp(*imu_.front()) > imu_ns) {
        continue;
      }
      try {
        track(left, right, left_mask, right_mask, imu_ns);
      } catch (const std::exception & error) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 3000, "%s", error.what());
      }
      return;
    }
  }

  void track(
    const Image::ConstSharedPtr & left, const Image::ConstSharedPtr & right,
    const Image::ConstSharedPtr & left_mask, const Image::ConstSharedPtr & right_mask,
    int64_t imu_ns)
  {
    const auto left_image = cv_bridge::toCvShare(left, "mono8");
    const auto right_image = cv_bridge::toCvShare(right, "mono8");
    cv::Mat exclude_left, exclude_right;
    if (left_mask && right_mask &&
      left_mask->width == left->width && right_mask->width == right->width &&
      left_mask->height == left->height && right_mask->height == right->height)
    {
      exclude_left = cv_bridge::toCvCopy(left_mask, "mono8")->image;
      exclude_right = cv_bridge::toCvCopy(right_mask, "mono8")->image;
    } else if (use_masks_) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
        "No timely stereo mask; tracking uses the available image features");
    }

    std::vector<ORB_SLAM3::IMU::Point> samples;
    const auto endpoint = std::lower_bound(imu_.begin(), imu_.end(), imu_ns,
      [](const Imu::ConstSharedPtr & sample, int64_t t) {return stamp(*sample) < t;});
    if (endpoint == imu_.end() || stamp(**endpoint) - imu_ns > 25000000) {
      return;
    }
    const std::string imu_frame = (*endpoint)->header.frame_id;
    if (epoch_ == 0) {
      epoch_ = stamp(*imu_.front());
    }
    for (auto it = imu_.begin(); it != std::next(endpoint); ++it) {
      const auto & msg = **it;
      const auto time = stamp(msg);
      if (time > submitted_imu_ && msg.header.frame_id == imu_frame) {
        const auto & a = msg.linear_acceleration;
        const auto & w = msg.angular_velocity;
        samples.emplace_back(a.x, a.y, a.z, w.x, w.y, w.z, (time - epoch_) * 1e-9);
      }
    }
    if (samples.empty()) {
      return;
    }
    submitted_imu_ = stamp(**endpoint);
    imu_.erase(imu_.begin(), endpoint);
    previous_image_ = imu_ns;
    slam_->SetDynamicStereoMasks(exclude_left, exclude_right);
    slam_->TrackStereo(
      left_image->image, right_image->image, (imu_ns - epoch_) * 1e-9, samples);
    std_msgs::msg::Int32 state;
    state.data = slam_->GetTrackingState();
    state_->publish(state);
    unsigned long map_id = 0;
    int map_revision = 0;
    Sophus::SE3f body;
    if (!slam_->GetTrackedBodyPose(body, map_id, map_revision)) {
      return;
    }
    auto header = left->header;
    header.frame_id = world_;
    if (!have_map_ || active_map_ != map_id || last_revision_ != map_revision) {
      revision_->publish(header);
      active_map_ = map_id;
      last_revision_ = map_revision;
      have_map_ = true;
    }
    publish_pose(header, body, imu_frame);
    unsigned long key_id;
    double key_time;
    Sophus::SE3f key_pose;
    if (slam_->GetLatestBodyKeyframe(key_pose, key_id, key_time) && key_id != latest_keyframe_) {
      latest_keyframe_ = key_id;
      header.stamp = rclcpp::Time(epoch_ + std::llround(key_time * 1e9) - shift_ns_);
      keyframe_->publish(to_message(header, key_pose));
    }
  }

  static geometry_msgs::msg::PoseStamped to_message(
    const std_msgs::msg::Header & header, const Sophus::SE3f & body)
  {
    geometry_msgs::msg::PoseStamped pose;
    pose.header = header;
    const auto p = body.translation();
    const auto q = body.unit_quaternion();
    pose.pose.position.x = p.x();
    pose.pose.position.y = p.y();
    pose.pose.position.z = p.z();
    pose.pose.orientation.x = q.x();
    pose.pose.orientation.y = q.y();
    pose.pose.orientation.z = q.z();
    pose.pose.orientation.w = q.w();
    return pose;
  }

  void publish_pose(
    const std_msgs::msg::Header & header, const Sophus::SE3f & body, const std::string & imu_frame)
  {
    pose_->publish(to_message(header, body));
    const auto p = body.translation();
    const auto q = body.unit_quaternion();
    if (trajectory_.is_open()) {
      trajectory_ << std::fixed << std::setprecision(9) << rclcpp::Time(header.stamp).seconds()
                  << ' ' << p.x() << ' ' << p.y() << ' ' << p.z()
                  << ' ' << q.x() << ' ' << q.y() << ' ' << q.z() << ' ' << q.w()
                  << std::endl;
    }
    if (!publish_tf_) {
      return;
    }
    try {
      const rclcpp::Time time(header.stamp, get_clock()->get_clock_type());
      const auto imu_from_base = tf_buffer_.lookupTransform(imu_frame, base_, time);
      auto result = body * as_pose(imu_from_base.transform);
      if (use_wheel_odom_) {
        const auto odom_from_base = tf_buffer_.lookupTransform(odom_, base_, time, rclcpp::Duration::from_seconds(0.04));
        result = result * as_pose(odom_from_base.transform).inverse();
      }
      geometry_msgs::msg::TransformStamped tf;
      tf.header = header;
      tf.child_frame_id = use_wheel_odom_ ? odom_ : base_;
      const auto t = result.translation();
      const auto r = result.unit_quaternion();
      tf.transform.translation.x = t.x();
      tf.transform.translation.y = t.y();
      tf.transform.translation.z = t.z();
      tf.transform.rotation.x = r.x();
      tf.transform.rotation.y = r.y();
      tf.transform.rotation.z = r.z();
      tf.transform.rotation.w = r.w();
      broadcaster_.sendTransform(tf);
    } catch (const tf2::TransformException & error) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 3000, "TF: %s", error.what());
    }
  }

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener listener_;
  tf2_ros::TransformBroadcaster broadcaster_;
  std::unique_ptr<ORB_SLAM3::System> slam_;
  std::ofstream trajectory_;
  std::string world_, base_, odom_;
  bool use_wheel_odom_, publish_tf_, use_masks_, have_map_ = false;
  int64_t mask_wait_, mask_slop_, stereo_slop_, shift_ns_;
  int64_t epoch_ = 0, previous_image_ = std::numeric_limits<int64_t>::min();
  int64_t submitted_imu_ = std::numeric_limits<int64_t>::min();
  int last_revision_ = 0;
  unsigned long active_map_ = 0, latest_keyframe_ = std::numeric_limits<unsigned long>::max();
  Images left_, right_, left_masks_, right_masks_;
  ImuQueue imu_;
  rclcpp::Subscription<Image>::SharedPtr left_sub_, right_sub_, left_mask_sub_, right_mask_sub_;
  rclcpp::Subscription<Imu>::SharedPtr imu_sub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_, keyframe_;
  rclcpp::Publisher<std_msgs::msg::Header>::SharedPtr revision_;
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr state_;
  rclcpp::TimerBase::SharedPtr timer_;
};
}  // namespace semantic_mapping

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<semantic_mapping::StereoInertial>());
  } catch (const std::exception & error) {
    RCLCPP_FATAL(rclcpp::get_logger("stereo_inertial"), "%s", error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
