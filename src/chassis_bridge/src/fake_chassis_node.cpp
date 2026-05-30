#include "chassis_bridge/fake_chassis_node.hpp"

#include <algorithm>
#include <chrono>
#include <memory>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2/LinearMath/Quaternion.h"

namespace chassis_bridge
{

FakeChassisNode::FakeChassisNode(const rclcpp::NodeOptions & options)
: Node("fake_chassis_node", options),
  logger_(this->get_logger()),
  last_update_time_(this->now())
{
  base_frame_id_ = this->declare_parameter<std::string>("base_frame_id", "base_link");
  odom_frame_id_ = this->declare_parameter<std::string>("odom_frame_id", "odom");
  cmd_vel_topic_ = this->declare_parameter<std::string>("cmd_vel_topic", "/cmd_vel");
  odom_topic_ = this->declare_parameter<std::string>("odom_topic", "/odom");
  control_frequency_hz_ = this->declare_parameter<double>("control_frequency_hz", 50.0);
  max_linear_velocity_mps_ = this->declare_parameter<double>("max_linear_velocity_mps", 0.4);
  max_angular_velocity_radps_ =
    this->declare_parameter<double>("max_angular_velocity_radps", 1.5);
  publish_tf_ = this->declare_parameter<bool>("publish_tf", true);

  if (control_frequency_hz_ <= 0.0) {
    RCLCPP_WARN(
      logger_, "Invalid control_frequency_hz=%.3f, fallback to 50.0", control_frequency_hz_);
    control_frequency_hz_ = 50.0;
  }

  odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>(odom_topic_, rclcpp::QoS(10));
  cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
    cmd_vel_topic_, rclcpp::QoS(10),
    std::bind(&FakeChassisNode::on_cmd_vel, this, std::placeholders::_1));
  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

  const auto period = std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::duration<double>(1.0 / control_frequency_hz_));
  timer_ = this->create_wall_timer(period, std::bind(&FakeChassisNode::on_timer, this));

  RCLCPP_INFO(
    logger_,
    "Fake chassis started: cmd_vel_topic=%s odom_topic=%s odom_frame_id=%s base_frame_id=%s "
    "frequency=%.2f max_vx=%.3f max_wz=%.3f publish_tf=%s",
    cmd_vel_topic_.c_str(), odom_topic_.c_str(), odom_frame_id_.c_str(), base_frame_id_.c_str(),
    control_frequency_hz_, max_linear_velocity_mps_, max_angular_velocity_radps_,
    publish_tf_ ? "true" : "false");
}

void FakeChassisNode::on_cmd_vel(const geometry_msgs::msg::Twist::SharedPtr msg)
{
  state_.cmd_vx = std::clamp(
    msg->linear.x, -max_linear_velocity_mps_, max_linear_velocity_mps_);
  state_.cmd_wz = std::clamp(
    msg->angular.z, -max_angular_velocity_radps_, max_angular_velocity_radps_);
}

void FakeChassisNode::on_timer()
{
  const rclcpp::Time now = this->now();
  const double dt_sec = (now - last_update_time_).seconds();
  last_update_time_ = now;

  estimator_.integrate(dt_sec, state_.cmd_vx, state_.cmd_wz, state_);
  odom_pub_->publish(make_odom_msg(now));
  if (publish_tf_) {
    publish_tf(now);
  }
}

nav_msgs::msg::Odometry FakeChassisNode::make_odom_msg(const rclcpp::Time & stamp) const
{
  nav_msgs::msg::Odometry msg;
  msg.header.stamp = stamp;
  msg.header.frame_id = odom_frame_id_;
  msg.child_frame_id = base_frame_id_;
  msg.pose.pose.position.x = state_.odom_x;
  msg.pose.pose.position.y = state_.odom_y;
  msg.pose.pose.position.z = 0.0;

  tf2::Quaternion q;
  q.setRPY(0.0, 0.0, state_.odom_theta);
  msg.pose.pose.orientation.x = q.x();
  msg.pose.pose.orientation.y = q.y();
  msg.pose.pose.orientation.z = q.z();
  msg.pose.pose.orientation.w = q.w();

  msg.twist.twist.linear.x = state_.cmd_vx;
  msg.twist.twist.angular.z = state_.cmd_wz;
  return msg;
}

void FakeChassisNode::publish_tf(const rclcpp::Time & stamp) const
{
  geometry_msgs::msg::TransformStamped transform;
  transform.header.stamp = stamp;
  transform.header.frame_id = odom_frame_id_;
  transform.child_frame_id = base_frame_id_;
  transform.transform.translation.x = state_.odom_x;
  transform.transform.translation.y = state_.odom_y;
  transform.transform.translation.z = 0.0;

  tf2::Quaternion q;
  q.setRPY(0.0, 0.0, state_.odom_theta);
  transform.transform.rotation.x = q.x();
  transform.transform.rotation.y = q.y();
  transform.transform.rotation.z = q.z();
  transform.transform.rotation.w = q.w();
  tf_broadcaster_->sendTransform(transform);
}

}  // namespace chassis_bridge

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<chassis_bridge::FakeChassisNode>());
  rclcpp::shutdown();
  return 0;
}
