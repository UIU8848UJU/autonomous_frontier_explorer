#ifndef CHASSIS_BRIDGE__FAKE_CHASSIS_NODE_HPP_
#define CHASSIS_BRIDGE__FAKE_CHASSIS_NODE_HPP_

#include <memory>
#include <string>

#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/transform_broadcaster.h"

#include "chassis_bridge/chassis_state.hpp"
#include "chassis_bridge/odom_estimator.hpp"

namespace chassis_bridge
{

/// @brief fake 底盘节点，订阅速度指令并发布积分后的 odom 与 odom->base_link TF。
class FakeChassisNode : public rclcpp::Node
{
public:
  /// @brief 构造 fake 底盘节点并加载 ROS2 参数。
  explicit FakeChassisNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  void on_cmd_vel(const geometry_msgs::msg::Twist::SharedPtr msg);
  void on_timer();
  nav_msgs::msg::Odometry make_odom_msg(const rclcpp::Time & stamp) const;
  void publish_tf(const rclcpp::Time & stamp) const;

  rclcpp::Logger logger_;
  std::string base_frame_id_;
  std::string odom_frame_id_;
  std::string cmd_vel_topic_;
  std::string odom_topic_;
  double control_frequency_hz_{50.0};
  double max_linear_velocity_mps_{0.4};
  double max_angular_velocity_radps_{1.5};
  bool publish_tf_{true};

  FakeChassisState state_;
  OdomEstimator estimator_;
  rclcpp::Time last_update_time_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
};

}  // namespace chassis_bridge

#endif  // CHASSIS_BRIDGE__FAKE_CHASSIS_NODE_HPP_
