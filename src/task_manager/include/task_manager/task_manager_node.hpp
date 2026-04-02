#pragma once

#include <memory>
#include <vector>

#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_srvs/srv/trigger.hpp"

#include "task_manager/task_types.hpp"

namespace task_manager
{

class TaskManagerNode : public rclcpp::Node
{
public:
  using NavigateToPose = nav2_msgs::action::NavigateToPose;
  using GoalHandleNavigateToPose = rclcpp_action::ClientGoalHandle<NavigateToPose>;
  using Trigger = std_srvs::srv::Trigger;

  explicit TaskManagerNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  void declare_params();
  void load_task_points();
  void create_interfaces();

  void handle_start_navigation(
    const std::shared_ptr<Trigger::Request>,
    std::shared_ptr<Trigger::Response> response);

  void handle_cancel_navigation(
    const std::shared_ptr<Trigger::Request>,
    std::shared_ptr<Trigger::Response> response);

  void handle_pause_navigation(
    const std::shared_ptr<Trigger::Request>,
    std::shared_ptr<Trigger::Response> response);

  void try_send_current_goal();
  void publish_state();
  void set_state(TaskState new_state, const std::string & reason);

  void goal_response_callback(const GoalHandleNavigateToPose::SharedPtr & goal_handle);
  void feedback_callback(
    GoalHandleNavigateToPose::SharedPtr,
    const std::shared_ptr<const NavigateToPose::Feedback> feedback);
  void result_callback(const GoalHandleNavigateToPose::WrappedResult & result);

private:
  rclcpp_action::Client<NavigateToPose>::SharedPtr nav_to_pose_client_;

  rclcpp::Service<Trigger>::SharedPtr start_service_;
  rclcpp::Service<Trigger>::SharedPtr cancel_service_;
  rclcpp::Service<Trigger>::SharedPtr pause_service_;

  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr state_pub_;

  GoalHandleNavigateToPose::SharedPtr goal_handle_;

  std::vector<TaskPoint> task_points_;
  std::size_t current_index_{0};
  bool auto_advance_{true};

  TaskState state_{TaskState::IDLE};
};

}  // namespace task_manager