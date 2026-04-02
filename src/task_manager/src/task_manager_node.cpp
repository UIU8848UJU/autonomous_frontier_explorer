#include "task_manager/task_manager_node.hpp"

#include <chrono>
#include <functional>
#include <string>

using namespace std::chrono_literals;

namespace task_manager
{

TaskManagerNode::TaskManagerNode(const rclcpp::NodeOptions & options)
: Node("task_manager_node", options)
{
  declare_params();

  this->set_parameter(rclcpp::Parameter("use_sim_time", true));

  nav_to_pose_client_ =
    rclcpp_action::create_client<NavigateToPose>(this, "navigate_to_pose");

  load_task_points();
  create_interfaces();

  RCLCPP_INFO(this->get_logger(), "TaskManagerNode started.");
  publish_state();
}

void TaskManagerNode::declare_params()
{
  this->declare_parameter<bool>("auto_advance", true);
  this->declare_parameter<std::vector<double>>("goal", {0.5, 0.0, 1.0});

  this->declare_parameter<std::vector<std::string>>(
    "waypoint_names", {"p1", "p2"});

  this->declare_parameter<std::vector<double>>(
    "waypoint_values",
    {
      0.5, 0.0, 1.0,
      1.0, 0.0, 1.0
    });
}

void TaskManagerNode::load_task_points()
{
  auto_advance_ = this->get_parameter("auto_advance").as_bool();

  const auto names = this->get_parameter("waypoint_names").as_string_array();
  const auto values = this->get_parameter("waypoint_values").as_double_array();

  task_points_.clear();

  if (!names.empty() && values.size() == names.size() * 3) {
    for (std::size_t i = 0; i < names.size(); ++i) {
      TaskPoint point;
      point.name = names[i];
      point.pose.header.frame_id = "map";
      point.pose.pose.position.x = values[i * 3 + 0];
      point.pose.pose.position.y = values[i * 3 + 1];
      point.pose.pose.position.z = 0.0;
      point.pose.pose.orientation.x = 0.0;
      point.pose.pose.orientation.y = 0.0;
      point.pose.pose.orientation.z = 0.0;
      point.pose.pose.orientation.w = values[i * 3 + 2];
      task_points_.push_back(point);
    }
  }

  if (task_points_.empty()) {
    const auto goal = this->get_parameter("goal").as_double_array();
    TaskPoint point;
    point.name = "default_goal";
    point.pose.header.frame_id = "map";
    point.pose.pose.position.x = goal[0];
    point.pose.pose.position.y = goal[1];
    point.pose.pose.position.z = 0.0;
    point.pose.pose.orientation.w = goal[2];
    task_points_.push_back(point);
  }
}

void TaskManagerNode::create_interfaces()
{
  state_pub_ = this->create_publisher<std_msgs::msg::String>("task_state", 10);

  start_service_ = this->create_service<Trigger>(
    "start_navigation",
    std::bind(&TaskManagerNode::handle_start_navigation, this,
      std::placeholders::_1, std::placeholders::_2));

  cancel_service_ = this->create_service<Trigger>(
    "cancel_navigation",
    std::bind(&TaskManagerNode::handle_cancel_navigation, this,
      std::placeholders::_1, std::placeholders::_2));

  pause_service_ = this->create_service<Trigger>(
    "pause_navigation",
    std::bind(&TaskManagerNode::handle_pause_navigation, this,
      std::placeholders::_1, std::placeholders::_2));
}

void TaskManagerNode::publish_state()
{
  std_msgs::msg::String msg;
  msg.data = to_string(state_);
  state_pub_->publish(msg);
}

void TaskManagerNode::set_state(TaskState new_state, const std::string & reason)
{
  if (state_ == new_state) {
    return;
  }

  RCLCPP_INFO(
    this->get_logger(),
    "State transition: %s -> %s (%s)",
    to_string(state_).c_str(),
    to_string(new_state).c_str(),
    reason.c_str());

  state_ = new_state;
  publish_state();
}

void TaskManagerNode::handle_start_navigation(
  const std::shared_ptr<Trigger::Request>,
  std::shared_ptr<Trigger::Response> response)
{
  if (state_ == TaskState::NAVIGATING) {
    response->success = false;
    response->message = "Navigation already in progress.";
    return;
  }

  if (!nav_to_pose_client_->wait_for_action_server(1s)) {
    response->success = false;
    response->message = "navigate_to_pose action server not available.";
    return;
  }

  try_send_current_goal();
  response->success = true;
  response->message = "Navigation started.";
}

void TaskManagerNode::handle_cancel_navigation(
  const std::shared_ptr<Trigger::Request>,
  std::shared_ptr<Trigger::Response> response)
{
  if (state_ != TaskState::NAVIGATING || !goal_handle_) {
    response->success = false;
    response->message = "No active goal.";
    return;
  }

  nav_to_pose_client_->async_cancel_goal(goal_handle_);
  response->success = true;
  response->message = "Cancel request sent.";
}

void TaskManagerNode::handle_pause_navigation(
  const std::shared_ptr<Trigger::Request>,
  std::shared_ptr<Trigger::Response> response)
{
  if (state_ != TaskState::NAVIGATING || !goal_handle_) {
    response->success = false;
    response->message = "No active goal to pause.";
    return;
  }

  nav_to_pose_client_->async_cancel_goal(goal_handle_);
  set_state(TaskState::PAUSED, "paused by service");
  response->success = true;
  response->message = "Pause request sent.";
}

void TaskManagerNode::try_send_current_goal()
{
  if (current_index_ >= task_points_.size()) {
    current_index_ = 0;
  }

  auto goal_msg = NavigateToPose::Goal();
  goal_msg.pose = task_points_[current_index_].pose;
  goal_msg.pose.header.stamp = this->now();

  RCLCPP_INFO(
    this->get_logger(),
    "Sending goal [%s]: x=%.2f, y=%.2f",
    task_points_[current_index_].name.c_str(),
    goal_msg.pose.pose.position.x,
    goal_msg.pose.pose.position.y);

  rclcpp_action::Client<NavigateToPose>::SendGoalOptions options;
  options.goal_response_callback =
    std::bind(&TaskManagerNode::goal_response_callback, this, std::placeholders::_1);
  options.feedback_callback =
    std::bind(&TaskManagerNode::feedback_callback, this,
      std::placeholders::_1, std::placeholders::_2);
  options.result_callback =
    std::bind(&TaskManagerNode::result_callback, this, std::placeholders::_1);

  nav_to_pose_client_->async_send_goal(goal_msg, options);
}

void TaskManagerNode::goal_response_callback(const GoalHandleNavigateToPose::SharedPtr & goal_handle)
{
  if (!goal_handle) {
    set_state(TaskState::FAILED, "goal rejected");
    return;
  }

  goal_handle_ = goal_handle;
  set_state(TaskState::NAVIGATING, "goal accepted");
}

void TaskManagerNode::feedback_callback(
  GoalHandleNavigateToPose::SharedPtr,
  const std::shared_ptr<const NavigateToPose::Feedback> feedback)
{
  RCLCPP_INFO(
    this->get_logger(),
    "Distance remaining: %.3f",
    feedback->distance_remaining);
}

void TaskManagerNode::result_callback(const GoalHandleNavigateToPose::WrappedResult & result)
{
  switch (result.code) {
    case rclcpp_action::ResultCode::SUCCEEDED:
      set_state(TaskState::SUCCEEDED, "goal reached");
      if (auto_advance_) {
        ++current_index_;
        if (current_index_ < task_points_.size()) {
          try_send_current_goal();
          return;
        }
      }
      break;

    case rclcpp_action::ResultCode::CANCELED:
      set_state(TaskState::CANCELED, "goal canceled");
      break;

    case rclcpp_action::ResultCode::ABORTED:
    default:
      set_state(TaskState::FAILED, "goal aborted");
      break;
  }
}

}  // namespace task_manager