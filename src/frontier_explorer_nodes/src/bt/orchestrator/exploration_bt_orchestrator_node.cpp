#include "frontier_explorer_nodes/nodes/exploration_bt_orchestrator_node.hpp"

#include <algorithm>
#include <chrono>
#include <functional>
#include <mutex>

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "ament_index_cpp/get_package_prefix.hpp"
#include "frontier_explorer_nodes/nodes/exploration_bt_defaults.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "robot_interfaces/srv/mark_frontier_failed.hpp"

namespace frontier_explorer
{
namespace
{
constexpr int kStatePublisherDepth = 10;
}

ExplorationBtOrchestratorNode::ExplorationBtOrchestratorNode(
    const rclcpp::NodeOptions & options)
: Node("exploration_bt_orchestrator_node", options)
{
    const auto default_bt_xml = ament_index_cpp::get_package_share_directory(
        "frontier_explorer_nodes") + "/behavior_trees/exploration_tree.xml";
    const auto default_bt_plugin = ament_index_cpp::get_package_prefix(
        "frontier_explorer_nodes") + "/lib/libfrontier_explorer_bt_nodes.so";

    declare_parameter<std::string>("bt_xml_file", default_bt_xml);
    declare_parameter<std::vector<std::string>>(
        "bt_plugin_libraries",
        std::vector<std::string>{default_bt_plugin});
    declare_parameter<std::string>(
        "frontier_candidates_service",
        exploration_bt_defaults::kFrontierCandidatesService);
    declare_parameter<std::string>(
        "mark_failed_service",
        exploration_bt_defaults::kMarkFailedService);
    declare_parameter<std::string>(
        "goal_feasibility_service",
        exploration_bt_defaults::kGoalFeasibilityService);
    declare_parameter<std::string>(
        "navigation_action",
        exploration_bt_defaults::kNavigationAction);
    declare_parameter<std::string>(
        "navigate_to_pose_action",
        exploration_bt_defaults::kNavigationAction);
    declare_parameter<double>("tick_period_sec", exploration_bt_defaults::kTickPeriodSec);
    declare_parameter<double>(
        "service_retry_delay_sec",
        exploration_bt_defaults::kServiceRetryDelaySec);
    declare_parameter<int>(
        "max_frontier_candidates",
        exploration_bt_defaults::kMaxFrontierCandidates);
    declare_parameter<int>(
        "max_feasibility_recoverable_retries",
        exploration_bt_defaults::kMaxFeasibilityRecoverableRetries);
    declare_parameter<double>(
        "feasible_path_length_weight",
        exploration_bt_defaults::kFeasiblePathLengthWeight);

    bt_xml_file_ = get_parameter("bt_xml_file").as_string();
    bt_plugin_libraries_ = get_parameter("bt_plugin_libraries").as_string_array();
    tick_period_sec_ = std::max(0.02, get_parameter("tick_period_sec").as_double());

    context_ = std::make_shared<ExplorationBtContext>();
    context_->node = this;
    context_->logger = get_logger();
    context_->service_retry_delay_sec =
        std::max(0.1, get_parameter("service_retry_delay_sec").as_double());
    context_->max_frontier_candidates = static_cast<uint32_t>(
        std::max(1, static_cast<int>(get_parameter("max_frontier_candidates").as_int())));
    context_->max_feasibility_recoverable_retries = static_cast<uint32_t>(
        std::max(
            0,
            static_cast<int>(get_parameter("max_feasibility_recoverable_retries").as_int())));
    context_->feasible_path_length_weight =
        std::max(0.0, get_parameter("feasible_path_length_weight").as_double());
    context_->get_candidates_client =
        create_client<robot_interfaces::srv::GetFrontierCandidates>(
            get_parameter("frontier_candidates_service").as_string());
    context_->mark_failed_client =
        create_client<robot_interfaces::srv::MarkFrontierFailed>(
            get_parameter("mark_failed_service").as_string());
    context_->feasibility_client =
        create_client<robot_interfaces::srv::CheckGoalFeasibility>(
            get_parameter("goal_feasibility_service").as_string());
    context_->nav_client =
        rclcpp_action::create_client<ExplorationBtContext::NavigateToPose>(
            this,
            get_parameter("navigation_action").as_string());

    load_bt_plugins();

    state_pub_ = create_publisher<robot_interfaces::msg::ExplorationState>(
        "/exploration_state",
        rclcpp::QoS(rclcpp::KeepLast(kStatePublisherDepth)).reliable());

    start_srv_ = create_service<std_srvs::srv::Trigger>(
        "~/start_exploration",
        std::bind(
            &ExplorationBtOrchestratorNode::handle_start,
            this,
            std::placeholders::_1,
            std::placeholders::_2));
    stop_srv_ = create_service<std_srvs::srv::Trigger>(
        "~/stop_exploration",
        std::bind(
            &ExplorationBtOrchestratorNode::handle_stop,
            this,
            std::placeholders::_1,
            std::placeholders::_2));
    pause_srv_ = create_service<std_srvs::srv::Trigger>(
        "~/pause_exploration",
        std::bind(
            &ExplorationBtOrchestratorNode::handle_stop,
            this,
            std::placeholders::_1,
            std::placeholders::_2));
    resume_srv_ = create_service<std_srvs::srv::Trigger>(
        "~/resume_exploration",
        std::bind(
            &ExplorationBtOrchestratorNode::handle_start,
            this,
            std::placeholders::_1,
            std::placeholders::_2));

    timer_ = create_wall_timer(
        std::chrono::duration<double>(tick_period_sec_),
        std::bind(&ExplorationBtOrchestratorNode::tick_tree, this));

    publish_state("IDLE");
    RCLCPP_INFO(
        get_logger(),
        "ExplorationBtOrchestratorNode started: bt_xml=%s",
        bt_xml_file_.c_str());
}

void ExplorationBtOrchestratorNode::load_bt_plugins()
{
    for (const auto & plugin_library : bt_plugin_libraries_) {
        if (plugin_library.empty()) {
            continue;
        }
        factory_.registerFromPlugin(plugin_library);
        RCLCPP_INFO(
            get_logger(),
            "Loaded Exploration BT plugin library: %s",
            plugin_library.c_str());
    }
}

void ExplorationBtOrchestratorNode::handle_start(
    const std::shared_ptr<std_srvs::srv::Trigger::Request>,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
    {
        std::lock_guard<std::mutex> lock(context_->mutex);
        context_->exploration_complete = false;
        context_->navigation_failed = false;
        context_->stop_requested = false;
        context_->current_goal.reset();
        context_->last_detail = "BT_STARTED";
    }

    try {
        blackboard_ = BT::Blackboard::create();
        blackboard_->set<std::shared_ptr<ExplorationBtContext>>(
            kExplorationBtContextBlackboardKey,
            context_);
        tree_ = factory_.createTreeFromFile(bt_xml_file_, blackboard_);
    } catch (const std::exception & ex) {
        state_ = ExplorationBtOrchestratorState::FAILED;
        response->success = false;
        response->message = std::string("failed to load exploration BT: ") + ex.what();
        publish_state(response->message);
        return;
    }

    state_ = ExplorationBtOrchestratorState::RUNNING;
    response->success = true;
    response->message = "exploration BT started";
    publish_state("BT_RUNNING");
}

void ExplorationBtOrchestratorNode::handle_stop(
    const std::shared_ptr<std_srvs::srv::Trigger::Request>,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
    {
        std::lock_guard<std::mutex> lock(context_->mutex);
        context_->stop_requested = true;
        context_->last_detail = "BT_CANCELLED";
    }
    if (tree_.has_value()) {
        tree_->haltTree();
        tree_.reset();
    }
    state_ = ExplorationBtOrchestratorState::CANCELLED;
    response->success = true;
    response->message = "exploration BT stopped";
    publish_state("BT_CANCELLED");
}

void ExplorationBtOrchestratorNode::tick_tree()
{
    if (state_ != ExplorationBtOrchestratorState::RUNNING || !tree_.has_value()) {
        publish_state(current_detail());
        return;
    }

    const auto status = tree_->tickRoot();
    publish_state(current_detail());

    if (status == BT::NodeStatus::RUNNING) {
        return;
    }

    if (is_exploration_complete()) {
        state_ = ExplorationBtOrchestratorState::COMPLETED;
        tree_->haltTree();
        tree_.reset();
        publish_state("BT_COMPLETED");
        return;
    }

    if (status == BT::NodeStatus::SUCCESS) {
        try {
            blackboard_ = BT::Blackboard::create();
            blackboard_->set<std::shared_ptr<ExplorationBtContext>>(
                kExplorationBtContextBlackboardKey,
                context_);
            tree_ = factory_.createTreeFromFile(bt_xml_file_, blackboard_);
        } catch (const std::exception & ex) {
            state_ = ExplorationBtOrchestratorState::FAILED;
            publish_state(std::string("BT_RELOAD_FAILED: ") + ex.what());
        }
        return;
    }

    state_ = ExplorationBtOrchestratorState::FAILED;
    tree_->haltTree();
    tree_.reset();
    publish_state(current_detail());
}

bool ExplorationBtOrchestratorNode::is_exploration_complete() const
{
    std::lock_guard<std::mutex> lock(context_->mutex);
    return context_->exploration_complete;
}

std::string ExplorationBtOrchestratorNode::current_detail() const
{
    std::lock_guard<std::mutex> lock(context_->mutex);
    return context_->last_detail;
}

void ExplorationBtOrchestratorNode::publish_state(const std::string & detail)
{
    if (!state_pub_) {
        return;
    }

    robot_interfaces::msg::ExplorationState msg;
    msg.stamp = now();
    msg.detail = detail;
    switch (state_) {
        case ExplorationBtOrchestratorState::IDLE:
            msg.state = msg.IDLE;
            break;
        case ExplorationBtOrchestratorState::RUNNING:
            msg.state = msg.RUNNING;
            break;
        case ExplorationBtOrchestratorState::COMPLETED:
            msg.state = msg.COMPLETED;
            break;
        case ExplorationBtOrchestratorState::FAILED:
            msg.state = msg.STUCK;
            break;
        case ExplorationBtOrchestratorState::CANCELLED:
            msg.state = msg.STOPPED;
            break;
    }
    state_pub_->publish(msg);
}

}  // namespace frontier_explorer
