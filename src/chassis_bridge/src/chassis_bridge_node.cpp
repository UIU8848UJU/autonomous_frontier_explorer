#include "rclcpp/rclcpp.hpp"

/// @brief 真实底盘串口接入预留节点，当前 MVP 不实现串口读写。
class ChassisBridgeNode : public rclcpp::Node
{
public:
  /// @brief 构造真实底盘桥接预留节点。
  ChassisBridgeNode()
  : Node("chassis_bridge_node"),
    logger_(this->get_logger())
  {
    RCLCPP_INFO(logger_, "chassis_bridge_node is reserved for serial chassis integration");
  }

private:
  rclcpp::Logger logger_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ChassisBridgeNode>());
  rclcpp::shutdown();
  return 0;
}
