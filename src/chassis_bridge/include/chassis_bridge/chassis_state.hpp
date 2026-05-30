#ifndef CHASSIS_BRIDGE__CHASSIS_STATE_HPP_
#define CHASSIS_BRIDGE__CHASSIS_STATE_HPP_

namespace chassis_bridge
{

/// @brief fake 底盘内部状态，保存当前速度指令和积分后的里程计位姿。
struct FakeChassisState
{
  double cmd_vx{0.0};
  double cmd_wz{0.0};
  double odom_x{0.0};
  double odom_y{0.0};
  double odom_theta{0.0};
};

}  // namespace chassis_bridge

#endif  // CHASSIS_BRIDGE__CHASSIS_STATE_HPP_
