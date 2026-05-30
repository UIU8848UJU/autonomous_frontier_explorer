#ifndef CHASSIS_BRIDGE__ODOM_ESTIMATOR_HPP_
#define CHASSIS_BRIDGE__ODOM_ESTIMATOR_HPP_

#include "chassis_bridge/chassis_state.hpp"

namespace chassis_bridge
{

/// @brief 根据底盘线速度和角速度进行二维里程计积分。
class OdomEstimator
{
public:
  /// @brief 将速度指令积分到状态位姿。
  /// @param dt_sec 积分时间，单位秒；非正数时不更新。
  /// @param vx_mps 机器人本体系 x 方向线速度，单位 m/s。
  /// @param wz_radps 机器人绕 z 轴角速度，单位 rad/s。
  /// @param state 待更新的 fake 底盘状态。
  void integrate(double dt_sec, double vx_mps, double wz_radps, FakeChassisState & state) const;

  /// @brief 将角度归一化到 [-pi, pi)。
  /// @param angle_rad 输入角度，单位 rad。
  /// @return 归一化后的角度。
  static double normalize_angle(double angle_rad);
};

}  // namespace chassis_bridge

#endif  // CHASSIS_BRIDGE__ODOM_ESTIMATOR_HPP_
