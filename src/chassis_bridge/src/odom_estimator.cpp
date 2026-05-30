#include "chassis_bridge/odom_estimator.hpp"

#include <cmath>

namespace chassis_bridge
{

void OdomEstimator::integrate(
  const double dt_sec, const double vx_mps, const double wz_radps, FakeChassisState & state) const
{
  if (dt_sec <= 0.0) {
    return;
  }

  const double delta_theta = wz_radps * dt_sec;
  const double mid_theta = state.odom_theta + 0.5 * delta_theta;
  state.odom_x += vx_mps * std::cos(mid_theta) * dt_sec;
  state.odom_y += vx_mps * std::sin(mid_theta) * dt_sec;
  state.odom_theta = normalize_angle(state.odom_theta + delta_theta);
}

double OdomEstimator::normalize_angle(double angle_rad)
{
  constexpr double kPi = 3.14159265358979323846;
  constexpr double kTwoPi = 2.0 * kPi;

  while (angle_rad >= kPi) {
    angle_rad -= kTwoPi;
  }
  while (angle_rad < -kPi) {
    angle_rad += kTwoPi;
  }
  return angle_rad;
}

}  // namespace chassis_bridge
