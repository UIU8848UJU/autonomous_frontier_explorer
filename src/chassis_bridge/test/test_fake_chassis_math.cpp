#include "chassis_bridge/odom_estimator.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <vector>

namespace
{

using chassis_bridge::FakeChassisState;
using chassis_bridge::OdomEstimator;

struct IntegrateCase
{
  double dt_sec;
  double vx_mps;
  double wz_radps;
  double expected_x;
  double expected_y;
  double expected_theta;
};

TEST(FakeChassisMathTest, IntegratesTableDrivenMotionData)
{
  const std::vector<IntegrateCase> test_data = {
    {1.0, 0.2, 0.0, 0.2, 0.0, 0.0},
    {2.0, 0.0, 0.5, 0.0, 0.0, 1.0},
    {1.0, 0.4, 1.0, 0.4 * std::cos(0.5), 0.4 * std::sin(0.5), 1.0},
  };

  const OdomEstimator estimator;
  for (const auto & test_case : test_data) {
    FakeChassisState state;
    estimator.integrate(test_case.dt_sec, test_case.vx_mps, test_case.wz_radps, state);

    EXPECT_NEAR(state.odom_x, test_case.expected_x, 1.0e-9);
    EXPECT_NEAR(state.odom_y, test_case.expected_y, 1.0e-9);
    EXPECT_NEAR(state.odom_theta, test_case.expected_theta, 1.0e-9);
  }
}

TEST(FakeChassisMathTest, AccumulatesSequentialMotionData)
{
  const OdomEstimator estimator;
  FakeChassisState state;

  estimator.integrate(1.0, 0.3, 0.0, state);
  estimator.integrate(1.0, 0.3, 0.0, state);

  EXPECT_NEAR(state.odom_x, 0.6, 1.0e-9);
  EXPECT_NEAR(state.odom_y, 0.0, 1.0e-9);
  EXPECT_NEAR(state.odom_theta, 0.0, 1.0e-9);
}

TEST(FakeChassisMathTest, IgnoresNonPositiveDt)
{
  const OdomEstimator estimator;
  FakeChassisState state;
  state.odom_x = 1.0;
  state.odom_y = 2.0;
  state.odom_theta = 0.3;

  estimator.integrate(0.0, 1.0, 1.0, state);
  estimator.integrate(-1.0, 1.0, 1.0, state);

  EXPECT_DOUBLE_EQ(state.odom_x, 1.0);
  EXPECT_DOUBLE_EQ(state.odom_y, 2.0);
  EXPECT_DOUBLE_EQ(state.odom_theta, 0.3);
}

TEST(FakeChassisMathTest, NormalizesAngle)
{
  constexpr double kPi = 3.14159265358979323846;

  EXPECT_NEAR(OdomEstimator::normalize_angle(kPi), -kPi, 1.0e-12);
  EXPECT_NEAR(OdomEstimator::normalize_angle(3.0 * kPi), -kPi, 1.0e-12);
  EXPECT_NEAR(OdomEstimator::normalize_angle(-3.0 * kPi), -kPi, 1.0e-12);
}

}  // namespace
