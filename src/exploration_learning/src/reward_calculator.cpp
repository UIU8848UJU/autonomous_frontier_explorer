#include "exploration_learning/reward/reward_calculator.hpp"

namespace exploration_learning
{

double RewardCalculator::calculate(const RewardInput & input) const
{
  double reward = input.explored_area_delta - (0.05 * input.path_length_delta);
  if (input.reached_goal) {
    reward += 1.0;
  }
  if (input.collision) {
    reward -= 5.0;
  }
  return reward;
}

}  // namespace exploration_learning
