#pragma once

namespace exploration_learning
{

struct RewardInput
{
  double explored_area_delta{0.0};
  double path_length_delta{0.0};
  bool reached_goal{false};
  bool collision{false};
};

class RewardCalculator
{
public:
  double calculate(const RewardInput & input) const;
};

}  // namespace exploration_learning
