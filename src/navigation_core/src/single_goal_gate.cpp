#include "navigation_core/single_goal_gate.hpp"

namespace navigation
{
namespace navigation_core
{

bool SingleGoalGate::tryAcquire()
{
    bool expected = false;
    return active_.compare_exchange_strong(expected, true);
}

void SingleGoalGate::release()
{
    active_.store(false);
}

bool SingleGoalGate::isActive() const
{
    return active_.load();
}

}  // namespace navigation_core
}  // namespace navigation
