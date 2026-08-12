#include "navigation_core/single_goal_gate.hpp"

namespace frontier_explorer
{
namespace navigation_core
{

bool SingleGoalGate::tryAcquire()
{
    std::lock_guard<std::mutex> lock(mutex_);
    if (active_) {
        return false;
    }
    active_ = true;
    return true;
}

void SingleGoalGate::release()
{
    std::lock_guard<std::mutex> lock(mutex_);
    active_ = false;
}

bool SingleGoalGate::isActive() const
{
    std::lock_guard<std::mutex> lock(mutex_);
    return active_;
}

}  // namespace navigation_core
}  // namespace frontier_explorer
