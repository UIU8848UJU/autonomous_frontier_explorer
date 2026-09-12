#pragma once

namespace exploration_core
{

/// @brief 探索行为的通用运行状态。
enum class ExplorationStatus
{
    IDLE,
    RUNNING,
    COMPLETED,
    STUCK,
    FAILED
};

}  // namespace exploration_core
