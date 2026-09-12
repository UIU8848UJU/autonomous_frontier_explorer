#pragma once

#include <optional>
#include <string>

#include "grid_map_core/types/grid_cell.hpp"

namespace exploration_core
{

/// @brief 行为策略对执行器发出的下一步意图。
enum class ExplorationDecisionType
{
    WAIT,
    NAVIGATE,
    COMPLETED,
    STUCK,
    FAILED
};

/// @brief 探索策略的输出，不包含 ROS Action 或 Nav2 消息。
struct ExplorationDecision
{
    ExplorationDecisionType type{ExplorationDecisionType::WAIT};
    std::optional<grid_map_core::GridCell> goal;
    std::string detail;
};

}  // namespace exploration_core
