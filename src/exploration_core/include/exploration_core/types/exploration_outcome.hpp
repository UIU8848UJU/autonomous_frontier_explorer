#pragma once

#include <optional>
#include <string>

#include "grid_map_core/types/grid_cell.hpp"

namespace exploration_core
{

/// @brief 执行器对上一次行为结果的反馈。
enum class ExplorationOutcomeType
{
    NAVIGATION_SUCCEEDED,
    NAVIGATION_FAILED,
    CANCELED
};

/// @brief 行为层接收的结果反馈，不绑定具体导航实现。
struct ExplorationOutcome
{
    ExplorationOutcomeType type{ExplorationOutcomeType::CANCELED};
    std::optional<grid_map_core::GridCell> goal;
    std::string detail;
};

}  // namespace exploration_core
