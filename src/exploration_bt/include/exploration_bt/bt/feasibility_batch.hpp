#pragma once

#include <algorithm>
#include <cstddef>

namespace exploration
{

/// @brief 返回首批可行性检查的结束下标。
inline std::size_t initial_feasibility_batch_end(
    std::size_t candidate_count,
    std::size_t top_k)
{
    return std::min(candidate_count, std::max<std::size_t>(1U, top_k));
}

/// @brief 当前批次全部不可行时，计算下一批的结束下标。
inline std::size_t expand_feasibility_batch_end(
    std::size_t current_end,
    std::size_t candidate_count,
    std::size_t top_k)
{
    return std::min(
        candidate_count,
        current_end + std::max<std::size_t>(1U, top_k));
}

}  // 命名空间 exploration
