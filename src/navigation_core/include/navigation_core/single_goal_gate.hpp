#pragma once

#include <atomic>

namespace navigation
{
namespace navigation_core
{

/// @brief: 保护同一时间只有一个导航目标处于活动状态。
class SingleGoalGate
{
public:
    /// @brief: 尝试获取导航目标占用权。
    /// @return: true 表示获取成功，false 表示已有目标处于活动状态。
    bool tryAcquire();

    /// @brief: 释放当前导航目标占用权。
    void release();

    /// @brief: 查询当前是否存在活动导航目标。
    /// @return: true 表示存在活动目标。
    bool isActive() const;

private:
    std::atomic_bool active_{false};
};

}  // namespace navigation_core
}  // namespace navigation
