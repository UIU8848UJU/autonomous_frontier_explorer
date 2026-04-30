#pragma once

#include "rclcpp/rclcpp.hpp"

namespace map_manager
{

/// @brief: 地图融合预留接口；当前版本不实现多机器人地图融合算法
class MapMerger
{
public:
    /// @brief: 构造地图融合预留类
    /// @param logger 父 logger，会在内部派生 map_merger 子 logger
    explicit MapMerger(const rclcpp::Logger & logger)
    : logger_(make_child_logger(logger))
    {
    }

private:
    /// @brief: 基于父 logger 创建 map_merger 子 logger
    /// @param logger 父 logger
    /// @return: map_merger 子 logger
    static rclcpp::Logger make_child_logger(const rclcpp::Logger & logger)
    {
        auto logger_copy = logger;
        return logger_copy.get_child("map_merger");
    }

    rclcpp::Logger logger_;
};

}  // namespace map_manager
