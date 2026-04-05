#pragma once

// 类名宏可以在每个 cpp 文件里单独定义
#ifndef __CLASS_NAME__
#define __CLASS_NAME__ "UNKNOWN_CLASS"
#endif

/// @brief 日志
#define RCLCPP_INFO_WITH_CONTEXT(logger, fmt, ...) \
    RCLCPP_INFO(logger, "[%s::%s] " fmt, __CLASS_NAME__, __func__, ##__VA_ARGS__)

#define RCLCPP_DEBUG_WITH_CONTEXT(logger, fmt, ...) \
    RCLCPP_DEBUG(logger, "[%s::%s] " fmt, __CLASS_NAME__, __func__, ##__VA_ARGS__)

#define RCLCPP_WARN_WITH_CONTEXT(logger, fmt, ...) \
    RCLCPP_WARN(logger, "[%s::%s] " fmt, __CLASS_NAME__, __func__, ##__VA_ARGS__)

#define RCLCPP_ERROR_WITH_CONTEXT(logger, fmt, ...) \
    RCLCPP_ERROR(logger, "[%s::%s] " fmt, __CLASS_NAME__, __func__, ##__VA_ARGS__)

#define RCLCPP_FATAL_WITH_CONTEXT(logger, fmt, ...) \
    RCLCPP_FATAL(logger, "[%s::%s] " fmt, __CLASS_NAME__, __func__, ##__VA_ARGS__)

/// @brief 节点流日志
#define RCLCPP_INFO_THROTTLE_WITH_CONTEXT(logger, clock, period_ms, fmt, ...) \
    RCLCPP_INFO_THROTTLE(logger, clock, period_ms, "[%s::%s] " fmt, __CLASS_NAME__, __func__, ##__VA_ARGS__)

#define RCLCPP_DEBUG_THROTTLE_WITH_CONTEXT(logger, clock, period_ms, fmt, ...) \
    RCLCPP_DEBUG_THROTTLE(logger, clock, period_ms, "[%s::%s] " fmt, __CLASS_NAME__, __func__, ##__VA_ARGS__)

#define RCLCPP_WARN_THROTTLE_WITH_CONTEXT(logger, clock, period_ms, fmt, ...) \
    RCLCPP_WARN_THROTTLE(logger, clock, period_ms, "[%s::%s] " fmt, __CLASS_NAME__, __func__, ##__VA_ARGS__)

#define RCLCPP_ERROR_THROTTLE_WITH_CONTEXT(logger, clock, period_ms, fmt, ...) \
    RCLCPP_ERROR_THROTTLE(logger, clock, period_ms, "[%s::%s] " fmt, __CLASS_NAME__, __func__, ##__VA_ARGS__)

#define RCLCPP_FATAL_THROTTLE_WITH_CONTEXT(logger, clock, period_ms, fmt, ...) \
    RCLCPP_FATAL_THROTTLE(logger, clock, period_ms, "[%s::%s] " fmt, __CLASS_NAME__, __func__, ##__VA_ARGS__)

// /// @brief 系统时钟日志 ，humble版本一下没有，后续看版本做if判断
// #define RCLCPP_INFO_THROTTLE_WITH_CONTEXT_WALL(logger, clock, period_ms, fmt, ...) \
//     RCLCPP_INFO_THROTTLE_WALL(logger, clock, period_ms, "[%s::%s] " fmt, __CLASS_NAME__, __func__, ##__VA_ARGS__)

// #define RCLCPP_DEBUG_THROTTLE_WITH_CONTEXT_WALL(logger, clock, period_ms, fmt, ...) \
//     RCLCPP_DEBUG_THROTTLE_WALL(logger, clock, period_ms, "[%s::%s] " fmt, __CLASS_NAME__, __func__, ##__VA_ARGS__)

// #define RCLCPP_WARN_THROTTLE_WITH_CONTEXT_WALL(logger, clock, period_ms, fmt, ...) \
//     RCLCPP_WARN_THROTTLE_WALL(logger, clock, period_ms, "[%s::%s] " fmt, __CLASS_NAME__, __func__, ##__VA_ARGS__)

// #define RCLCPP_ERROR_THROTTLE_WITH_CONTEXT_WALL(logger, clock, period_ms, fmt, ...) \
//     RCLCPP_ERROR_THROTTLE_WALL(logger, clock, period_ms, "[%s::%s] " fmt, __CLASS_NAME__, __func__, ##__VA_ARGS__)

// #define RCLCPP_FATAL_THROTTLE_WITH_CONTEXT_WALL(logger, clock, period_ms, fmt, ...) \
//     RCLCPP_FATAL_THROTTLE_WALL(logger, clock, period_ms, "[%s::%s] " fmt, __CLASS_NAME__, __func__, ##__VA_ARGS__)

/// @brief 一次性日志（ONCE） 
#define LOG_DEBUG_ONCE(logger, fmt, ...) \
    RCLCPP_DEBUG_ONCE(logger, "[%s::%s] " fmt, __CLASS_NAME__, __func__, ##__VA_ARGS__)

#define LOG_INFO_ONCE(logger, fmt, ...) \
    RCLCPP_INFO_ONCE(logger, "[%s::%s] " fmt, __CLASS_NAME__, __func__, ##__VA_ARGS__)

#define LOG_WARN_ONCE(logger, fmt, ...) \
    RCLCPP_WARN_ONCE(logger, "[%s::%s] " fmt, __CLASS_NAME__, __func__, ##__VA_ARGS__)

#define LOG_ERROR_ONCE(logger, fmt, ...) \
    RCLCPP_ERROR_ONCE(logger, "[%s::%s] " fmt, __CLASS_NAME__, __func__, ##__VA_ARGS__)

#define LOG_FATAL_ONCE(logger, fmt, ...) \
    RCLCPP_FATAL_ONCE(logger, "[%s::%s] " fmt, __CLASS_NAME__, __func__, ##__VA_ARGS__)
