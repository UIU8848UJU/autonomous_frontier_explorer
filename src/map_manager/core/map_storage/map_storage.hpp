#pragma once

#include <functional>
#include <memory>
#include <string>

#include "nav2_msgs/srv/save_map.hpp"
#include "rclcpp/rclcpp.hpp"

namespace map_manager
{

/// @brief: 地图保存参数，集中描述保存目录、文件命名和 Nav2 map_saver 服务选项
struct MapStorageConfig
{
    /// @brief: 地图保存目录
    std::string save_directory{"/home/xxx/mk_nav2/maps"};
    /// @brief: 保存文件名前缀
    std::string saved_map_prefix{"explore_map"};
    /// @brief: Nav2 map_saver 的服务名称
    std::string map_saver_service_name{"/map_saver/save_map"};
    /// @brief: map_saver 读取地图时使用的 topic
    std::string map_topic{"/map"};
    /// @brief: 保存图像格式
    std::string image_format{"pgm"};
    /// @brief: Nav2 地图保存模式
    std::string map_mode{"trinary"};
    /// @brief: 空闲阈值
    double free_thresh{0.25};
    /// @brief: 占用阈值
    double occupied_thresh{0.65};
    /// @brief: 等待 map_saver 服务可用的最长时间，单位秒
    double service_wait_timeout_sec{2.0};
};

/// @brief: 地图保存结果
struct MapSaveResult
{
    /// @brief: 是否保存成功
    bool success{false};
    /// @brief: 本次保存请求使用的 map_url
    std::string map_url;
    /// @brief: 失败或成功说明
    std::string message;
};

/// @brief: 封装地图输出目录、文件名生成和 Nav2 map_saver 服务调用
class MapStorage
{
public:
    using SaveMap = nav2_msgs::srv::SaveMap;
    using SaveResultCallback = std::function<void(const MapSaveResult &)>;

    /// @brief: 构造地图存储辅助类
    /// @param node 用于创建服务客户端的 ROS2 节点
    /// @param logger 父 logger，会在内部派生 map_storage 子 logger
    /// @param config 地图保存配置
    MapStorage(rclcpp::Node & node, const rclcpp::Logger & logger, MapStorageConfig config);

    /// @brief: 异步请求保存当前地图
    /// @param callback 保存完成后的回调
    /// @return: 请求是否成功发出；如果服务不可用或已有保存请求在进行中则返回 false
    bool save_current_map(SaveResultCallback callback);

    /// @brief: 判断当前是否已有保存请求在执行
    /// @return: true 表示正在等待 map_saver 返回
    bool is_saving() const;

    /// @brief: 读取当前保存配置
    /// @return: 保存配置的只读引用
    const MapStorageConfig & config() const;

private:
    /// @brief: 确保保存目录存在
    /// @return: true 表示目录存在或创建成功
    bool ensure_save_directory() const;

    /// @brief: 生成本次保存使用的 map_url
    /// @return: 不带扩展名的地图保存路径
    std::string generate_map_url() const;

    rclcpp::Logger logger_;
    MapStorageConfig config_;
    rclcpp::Client<SaveMap>::SharedPtr save_map_client_;
    bool saving_{false};
};

}  // namespace map_manager
