#pragma once

#include <functional>
#include <memory>
#include <string>

#include "nav2_msgs/srv/save_map.hpp"
#include "rclcpp/rclcpp.hpp"

namespace map_manager
{

/// @brief Nav2 map_saver 服务适配器配置。
struct Nav2MapSaverConfig
{
  std::string save_directory{"/home/xxx/mk_nav2/maps"};
  std::string saved_map_prefix{"explore_map"};
  std::string map_saver_service_name{"/map_saver/save_map"};
  std::string map_topic{"/map"};
  std::string image_format{"pgm"};
  std::string map_mode{"trinary"};
  double free_thresh{0.25};
  double occupied_thresh{0.65};
  double service_wait_timeout_sec{2.0};
};

/// @brief Nav2 SaveMap 响应适配后的结果。
struct Nav2MapSaveResult
{
  bool success{false};
  std::string map_url;
  std::string message;
};

/// @brief 将 map_manager 的保存请求适配到 Nav2 map_saver 服务。
class Nav2MapSaver
{
public:
  using SaveMap = nav2_msgs::srv::SaveMap;
  using SaveResultCallback = std::function<void (const Nav2MapSaveResult &)>;

  Nav2MapSaver(rclcpp::Node & node, const rclcpp::Logger & logger, Nav2MapSaverConfig config);

  /// @brief 异步请求 Nav2 保存当前地图。
  /// @details 每次调用都通过 callback 返回且仅返回一个最终结果，包括同步拒绝。
  /// @return 请求是否发出；服务不可用或已有请求时返回 false。
  bool save_current_map(SaveResultCallback callback);

  bool is_saving() const;
  const Nav2MapSaverConfig & config() const;

private:
  bool ensure_save_directory() const;
  std::string generate_map_url() const;

  rclcpp::Logger logger_;
  Nav2MapSaverConfig config_;
  rclcpp::Client<SaveMap>::SharedPtr save_map_client_;
  bool saving_{false};
};

}  // namespace map_manager
