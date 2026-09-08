#include "nav2_map_saver.hpp"

#include <chrono>
#include <exception>
#include <filesystem>
#include <iomanip>
#include <sstream>
#include <utility>

namespace map_manager
{
namespace
{
rclcpp::Logger make_child_logger(const rclcpp::Logger & logger, const std::string & child_name)
{
  auto logger_copy = logger;
  return logger_copy.get_child(child_name);
}
}  // namespace

Nav2MapSaver::Nav2MapSaver(
  rclcpp::Node & node,
  const rclcpp::Logger & logger,
  Nav2MapSaverConfig config)
: logger_(make_child_logger(logger, "nav2_map_saver")),
  config_(std::move(config))
{
  save_map_client_ = node.create_client<SaveMap>(config_.map_saver_service_name);
  RCLCPP_INFO(
    logger_,
    "Nav2MapSaver ready: directory=%s, prefix=%s, service=%s, map_topic=%s",
    config_.save_directory.c_str(),
    config_.saved_map_prefix.c_str(),
    config_.map_saver_service_name.c_str(),
    config_.map_topic.c_str());
}

bool Nav2MapSaver::save_current_map(SaveResultCallback callback)
{
  if (saving_) {
    RCLCPP_WARN(logger_, "Skip map save because another save request is still running.");
    Nav2MapSaveResult result;
    result.message = "save_already_in_progress";
    if (callback) {
      callback(result);
    }
    return false;
  }

  if (!ensure_save_directory()) {
    Nav2MapSaveResult result;
    result.message = "save_directory_unavailable";
    if (callback) {
      callback(result);
    }
    return false;
  }

  const auto wait_timeout = std::chrono::duration<double>(config_.service_wait_timeout_sec);
  if (!save_map_client_->wait_for_service(
      std::chrono::duration_cast<std::chrono::nanoseconds>(wait_timeout)))
  {
    RCLCPP_ERROR(
      logger_,
      "Map saver service is not available: %s",
      config_.map_saver_service_name.c_str());
    Nav2MapSaveResult result;
    result.message = "map_saver_service_unavailable";
    if (callback) {
      callback(result);
    }
    return false;
  }

  auto request = std::make_shared<SaveMap::Request>();
  request->map_topic = config_.map_topic;
  request->map_url = generate_map_url();
  request->image_format = config_.image_format;
  request->map_mode = config_.map_mode;
  request->free_thresh = static_cast<float>(config_.free_thresh);
  request->occupied_thresh = static_cast<float>(config_.occupied_thresh);

  saving_ = true;
  RCLCPP_INFO(logger_, "Trigger Nav2 map save: map_url=%s", request->map_url.c_str());

  try {
    save_map_client_->async_send_request(
      request,
      [this, callback, map_url = request->map_url](
        rclcpp::Client<SaveMap>::SharedFuture future) {
        Nav2MapSaveResult result;
        result.map_url = map_url;
        try {
          const auto response = future.get();
          result.success = response && response->result;
          result.message = result.success ? "saved" : "map_saver_returned_false";
        } catch (const std::exception & ex) {
          result.message = ex.what();
        }

        saving_ = false;
        if (result.success) {
          RCLCPP_INFO(logger_, "Nav2 map save succeeded: %s", result.map_url.c_str());
        } else {
          RCLCPP_ERROR(
            logger_,
            "Nav2 map save failed: url=%s, reason=%s",
            result.map_url.c_str(),
            result.message.c_str());
        }

        if (callback) {
          callback(result);
        }
      });
  } catch (const std::exception & ex) {
    saving_ = false;
    Nav2MapSaveResult result;
    result.map_url = request->map_url;
    result.message = ex.what();
    RCLCPP_ERROR(
      logger_,
      "Failed to send Nav2 map save request: url=%s, reason=%s",
      result.map_url.c_str(),
      result.message.c_str());
    if (callback) {
      callback(result);
    }
    return false;
  }

  return true;
}

bool Nav2MapSaver::is_saving() const
{
  return saving_;
}

const Nav2MapSaverConfig & Nav2MapSaver::config() const
{
  return config_;
}

bool Nav2MapSaver::ensure_save_directory() const
{
  try {
    std::filesystem::create_directories(config_.save_directory);
    return std::filesystem::is_directory(config_.save_directory);
  } catch (const std::exception & ex) {
    RCLCPP_ERROR(
      logger_,
      "Failed to prepare save directory: directory=%s, error=%s",
      config_.save_directory.c_str(),
      ex.what());
    return false;
  }
}

std::string Nav2MapSaver::generate_map_url() const
{
  const auto now = std::chrono::system_clock::now();
  const auto time_t_now = std::chrono::system_clock::to_time_t(now);
  const auto milliseconds = std::chrono::duration_cast<std::chrono::milliseconds>(
    now.time_since_epoch()) % 1000;

  std::tm local_time{};
#if defined(_WIN32)
  localtime_s(&local_time, &time_t_now);
#else
  localtime_r(&time_t_now, &local_time);
#endif

  std::ostringstream file_name;
  file_name << config_.saved_map_prefix << "_"
            << std::put_time(&local_time, "%Y%m%d_%H%M%S")
            << "_" << std::setw(3) << std::setfill('0') << milliseconds.count();

  return (std::filesystem::path(config_.save_directory) / file_name.str()).string();
}

}  // namespace map_manager
