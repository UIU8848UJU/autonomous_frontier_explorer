#pragma once

#include <fstream>
#include <mutex>
#include <string>

#include "rclcpp/rclcpp.hpp"

namespace exploration_learning::collector
{

/// @brief JSONL append-only 数据集写入器。
///
/// DatasetWriter 只负责创建 episode 目录、写入 metadata 和追加 record。
/// 它不负责构建业务 record，也不负责训练、采样或数据格式转换。
class DatasetWriter
{
public:
  /// @brief 构造数据集写入器。
  /// @param logger 父节点 logger，用于输出落盘错误上下文。
  explicit DatasetWriter(const rclcpp::Logger & logger);

  /// @brief 配置并打开当前 episode 输出文件。
  /// @param output_dir 数据集根目录。
  /// @param episode_id 当前 episode 标识。
  /// @param flush_every_n 每写入多少条 record 后 flush。
  void open(
    const std::string & output_dir,
    const std::string & episode_id,
    int flush_every_n);

  /// @brief 写入 episode_metadata.json。
  /// @param metadata_json JSON object 字符串。
  void write_metadata(const std::string & metadata_json);

  /// @brief 追加一条 decision record JSONL。
  /// @param record_json 完整 JSON object 字符串。
  void append_decision_record(const std::string & record_json);

  /// @brief 刷新并关闭输出文件。
  void close();

  /// @brief 返回 decision_records.jsonl 完整路径。
  /// @return 当前 records 文件路径。
  const std::string & records_file_path() const;

  /// @brief 返回 episode_metadata.json 完整路径。
  /// @return 当前 metadata 文件路径。
  const std::string & metadata_file_path() const;

private:
  /// @brief 确认文件已打开。
  /// @return true 表示 records 文件可写。
  bool ensure_open() const;

  /// @brief 日志器。
  rclcpp::Logger logger_;

  /// @brief 当前 episode 目录路径。
  std::string episode_dir_;

  /// @brief decision_records.jsonl 路径。
  std::string records_file_path_;

  /// @brief episode_metadata.json 路径。
  std::string metadata_file_path_;

  /// @brief JSONL 输出流。
  std::ofstream records_file_;

  /// @brief 文件写入互斥锁。
  mutable std::mutex file_mutex_;

  /// @brief flush 阈值。
  int flush_every_n_{1};

  /// @brief 尚未 flush 的记录数量。
  std::size_t records_since_flush_{0U};
};

}  // namespace exploration_learning::collector
