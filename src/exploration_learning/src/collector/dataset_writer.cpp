#include "exploration_learning/collector/dataset_writer.hpp"

#include <algorithm>
#include <filesystem>
#include <stdexcept>

namespace exploration_learning::collector
{

DatasetWriter::DatasetWriter(const rclcpp::Logger & logger)
: logger_(rclcpp::Logger(logger).get_child("dataset_writer"))
{
}

void DatasetWriter::open(
  const std::string & output_dir,
  const std::string & episode_id,
  int flush_every_n)
{
  close();

  flush_every_n_ = std::max(1, flush_every_n);
  const std::filesystem::path episode_dir =
    std::filesystem::path(output_dir) / episode_id;
  std::filesystem::create_directories(episode_dir);

  episode_dir_ = episode_dir.string();
  records_file_path_ = (episode_dir / "decision_records.jsonl").string();
  metadata_file_path_ = (episode_dir / "episode_metadata.json").string();

  std::lock_guard<std::mutex> lock(file_mutex_);
  records_file_.open(records_file_path_, std::ios::out | std::ios::app);
  if (!records_file_.is_open()) {
    RCLCPP_ERROR(
      logger_,
      "Failed to open decision record file: %s",
      records_file_path_.c_str());
    throw std::runtime_error("failed to open decision record file: " + records_file_path_);
  }
  records_since_flush_ = 0U;
}

void DatasetWriter::write_metadata(const std::string & metadata_json)
{
  std::ofstream metadata_file(metadata_file_path_, std::ios::out | std::ios::trunc);
  if (!metadata_file.is_open()) {
    RCLCPP_ERROR(
      logger_,
      "Failed to open episode metadata file: %s",
      metadata_file_path_.c_str());
    throw std::runtime_error("failed to open episode metadata file: " + metadata_file_path_);
  }

  metadata_file << metadata_json << "\n";
  metadata_file.close();
}

void DatasetWriter::append_decision_record(const std::string & record_json)
{
  std::lock_guard<std::mutex> lock(file_mutex_);
  if (!ensure_open()) {
    return;
  }

  records_file_ << record_json << "\n";
  ++records_since_flush_;
  if (records_since_flush_ >= static_cast<std::size_t>(flush_every_n_)) {
    records_file_.flush();
    records_since_flush_ = 0U;
  }
}

void DatasetWriter::close()
{
  std::lock_guard<std::mutex> lock(file_mutex_);
  if (records_file_.is_open()) {
    records_file_.flush();
    records_file_.close();
  }
  records_since_flush_ = 0U;
}

const std::string & DatasetWriter::records_file_path() const
{
  return records_file_path_;
}

const std::string & DatasetWriter::metadata_file_path() const
{
  return metadata_file_path_;
}

bool DatasetWriter::ensure_open() const
{
  if (records_file_.is_open()) {
    return true;
  }

  RCLCPP_ERROR(
    logger_,
    "Decision record file is not open: %s",
    records_file_path_.c_str());
  return false;
}

}  // namespace exploration_learning::collector
