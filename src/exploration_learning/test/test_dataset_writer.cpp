#include "exploration_learning/collector/dataset_writer.hpp"

#include <filesystem>
#include <fstream>
#include <sstream>

#include "gtest/gtest.h"

namespace exploration_learning::collector
{
namespace
{

std::string read_file(const std::filesystem::path & path)
{
  std::ifstream file(path);
  std::ostringstream content;
  content << file.rdbuf();
  return content.str();
}

}  // namespace

TEST(DatasetWriterTest, WritesMetadataAndDecisionRecords)
{
  const auto output_dir =
    std::filesystem::temp_directory_path() / "exploration_learning_writer_test";
  std::filesystem::remove_all(output_dir);

  DatasetWriter writer(rclcpp::get_logger("dataset_writer_test"));
  writer.open(output_dir.string(), "episode_001", 2);
  writer.write_metadata("{\"episode_id\":\"episode_001\"}");
  writer.append_decision_record("{\"decision_id\":0}");
  writer.append_decision_record("{\"decision_id\":1}");
  writer.close();

  const auto metadata_path = output_dir / "episode_001" / "episode_metadata.json";
  const auto records_path = output_dir / "episode_001" / "decision_records.jsonl";

  ASSERT_TRUE(std::filesystem::exists(metadata_path));
  ASSERT_TRUE(std::filesystem::exists(records_path));
  EXPECT_EQ(read_file(metadata_path), "{\"episode_id\":\"episode_001\"}\n");
  EXPECT_EQ(read_file(records_path), "{\"decision_id\":0}\n{\"decision_id\":1}\n");

  std::filesystem::remove_all(output_dir);
}

}  // namespace exploration_learning::collector
