#include "exploration_learning/dataset/dataset_writer.hpp"

namespace exploration_learning
{

void DatasetWriter::configure(const std::string & dataset_path)
{
  dataset_path_ = dataset_path;
}

const std::string & DatasetWriter::dataset_path() const
{
  return dataset_path_;
}

}  // namespace exploration_learning
