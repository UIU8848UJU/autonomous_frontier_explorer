#pragma once

#include <string>

namespace exploration_learning
{

class DatasetWriter
{
public:
  void configure(const std::string & dataset_path);
  const std::string & dataset_path() const;

private:
  std::string dataset_path_;
};

}  // namespace exploration_learning
