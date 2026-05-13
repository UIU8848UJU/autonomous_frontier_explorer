#pragma once

#include <cstddef>
#include <string>

namespace exploration_learning
{

class EpisodeRecorder
{
public:
  void start_episode(const std::string & episode_id);
  void finish_episode();

  bool active() const;
  const std::string & episode_id() const;
  std::size_t transition_count() const;

private:
  bool active_{false};
  std::string episode_id_;
  std::size_t transition_count_{0U};
};

}  // namespace exploration_learning
