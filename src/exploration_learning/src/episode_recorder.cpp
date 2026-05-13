#include "exploration_learning/data_collection/episode_recorder.hpp"

namespace exploration_learning
{

void EpisodeRecorder::start_episode(const std::string & episode_id)
{
  episode_id_ = episode_id;
  transition_count_ = 0U;
  active_ = true;
}

void EpisodeRecorder::finish_episode()
{
  active_ = false;
}

bool EpisodeRecorder::active() const
{
  return active_;
}

const std::string & EpisodeRecorder::episode_id() const
{
  return episode_id_;
}

std::size_t EpisodeRecorder::transition_count() const
{
  return transition_count_;
}

}  // namespace exploration_learning
