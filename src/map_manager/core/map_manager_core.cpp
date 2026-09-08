#include "map_manager_core/map_manager_core.hpp"

#include <algorithm>
#include <cstddef>
#include <utility>

namespace map_manager
{

void MapManagerCore::update_map(MapSnapshot map)
{
  latest_map_ = std::move(map);
  map_stats_.width = latest_map_.width;
  map_stats_.height = latest_map_.height;
  map_stats_.resolution = latest_map_.resolution;
  map_stats_.unknown_ratio = calculate_unknown_ratio(latest_map_);
  const auto expected_cell_count =
    static_cast<std::size_t>(latest_map_.width) *
    static_cast<std::size_t>(latest_map_.height);
  map_stats_.valid = latest_map_.width > 0U && latest_map_.height > 0U &&
    latest_map_.data.size() == expected_cell_count;
}

void MapManagerCore::update_exploration_state(ExplorationState state)
{
  last_exploration_state_ = state.phase;
  last_exploration_detail_ = std::move(state.detail);
}

bool MapManagerCore::should_save(bool enable_auto_save) const
{
  return enable_auto_save && !save_requested_ && !save_succeeded_ && map_stats_.valid &&
         last_exploration_state_ == kExplorationCompleted;
}

void MapManagerCore::mark_completion_detected()
{
  completion_detected_ = true;
}

void MapManagerCore::mark_save_requested()
{
  save_requested_ = true;
}

void MapManagerCore::record_save_result(bool success, const std::string & map_url)
{
  save_succeeded_ = success;
  saved_map_url_ = map_url;
  if (!success) {
    save_requested_ = false;
  }
}

double MapManagerCore::calculate_unknown_ratio(const MapSnapshot & map)
{
  if (map.data.empty()) {
    return 1.0;
  }

  const auto unknown_count = std::count(map.data.begin(), map.data.end(), -1);
  return static_cast<double>(unknown_count) / static_cast<double>(map.data.size());
}

const MapSnapshot & MapManagerCore::latest_map() const
{
  return latest_map_;
}

const MapStatistics & MapManagerCore::map_statistics() const
{
  return map_stats_;
}

ExplorationPhase MapManagerCore::last_exploration_state() const
{
  return last_exploration_state_;
}

const std::string & MapManagerCore::last_exploration_detail() const
{
  return last_exploration_detail_;
}

const std::string & MapManagerCore::saved_map_url() const
{
  return saved_map_url_;
}

bool MapManagerCore::completion_detected() const
{
  return completion_detected_;
}

bool MapManagerCore::save_requested() const
{
  return save_requested_;
}

bool MapManagerCore::save_succeeded() const
{
  return save_succeeded_;
}

}  // namespace map_manager
