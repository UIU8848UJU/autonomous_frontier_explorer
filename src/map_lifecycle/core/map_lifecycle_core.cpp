#include "map_lifecycle_core/map_lifecycle_core.hpp"

#include <algorithm>
#include <cstddef>
#include <utility>

namespace map_lifecycle
{

void MapLifecycleCore::update_map(grid_map_core::GridMap map)
{
  latest_map_ = std::move(map);
  map_stats_.width = latest_map_.width;
  map_stats_.height = latest_map_.height;
  map_stats_.resolution = latest_map_.resolution;
  map_stats_.unknown_ratio = calculate_unknown_ratio(latest_map_);
  map_stats_.explored_ratio = 1.0 - map_stats_.unknown_ratio;
  map_stats_.free_ratio = 0.0;
  map_stats_.occupied_ratio = 0.0;
  if (!latest_map_.data.empty()) {
    std::size_t free_count = 0U;
    std::size_t occupied_count = 0U;
    for (const auto cell : latest_map_.data) {
      if (cell == 0) {
        ++free_count;
      } else if (cell >= 50) {
        ++occupied_count;
      }
    }
    const auto cell_count = static_cast<double>(latest_map_.data.size());
    map_stats_.free_ratio = static_cast<double>(free_count) / cell_count;
    map_stats_.occupied_ratio = static_cast<double>(occupied_count) / cell_count;
  }
  map_stats_.valid = latest_map_.isReady();

  if (state_ != MapLifecycleState::SAVING && state_ != MapLifecycleState::SAVED) {
    state_ = map_stats_.valid ?
      (completion_detected_ ? MapLifecycleState::READY : MapLifecycleState::ACTIVE) :
      MapLifecycleState::EMPTY;
  }
}

void MapLifecycleCore::handle_exploration_completed(const ExplorationCompletedEvent & event)
{
  (void)event;
  completion_detected_ = true;
  if (map_stats_.valid && state_ != MapLifecycleState::SAVED) {
    state_ = MapLifecycleState::READY;
  }
}

bool MapLifecycleCore::should_save(bool enable_auto_save) const
{
  return enable_auto_save && completion_detected_ && map_stats_.valid &&
         (state_ == MapLifecycleState::READY || state_ == MapLifecycleState::FAILED);
}

void MapLifecycleCore::mark_save_requested()
{
  if (state_ == MapLifecycleState::READY || state_ == MapLifecycleState::FAILED) {
    state_ = MapLifecycleState::SAVING;
  }
}

void MapLifecycleCore::record_save_result(bool success, const std::string & map_url)
{
  if (success) {
    saved_map_url_ = map_url;
    state_ = MapLifecycleState::SAVED;
  } else {
    state_ = MapLifecycleState::FAILED;
  }
}

double MapLifecycleCore::calculate_unknown_ratio(const grid_map_core::GridMap & map)
{
  if (map.data.empty()) {
    return 1.0;
  }

  const auto unknown_count = std::count(map.data.begin(), map.data.end(), -1);
  return static_cast<double>(unknown_count) / static_cast<double>(map.data.size());
}

const grid_map_core::GridMap & MapLifecycleCore::latest_map() const
{
  return latest_map_;
}

const MapStatistics & MapLifecycleCore::map_statistics() const
{
  return map_stats_;
}

MapLifecycleState MapLifecycleCore::state() const
{
  return state_;
}

const std::string & MapLifecycleCore::saved_map_url() const
{
  return saved_map_url_;
}

bool MapLifecycleCore::completion_detected() const
{
  return completion_detected_;
}

}  // 命名空间 map_lifecycle
