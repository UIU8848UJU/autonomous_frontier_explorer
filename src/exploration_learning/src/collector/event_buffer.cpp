#include "exploration_learning/collector/event_buffer.hpp"

#include <algorithm>

namespace exploration_learning::collector
{

EventBuffer::EventBuffer(double buffer_duration_sec)
: buffer_duration_(rclcpp::Duration::from_seconds(std::max(0.0, buffer_duration_sec)))
{
}

void EventBuffer::set_buffer_duration(double buffer_duration_sec)
{
  std::lock_guard<std::mutex> lock(mutex_);
  buffer_duration_ = rclcpp::Duration::from_seconds(std::max(0.0, buffer_duration_sec));
  events_.clear();
}

void EventBuffer::add_event(const TopicEvent & event)
{
  std::lock_guard<std::mutex> lock(mutex_);
  events_.push_back(event);
  prune_locked(event.timestamp);
}

std::optional<TopicEvent> EventBuffer::latest_event(const std::string & topic_name) const
{
  std::lock_guard<std::mutex> lock(mutex_);
  for (auto it = events_.rbegin(); it != events_.rend(); ++it) {
    if (it->topic_name == topic_name) {
      return *it;
    }
  }
  return std::nullopt;
}

std::vector<TopicEvent> EventBuffer::events_for_topic(const std::string & topic_name) const
{
  std::lock_guard<std::mutex> lock(mutex_);
  std::vector<TopicEvent> result;
  for (const auto & event : events_) {
    if (event.topic_name == topic_name) {
      result.push_back(event);
    }
  }
  return result;
}

std::size_t EventBuffer::size() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return events_.size();
}

void EventBuffer::clear()
{
  std::lock_guard<std::mutex> lock(mutex_);
  events_.clear();
}

void EventBuffer::prune_locked(const rclcpp::Time & now)
{
  while (!events_.empty() && (now - events_.front().timestamp) > buffer_duration_) {
    events_.pop_front();
  }
}

}  // namespace exploration_learning::collector
