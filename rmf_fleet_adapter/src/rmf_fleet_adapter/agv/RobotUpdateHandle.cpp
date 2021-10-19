/*
 * Copyright (C) 2020 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/

#include "internal_RobotUpdateHandle.hpp"

#include <rmf_traffic_ros2/Time.hpp>

#include <iostream>
#include <unordered_set>

namespace rmf_fleet_adapter {
namespace agv {

//==============================================================================
std::shared_ptr<RobotContext> RobotUpdateHandle::Implementation::get_context()
{
  auto output = context.lock();
  if (output)
    return output;

  if (reported_loss)
    return nullptr;

  std::cerr << "ERROR: [RobotUpdateHandle] Robot named [" << name << "] is no "
            << "longer available" << std::endl;
  reported_loss = true;
  return nullptr;
}

//==============================================================================
std::shared_ptr<const RobotContext>
RobotUpdateHandle::Implementation::get_context() const
{
  return const_cast<Implementation&>(*this).get_context();
}

//==============================================================================
void RobotUpdateHandle::interrupted()
{
  if (const auto context = _pimpl->get_context())
  {
    context->_interrupt_publisher.get_subscriber().on_next(
      RobotContext::Empty());
  }
}

std::ostream& operator<<(std::ostream& os, const std::vector<rmf_traffic::agv::Plan::Start>& location)
{
  for (const auto& l : location)
    os << " " << l.waypoint();
  os << " ";
  return os;
}

bool operator==(
  const std::vector<rmf_traffic::agv::Plan::Start>& a,
  const std::vector<rmf_traffic::agv::Plan::Start>& b)
{
  std::unordered_set<std::size_t> in_a;
  for (const auto& l : a)
    in_a.insert(l.waypoint());

  for (const auto& l : b)
  {
    if (in_a.erase(l.waypoint()) == 0)
      return false;
  }

  return in_a.empty();
}

bool operator!=(
  const std::vector<rmf_traffic::agv::Plan::Start>& a,
  const std::vector<rmf_traffic::agv::Plan::Start>& b)
{
  return !(a == b);
}

void check_update(
  const std::vector<rmf_traffic::agv::Plan::Start>& old,
  const std::vector<rmf_traffic::agv::Plan::Start>& next,
  const std::size_t line)
{
  if (next != old)
  {
    std::cout << " >>> Updating waypoint from [" << old << "] to ["
              << next << "] at line [" << line << "]";
  }
}

//==============================================================================
void RobotUpdateHandle::update_position(
  std::size_t waypoint,
  double orientation,
  std::size_t line)
{
  if (const auto context = _pimpl->get_context())
  {
    context->worker().schedule(
      [context, waypoint, orientation, line](const auto&)
      {
        const auto old = context->_location;
        context->_location = {
          rmf_traffic::agv::Plan::Start(
            rmf_traffic_ros2::convert(context->node()->now()),
            waypoint, orientation)
        };

        check_update(old, context->_location, line);
      });
  }
}

//==============================================================================
void RobotUpdateHandle::update_position(
  const Eigen::Vector3d& position,
  const std::vector<std::size_t>& lanes,
  std::size_t line)
{
  if (const auto context = _pimpl->get_context())
  {
    if (lanes.empty())
    {
      // *INDENT-OFF*
      throw std::runtime_error(
        "[RobotUpdateHandle::update_position] No lanes specified for "
        "function signature that requires at least one lane.");
      // *INDENT-ON*
    }

    const auto now = rmf_traffic_ros2::convert(context->node()->now());
    rmf_traffic::agv::Plan::StartSet starts;
    for (const auto l : lanes)
    {
      const auto& graph = context->navigation_graph();
      const auto wp = graph.get_lane(l).exit().waypoint_index();
      starts.push_back(
        {
          now, wp, position[2], Eigen::Vector2d(position.block<2, 1>(0, 0)), l
        });
    }

    context->worker().schedule(
      [context, starts = std::move(starts), line](const auto&)
      {
        const auto old = context->_location;
        context->_location = std::move(starts);

        check_update(old, context->_location, line);
      });
  }
}

//==============================================================================
void RobotUpdateHandle::update_position(
  const Eigen::Vector3d& position,
  const std::size_t waypoint,
  std::size_t line)
{
  if (const auto& context = _pimpl->get_context())
  {
    context->worker().schedule(
      [context, position, waypoint, line](const auto&)
      {
        const auto old = context->_location;
        context->_location = {
          rmf_traffic::agv::Plan::Start(
            rmf_traffic_ros2::convert(context->node()->now()),
            waypoint, position[2], Eigen::Vector2d(position.block<2, 1>(0, 0)))
        };

        check_update(old, context->_location, line);
      });
  }
}

//==============================================================================
void RobotUpdateHandle::update_position(
  const std::string& map_name,
  const Eigen::Vector3d& position,
  const double max_merge_waypoint_distance,
  const double max_merge_lane_distance,
  const double min_lane_length,
  std::size_t line)
{
  if (const auto context = _pimpl->get_context())
  {
    const auto now = rmf_traffic_ros2::convert(context->node()->now());
    auto starts = rmf_traffic::agv::compute_plan_starts(
      context->navigation_graph(), map_name, position, now,
      max_merge_waypoint_distance, max_merge_lane_distance,
      min_lane_length);

    if (starts.empty())
    {
      RCLCPP_ERROR(
        context->node()->get_logger(),
        "[RobotUpdateHandle::update_position] The robot [%s] has diverged "
        "from its navigation graph, currently located at <%f, %f, %f> on "
        "map [%s]", context->requester_id().c_str(),
        position[0], position[1], position[2], map_name.c_str());
      return;
    }

    context->worker().schedule(
      [context, starts = std::move(starts), line](const auto&)
      {
        const auto old = context->_location;
        context->_location = std::move(starts);
        check_update(old, context->_location, line);
      });
  }
}

//==============================================================================
RobotUpdateHandle& RobotUpdateHandle::set_charger_waypoint(
  const std::size_t charger_wp)
{
  if (const auto context = _pimpl->get_context())
  {
    auto end_state = context->current_task_end_state();
    end_state.charging_waypoint(charger_wp);
    context->current_task_end_state(end_state);
    RCLCPP_INFO(
      context->node()->get_logger(),
      "Charger waypoint for robot [%s] set to index [%ld]",
      context->name().c_str(),
      charger_wp);
  }

  return *this;
}

//==============================================================================
void RobotUpdateHandle::update_battery_soc(const double battery_soc)
{
  if (battery_soc < 0.0 || battery_soc > 1.0)
    return;

  if (const auto context = _pimpl->get_context())
  {
    context->worker().schedule(
      [context, battery_soc](const auto&)
      {
        context->current_battery_soc(battery_soc);
      });
  }
}

//==============================================================================
RobotUpdateHandle& RobotUpdateHandle::maximum_delay(
  rmf_utils::optional<rmf_traffic::Duration> value)
{
  if (const auto context = _pimpl->get_context())
  {
    context->worker().schedule(
      [context, value](const auto&)
      {
        context->maximum_delay(value);
      });
  }

  return *this;
}

//==============================================================================
rmf_utils::optional<rmf_traffic::Duration>
RobotUpdateHandle::maximum_delay() const
{
  if (const auto context = _pimpl->get_context())
    return context->maximum_delay();

  return rmf_utils::nullopt;
}

//==============================================================================
RobotUpdateHandle::RobotUpdateHandle()
{
  // Do nothing
}

//==============================================================================
RobotUpdateHandle::Unstable& RobotUpdateHandle::unstable()
{
  return _pimpl->unstable;
}

//==============================================================================
const RobotUpdateHandle::Unstable& RobotUpdateHandle::unstable() const
{
  return _pimpl->unstable;
}

//==============================================================================
rmf_traffic::schedule::Participant*
RobotUpdateHandle::Unstable::get_participant()
{
  if (const auto context = _pimpl->get_context())
  {
    auto& itinerary = context->itinerary();
    return &itinerary;
  }
  return nullptr;
}

//==============================================================================
void RobotUpdateHandle::Unstable::set_lift_entry_watchdog(
  Watchdog watchdog,
  rmf_traffic::Duration wait_duration)
{
  if (const auto context = _pimpl->get_context())
  {
    context->worker().schedule(
      [context, watchdog = std::move(watchdog), wait_duration](const auto&)
      {
        context->set_lift_entry_watchdog(watchdog, wait_duration);
      });
  }
}

} // namespace agv
} // namespace rmf_fleet_adapter
