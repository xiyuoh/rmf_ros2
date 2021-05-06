/*
 * Copyright (C) 2021 Open Source Robotics Foundation
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

#ifndef RMF_FLEET_ADAPTER__AGV__EVENT_HPP
#define RMF_FLEET_ADAPTER__AGV__EVENT_HPP

#include <rmf_traffic/agv/Graph.hpp>

#include <vector>

namespace rmf_fleet_adapter {
namespace agv {

//==============================================================================
class Event;
using ConstEventPtr = std::shared_ptr<const Event>;

//==============================================================================
/// Make a sequence of events. Each event will be run one at a time, starting
/// from the first element in the vector to the last. Each subsequent event will
/// not be started until the previous one is complete.
ConstEventPtr make_event_sequence(std::vector<ConstEventPtr> events);

//==============================================================================
/// Convert a rmf_traffic::agv::Graph::Lane::Event into a
/// rmf_fleet_adapter::agv::ConstEventPtr.
///
/// This is useful when converting events from a rmf_traffic::agv::Plan
ConstEventPtr make_event(const rmf_traffic::agv::Graph::Lane::Event& event);

//==============================================================================
/// Make an event where a door needs to open.
///
/// \param[in] door_name
///   The name of the door to open
///
/// \param[in] duration_estimate
///   An estimate of how long it will take the door to open from the time the
///   request is sent
ConstEventPtr make_door_open(
  std::string door_name,
  rmf_traffic::Duration duration_estimate);

//==============================================================================
/// Make an event where a door needs to close.
///
/// \param[in] door_name
///   The name of the door to close
///
/// \param[in] duration_estimate
///   An estimate of how long it will take the door to close from the time the
///   request is sent
ConstEventPtr make_door_close(
  std::string door_name,
  rmf_traffic::Duration duration_estimate);

//==============================================================================
/// Make an event where the robot needs to enter a lift.
///
/// \param[in] lift_name
///   The name of the lift that needs to be used
///
/// \param[in] floor_name
///   The name of the floor that the lift needs to open on
///
/// \param[in] duration_estimate
///   An estimate of how long it will take the lift to arrive and its doors to
///   open
ConstEventPtr make_lift_session_begin(
  std::string lift_name,
  std::string floor_name,
  rmf_traffic::Duration duration_estimate);

//==============================================================================
/// Make an event where the robot needs to move to a new floor
///
/// \param[in] lift_name
///   The name of the lift that needs to be used
///
/// \param[in] floor_name
///   The name of the floor that the lift needs to move to
///
/// \param[in] duration_estimate
///   An estimate of how long it will take the lift to arrive at its new floor
ConstEventPtr make_lift_arrive(
  std::string lift_name,
  std::string floor_name,
  rmf_traffic::Duration duration_estimate);

//==============================================================================
/// Make an event where the robot ends a session with a lift.
///
/// \param[in] lift_name
///   The name the lift that needs to be used
///
/// \param[in] floor_name
///   The name of the floor where the session is ending
///
/// \param[in] duration_estimate
///   An estimate of how long it will take the session to end
ConstEventPtr make_lift_session_end(
  std::string lift_name,
  std::string floor_name,
  rmf_traffic::Duration duration_estimate);

} // namespace agv
} // namespace rmf_fleet_adapter

#endif // RMF_FLEET_ADAPTER__AGV__EVENT_HPP
