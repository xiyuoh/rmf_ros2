/*
 * Copyright (C) 2019 Open Source Robotics Foundation
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

#include <rmf_traffic_ros2/reservations/ReservationRequests.hpp>
#include <rmf_traffic_ros2/Time.hpp>
#include <chrono>

namespace rmf_traffic_ros2 {

///=============================================================================
rmf_traffic::reservations::ReservationRequest::TimeRange convert(
  rmf_traffic_msgs::msg::ReservationTimeRange request)
{
  std::optional<rmf_traffic::Time> upper_bound =
    (request.has_upper_bound)?
      std::optional<rmf_traffic::Time>{convert(request.upper_bound)}
      : std::nullopt;

  std::optional<rmf_traffic::Time> lower_bound =
    (request.has_lower_bound)?
      std::optional<rmf_traffic::Time>{convert(request.lower_bound)}
      : std::nullopt;

  return
    rmf_traffic::reservations::ReservationRequest::TimeRange::make_time_range(
      upper_bound,
      lower_bound);
}

///=============================================================================
rmf_traffic_msgs::msg::ReservationTimeRange convert(
  rmf_traffic::reservations::ReservationRequest::TimeRange request)
{
  rmf_traffic_msgs::msg::ReservationTimeRange msg;
  msg.has_upper_bound = request.lower_bound().has_value();
  msg.has_lower_bound = request.upper_bound().has_value();

  if(request.lower_bound().has_value())
    msg.lower_bound = convert(request.lower_bound().value());

  if(request.upper_bound().has_value())
    msg.upper_bound = convert(request.upper_bound().value());

  return msg;
}

///=============================================================================
rmf_traffic::reservations::ReservationRequest convert(
  rmf_traffic_msgs::msg::ReservationRequest request)
{
  using namespace rmf_traffic::reservations;
  std::optional<ReservationRequest::TimeRange> start_times =
    (request.has_start_time) ?
      std::optional{convert(request.start_times)} : std::nullopt;

  std::optional<rmf_traffic::Duration> duration =
    (request.has_duration) ?
      std::optional{convert(request.duration)} : std::nullopt;

  std::optional<rmf_traffic::Time> finish_time =
    (request.has_finish_time) ?
      std::optional{convert(request.finish_time)} : std::nullopt;

  return rmf_traffic::reservations::ReservationRequest::make_request(
    request.resource_name,
    start_times,
    duration,
    finish_time
  );
}

///=============================================================================
rmf_traffic_msgs::msg::ReservationRequest convert(
  rmf_traffic::reservations::ReservationRequest request)
{
  rmf_traffic_msgs::msg::ReservationRequest msg;
  msg.resource_name = request.resource_name();

  msg.has_start_time = request.start_time().has_value();
  if(request.start_time().has_value())
    msg.start_times = convert(request.start_time().value());

  msg.has_finish_time = request.finish_time().has_value();
  if(request.finish_time().has_value())
    msg.finish_time = convert(request.finish_time().value());

  msg.has_duration = request.duration().has_value();
  if(request.duration().has_value())
    msg.duration = convert(request.duration().value());

  return msg;
}

}