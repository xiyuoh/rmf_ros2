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
#ifndef __RMF_TRAFFIC_ROS2__RESERVATIONS__RESERVATIONREQUEST_HPP__
#define __RMF_TRAFFIC_ROS2__RESERVATIONS__RESERVATIONREQUEST_HPP__

#include <rmf_traffic_msgs/msg/reservation_requests.hpp>
#include <rmf_traffic/reservations/ReservationRequest.hpp>

namespace rmf_traffic_ros2 {

///=============================================================================
rmf_traffic::reservations::ReservationRequest::TimeRange convert(
  rmf_traffic_msgs::msg::ReservationTimeRange request
);

///=============================================================================
rmf_traffic_msgs::msg::ReservationTimeRange convert(
  rmf_traffic::reservations::ReservationRequest::TimeRange request
);

///=============================================================================
rmf_traffic::reservations::ReservationRequest convert(
  rmf_traffic_msgs::msg::ReservationRequest request
);

///=============================================================================
rmf_traffic_msgs::msg::ReservationRequest convert(
  rmf_traffic::reservations::ReservationRequest request
);


}
#endif