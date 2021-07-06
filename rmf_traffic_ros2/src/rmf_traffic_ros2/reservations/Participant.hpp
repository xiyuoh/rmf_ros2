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
#ifndef __RMF_TRAFFIC_ROS2__INTERNAL__SRC__PARTICIPANT_HPP__
#define __RMF_TRAFFIC_ROS2__INTERNAL__SRC__PARTICIPANT_HPP__

#include <rmf_traffic/reservations/Participant.hpp>
#include <rmf_traffic_msgs/msg/reservation_proposal.hpp>
#include <rmf_traffic_msgs/msg/reservation_rollout.hpp>

#include <rclcpp/rclcpp.hpp>

#include "EventBus.hpp"

namespace rmf_traffic_ros2 {
class Participant : public rmf_traffic::reservations::Participant
{
public:
  using ProposalPub =
    rclcpp::Publisher<rmf_traffic_msgs::msg::ReservationProposal>::SharedPtr;
  using ProposalMsg =
    rmf_traffic_msgs::msg::ReservationProposal;
  using ParticipantId = rmf_traffic::reservations::ParticipantId;

  using RolloutPub =
    rclcpp::Publisher<rmf_traffic_msgs::msg::ReservationRollout>::SharedPtr;
  using RolloutMsg =
    rmf_traffic_msgs::msg::ReservationRollout;

  enum ClientProposalStatus
  {
    ACCEPTED, REJECTED
  };

  bool request_proposal(
    rmf_traffic::reservations::RequestId id,
    rmf_traffic::reservations::Reservation& res,
    uint64_t proposal_version) override;

  bool request_confirmed(
    rmf_traffic::reservations::RequestId id,
    rmf_traffic::reservations::Reservation& res,
    uint64_t proposal_version) override;

  bool unassign_request_confirmed(
    rmf_traffic::reservations::RequestId id,
    uint64_t proposal_version) override;

  bool unassign_request_proposal(
    rmf_traffic::reservations::RequestId id,
    uint64_t proposal_version) override;

  void notify_proposal_result(
    int64_t proposal_version, ClientProposalStatus status);

  EventNotificationBus<uint64_t, ClientProposalStatus>
    proposal_acceptance;

  ProposalPub _reservation_proposal_pub;
  RolloutPub _rollout_pub;

  ParticipantId _participant_id;
  rmf_traffic::Duration _wait_time;

  Participant(
    ParticipantId participant,
    ProposalPub proposal_pub,
    RolloutPub rollout_pub);

};
}

#endif
