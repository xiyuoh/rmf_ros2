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
#include <rmf_traffic_ros2/reservations/Reservation.hpp>
#include "Participant.hpp"

namespace rmf_traffic_ros2 {

bool Participant::request_proposal(
  rmf_traffic::reservations::RequestId id,
  rmf_traffic::reservations::Reservation& res,
  uint64_t proposal_version)
{
  ProposalMsg proposal_msg;
  proposal_msg.participant = _participant_id;
  proposal_msg.requestid = id;
  proposal_msg.proposalid = proposal_version;
  proposal_msg.assign_new_request = true;
  proposal_msg.reservation = convert(res);

  _reservation_proposal_pub->publish(proposal_msg);

  auto result = proposal_acceptance.wait_for(_wait_time, proposal_version);

  if(!result.has_value()) return false;
  return result.value() == ClientProposalStatus::ACCEPTED;
}

bool Participant::request_confirmed(
  rmf_traffic::reservations::RequestId id,
  rmf_traffic::reservations::Reservation& res,
  uint64_t proposal_version)
{
  RolloutMsg rollout_msg;
  rollout_msg.proposalid = proposal_version;
  rollout_msg.participant_id = _participant_id;

  _rollout_pub->publish(rollout_msg);
  return true;
}

bool Participant::unassign_request_confirmed(
  rmf_traffic::reservations::RequestId id,
  uint64_t proposal_version)
{
  /// TODO Figure out better way of rolling out with a single message
  RolloutMsg rollout_msg;
  rollout_msg.proposalid = proposal_version;
  rollout_msg.participant_id = _participant_id;

  _rollout_pub->publish(rollout_msg);
  return true;
}

bool Participant::unassign_request_proposal(
  rmf_traffic::reservations::RequestId id,
  uint64_t proposal_version)
{
  ProposalMsg proposal_msg;
  proposal_msg.participant = _participant_id;
  proposal_msg.requestid = id;
  proposal_msg.proposalid = proposal_version;
  proposal_msg.assign_new_request = false;

  _reservation_proposal_pub->publish(proposal_msg);

  auto result = proposal_acceptance.wait_for(_wait_time, proposal_version);

  if(!result.has_value()) return false;
  return result.value() == ClientProposalStatus::ACCEPTED;
}

Participant::Participant(
  ParticipantId participant_id,
  ProposalPub proposal_pub,
  RolloutPub rollout_pub):
  _participant_id(participant_id),
  _reservation_proposal_pub(proposal_pub),
  _rollout_pub(rollout_pub)
{

}
}
