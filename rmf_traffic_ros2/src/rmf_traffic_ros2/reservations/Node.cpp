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
#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>

#include <rmf_traffic_msgs/msg/reservation.hpp>
#include <rmf_traffic_msgs/msg/reservation_requests.hpp>
#include <rmf_traffic_msgs/msg/reservation_rollout.hpp>
#include <rmf_traffic_msgs/msg/reservation_register_participant.hpp>
#include <rmf_traffic_msgs/msg/reservation_participant_heart_beat.hpp>
#include <rmf_traffic_msgs/msg/reservation_proposal.hpp>
#include <rmf_traffic_msgs/msg/reservation_proposal_ack.hpp>
#include <rmf_traffic_msgs/msg/reservation_proposal_rej.hpp>
#include <rmf_traffic_msgs/msg/reservation_rollout.hpp>
#include <rmf_traffic_msgs/msg/reservation_cancel_rollout.hpp>

#include <rmf_traffic/reservations/Database.hpp>

#include <rmf_traffic_ros2/StandardNames.hpp>
#include <rmf_traffic_ros2/reservations/ReservationRequests.hpp>

#include "ParticipantRegistry.hpp"
namespace rmf_traffic_ros2 {

class ReservationManager : public rclcpp::Node
{
  public:
    ReservationManager()
      : Node("reservation_manager")
    {
      using std::placeholders::_1;

      rclcpp::QoS qos(10);

      // Publishers
      _reservation_proposal_pub =
        create_publisher<rmf_traffic_msgs::msg::ReservationProposal>(
          ReservationProposalTopicName, qos.reliable());
      _reservation_rollout_pub =
        create_publisher<rmf_traffic_msgs::msg::ReservationRollout>(
          ReservationRolloutTopicName, qos.reliable());

      _reservation_requests_sub =
        create_subscription<rmf_traffic_msgs::msg::ReservationRequests>(
          ReservationRequestTopicName,
          qos,
          std::bind(&ReservationManager::on_request_reservation, this, _1)
        );

      _reservation_proposal_ack_sub =
        create_subscription<rmf_traffic_msgs::msg::ReservationProposalAck>(
          ReservationProposalAckTopicName,
          qos,
          std::bind(&ReservationManager::on_proposal_acceptance, this, _1)
        );

      _reservation_proposal_rej_sub =
        create_subscription<rmf_traffic_msgs::msg::ReservationProposalRej>(
          ReservationProposalAckTopicName,
          qos,
          std::bind(&ReservationManager::on_proposal_rejection, this, _1)
        );

      _reservation_participant_registration_sub =
        create_subscription<rmf_traffic_msgs::msg::ReservationRegisterParticipant>(
          ReservationProposalAckTopicName,
          qos,
          std::bind(&ReservationManager::on_participant_register, this, _1)
        );

      _reservation_participant_heartbeat_sub =
        create_subscription<rmf_traffic_msgs::msg::ReservationParticipantHeartBeat>(
          ReservationUpdateHeartbeatTopicName,
          qos,
          std::bind(&ReservationManager::on_participant_heartbeat, this, _1)
        );
    }


  private:
    void on_request_reservation(
      const rmf_traffic_msgs::msg::ReservationRequests::SharedPtr req)
    {
      std::vector<rmf_traffic::reservations::ReservationRequest> res_queue;
      for(auto alt: req->alternatives)
        res_queue.push_back(convert(alt));

      db.request_reservation(
        req->participant,
        req->request_id,
        res_queue,
        req->priority);
    }

    void on_proposal_acceptance(
      const rmf_traffic_msgs::msg::ReservationProposalAck::SharedPtr participant)
    {
      auto participant_handler = _registry.get(participant->participant_id);
    }

    void on_proposal_rejection(
      const rmf_traffic_msgs::msg::ReservationProposalRej::SharedPtr participant)
    {

    }

    void on_participant_register(
      const rmf_traffic_msgs::msg::ReservationRegisterParticipant::SharedPtr participant)
    {
      std::shared_ptr<Participant> participant_handler
        = std::make_shared<Participant>(
          participant->participant_id,
          _reservation_proposal_pub,
          _reservation_rollout_pub
        );
      bool success = _registry.add(participant->participant_id, participant_handler);
      if(!success)
        return;
      db.register_participant(participant->participant_id, participant_handler);
    }

    void on_participant_heartbeat(
      const rmf_traffic_msgs::msg::ReservationParticipantHeartBeat::SharedPtr participant)
    {
      //registry.received_heartbeat(participant->participant_id);
    }

    rclcpp::TimerBase::SharedPtr timer_;

    // Publishers
    rclcpp::Publisher<rmf_traffic_msgs::msg::ReservationProposal>::SharedPtr
      _reservation_proposal_pub;
    rclcpp::Publisher<rmf_traffic_msgs::msg::ReservationRollout>::SharedPtr
      _reservation_rollout_pub;

    //Subscribers
    rclcpp::Subscription<rmf_traffic_msgs::msg::ReservationRequests>::SharedPtr
      _reservation_requests_sub;
    rclcpp::Subscription<rmf_traffic_msgs::msg::ReservationProposalAck>::SharedPtr
      _reservation_proposal_ack_sub;
    rclcpp::Subscription<rmf_traffic_msgs::msg::ReservationProposalRej>::SharedPtr
      _reservation_proposal_rej_sub;
    rclcpp::Subscription<rmf_traffic_msgs::msg::ReservationRegisterParticipant>::SharedPtr
      _reservation_participant_registration_sub;
    rclcpp::Subscription<rmf_traffic_msgs::msg::ReservationParticipantHeartBeat>::SharedPtr
      _reservation_participant_heartbeat_sub;

    rmf_traffic::reservations::Database db;

    ParticipantRegistry _registry;

};

}