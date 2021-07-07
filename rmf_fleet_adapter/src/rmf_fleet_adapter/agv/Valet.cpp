#include "Valet.hpp"

namespace rmf_fleet_adapter {
namespace agv {

ValetManager::ValetManager(
  uint64_t id,
  std::shared_ptr<Node> node,
  rmf_traffic::agv::Graph& graph,
  std::string const& name):
    _node(node),
    _graph(graph),
    _name(name),
    _participant_id(id)
{
  Node::ReservationRegister registrationMsg;
  registrationMsg.participant_id = id;
  _node->reservation_register().publish(registrationMsg);
  _reservation_proposal_observer = _node->reservation_proposal()
    .filter([weak = weak_from_this()]
      (rmf_traffic::agv::ReservationProposal const& proposal) {
      return proposal.participant_id == weak->_participant_id;
    })
    .map([weak = weak_from_this()]
      (rmf_traffic::agv::ReservationProposal const& proposal) {
      // TODO perform some check if the proposal is valid
      // For now just assume the reservation is always valid.
      Node::ReservationProposalAcl ackmsg;
      ackmsg.participant_id = weak->_participant_id;
      ackmsg.proposalid = proposal.proposalid;
      auto me = weak.lock();
      if(!me)
        return;
      me->_node->acknowledge_reservation().publish(ackmsg);
      me->_pending_proposals.insert_or_assign(
        {
          proposal.proposalid,
          PendingProposal
          {
            proposal.request_id,
            proposal.proposal_id,
            convert(proposal.reservation)
          }
        });
      
    });
  node->reservation_rollout.
    .filter([weak = weak_from_this()]
      (rmf_traffic::agv::ReservationRollout const& rollout) {
      return rollout.participant_id == weak->_participant_id;
    })
    .map([weak = weak_from_this()]
      (rmf_traffic::agv::ReservationRollout const& rollout) {
      auto me = weak.lock();
      if(!me)
        return;
      me->_request_to_proposal_id[proposal.requestid] = proposal.proposalid;
      if(me->_proposal_id_to_reservation_id.count(proposal.proposalid) != 0
      && me->_proposal_id_to_reservation_id[proposal.proposalid].has_value())
      {
        auto start = me->_proposal_id_to_reservation_id[proposal.proposalid]
          ->start_time();
        auto proposal = me->_schedule.erase(start)
      }
      me->_proposal_id_to_reservation_id[proposal.proposalid] =
        proposal.reservation;
    });
}

ValetManager::request_destination(
  rmf_traffic::agv::Graph::Waypoint const& waypoint,
  rmf_traffic::Time const& time_to_reach,
  rmf_traffic::Duration const& time_to_wait
)
{
  Node::ReservationRequests msg;
  //_node->
}

ValetManager::on_reservation_activated(
  std::function<void(rmf_traffic::reservations::Reservation const&)> callback)
{
  _execute_reservation_callback = callback;
}

}
}
