#ifndef SRC__RMF_FLEET_ADAPTER__VALET_HPP
#define SRC__RMF_FLEET_ADAPTER__VALET_HPP

#include <rmf_traffic_ros2/reservations/Reservation.hpp>
#include <rmf_traffic_ros2/reservations/ReservationRequest.hpp>
#include "Node.hpp"

namespace rmf_fleet_adapter {
namespace agv {

class ValetManager: public std::enable_shared_from_this<ValetManager>
{
public:
  ValetManager(
    uint64_t id,
    std::shared_ptr<Node> node,
    rmf_traffic::agv::Graph& graph,
    std::string const& name);

  void request_destination(
    rmf_traffic::agv::Graph::Waypoint const& waypoint,
    rmf_traffic::Time const& time_to_reach,
    rmf_traffic::Duration const& time_to_wait = std::chrono::seconds(90));

  void on_reservation_activated(
    std::function<void(rmf_traffic::reservations::Reservation const&)> callback);

private:
  std::shared_ptr<Node> _node;
  rmf_traffic::agv::Graph _graph;
  ReservationProposalObs _reservation_proposal_observer;
  ReservationRolloutObs _reservation_rollout_observer;
  uint64_t _participant_id;

  std::vector<std::string> _parking_spots;

  std::unordered_map<uint64_t, uint64_t> _request_to_proposal_id;
  std::unordered_map<uint64_t,
    std::optional<rmf_traffic::reservations::Reservation>>
      _proposal_id_to_reservations;
  std::map<rmf_traffic::Time, uint64_t> _schedule;
  
  struct PendingProposals
  {
    uint64_t request_id;
    uint64_t proposalid;
    std::optional<rmf_traffic::reservations::Reservation> reservation;
  };
  std::unordered_map<uint64_t, uint64_t> _pending_requests_to_proposal_id;
  std::unordered_map<uint64_t, PendingProposals> _pending_proposals;

  enum CurrentState
  {
    PARKED,
    MOVING
  };

  CurrentState _current_state;

  std::function<void(rmf_traffic::reservations::Reservation const&)>
    _execute_reservation_callback;
};

}
}

#endif
