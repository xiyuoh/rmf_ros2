#ifndef SRC__RMF_FLEET_ADAPTER__VALET_HPP
#define SRC__RMF_FLEET_ADAPTER__VALET_HPP

#include "Node.hpp"

namespace rmf_fleet_adapter {
namespace agv {

class ValetManager
{
public:
  ValetManager(
    std::shared_ptr<Node> node,
    rmf_traffic::agv::Graph& graph,
    std::string const& name);

  void request_destination(
    rmf_traffic::agv::Graph::Waypoint const& waypoint,
    rmf_traffic::Time const& time_to_reach,
    rmf_traffic::Duration const& time_to_wait = std::chrono::seconds(90));
private:
  std::shared_ptr<Node> _node;
  rmf_traffic::agv::Graph _graph;

  std::vector<std::string> _parking_spots;

  enum CurrentState
  {
    PARKED,
    MOVING
  };
};

}
}

#endif
