#include "Valet.hpp"

namespace rmf_fleet_adapter {
namespace agv {

ValetManager::ValetManager(
  std::shared_ptr<Node> node,
  rmf_traffic::agv::Graph& graph,
  std::string const& name):
    _node(node),
    _graph(graph),
    _name(name)
{
}

ValetManager::request_destination(
  rmf_traffic::agv::Graph::Waypoint const& waypoint,
  rmf_traffic::Time const& time_to_reach,
  rmf_traffic::Duration const& time_to_wait
)
{
  //_node->
}
}
}
