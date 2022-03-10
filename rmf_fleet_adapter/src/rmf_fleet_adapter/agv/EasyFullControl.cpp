/*
 * Copyright (C) 2022 Open Source Robotics Foundation
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

#include "internal_EasyFullControl.hpp"

#include <rmf_traffic/agv/VehicleTraits.hpp>
#include <yaml-cpp/yaml.h>

namespace rmf_fleet_adapter {
namespace agv {

//==============================================================================
std::shared_ptr<EasyFullControl> EasyFullControl::Implementation::make(
  const std::string& fleet_name,
  std::shared_ptr<Node> node,
  std::optional<std::string> server_uri)
{
  auto handle = std::shared_ptr<EasyFullControl>(new EasyFullControl);
  handle->_pimpl = rmf_utils::make_unique_impl<Implementation>(
      fleet_name,
      std::move(node),
      std::move(server_uri));

  return handle;
}

//==============================================================================
// void EasyFullControl::Implementation::initialize_fleet(
//   const std::string fleet_name,
//   const std::string config_file,
//   const std::string graph_file)
// {
//   // Fleet config
//   const YAML::Node fleet_config = YAML::LoadFile(config_file);

//   // Profile and traits
//   const YAML::Node profile = fleet_config["profile"];
//   const double footprint_rad = profile["footprint"].as<std::string>();
//   const double vicinity_rad = profile["vicinity"].as<std::string>();
//   const YAML::Node limits = fleet_config["limits"];
//   const YAML::Node linear = limits["linear"];
//   const double v_nom = linear[0];
//   const double a_nom = linear[1];
//   const YAML::Node angular = limits["angular"];
//   const double w_nom = angular[0];
//   const double b_nom = angular[1];

//   const bool reversible = fleet_config["reversible"].as<bool>();

//   auto traits = rmf_traffic::agv::VehicleTraits{
//     {v_nom, a_nom},
//     {w_nom, b_nom},
//     rmf_traffic::Profile{
//       rmf_traffic::geometry::make_final_convex<
//         rmf_traffic::geometry::Circle>(r_f),
//       rmf_traffic::geometry::make_final_convex<
//         rmf_traffic::geometry::Circle>(r_v)
//     }
//   };
//   traits.get_differential()->set_reversible(reversible);
//   return traits;

//   // Nav graph
//   const auto graph = rmf_fleet_adapter::agv::parse_graph(graph_file, traits);

  // add fleet
}

//==============================================================================
// void EasyFullControl::Implementation::add_robot(
//   // argument)
// {
//   //
// }

//==============================================================================
EasyFullControl::EasyFullControl()
{
  // Do nothing
}

} // namespace agv
} // namespace rmf_fleet_adapter
