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

#ifndef SRC__RMF_FLEET_ADAPTER__AGV__INTERNAL_EASYFULLCONTROL_HPP
#define SRC__RMF_FLEET_ADAPTER__AGV__INTERNAL_EASYFULLCONTROL_HPP

#include <rmf_fleet_adapter/agv/EasyFullControl.hpp>

namespace rmf_fleet_adapter {
namespace agv {

//==============================================================================
class EasyFullControl::Implementation
{
public:
  // std::shared_ptr<FleetUpdateHandle> fleet_handle;
  // const std::string name;
  // const std::shared_ptr<Node> node;
//   const rmf_traffic::agv::Graph graph;
//   const rmf_traffic::agv::VehicleTraits traits;
//   std::optional<std::string> server_uri;

  std::shared_ptr<EasyFullControl> make(
    const std::string& fleet_name,
    std::shared_ptr<Node> node,
    std::optional<std::string> server_uri);

  void initialize_fleet(
    const rmf_fleet_adapter::agv::AdapterPtr& adapter,
    const std::string fleet_name,
    const std::string config_file,
    const std::string graph_file,
    std::optional<std::string> server_uri);

  void add_robots(
    const std::string fleet_name,
    // const rclcpp::Node node=node, // set default
    const rmf_traffic::agv::Graph graph,
    const rmf_traffic::agv::VehicleTraits traits); // arguments KIV

};

} // namespace agv
} // namespace rmf_fleet_adapter

#endif // SRC__RMF_FLEET_ADAPTER__AGV__INTERNAL_EASYFULLCONTROL_HPP
