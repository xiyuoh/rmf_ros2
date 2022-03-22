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

// Public rmf_task API headers
#include <rmf_task/requests/ChargeBatteryFactory.hpp>
#include <rmf_task/requests/ParkRobotFactory.hpp>

#include <rmf_battery/agv/BatterySystem.hpp>
#include <rmf_battery/agv/SimpleMotionPowerSink.hpp>
#include <rmf_battery/agv/SimpleDevicePowerSink.hpp>

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
void EasyFullControl::Implementation::initialize_fleet(
  const rmf_fleet_adapter::agv::AdapterPtr& adapter,
  const std::string fleet_name,
  const std::string config_file,
  const std::string graph_file,
  std::optional<std::string> server_uri)
{
  // Fleet config
  const YAML::Node fleet_config = YAML::LoadFile(config_file);

  // Profile and traits
  const YAML::Node profile = fleet_config["profile"];
  const double footprint_rad = profile["footprint"].as<std::string>();
  const double vicinity_rad = profile["vicinity"].as<std::string>();
  const YAML::Node limits = fleet_config["limits"];
  const YAML::Node linear = limits["linear"];
  const double v_nom = linear[0];
  const double a_nom = linear[1];
  const YAML::Node angular = limits["angular"];
  const double w_nom = angular[0];
  const double b_nom = angular[1];
  const bool reversible = fleet_config["reversible"].as<bool>();
  auto traits = rmf_traffic::agv::VehicleTraits{
    {v_nom, a_nom},
    {w_nom, b_nom},
    rmf_traffic::Profile{
      rmf_traffic::geometry::make_final_convex<rmf_traffic::geometry::Circle>(r_f),
      rmf_traffic::geometry::make_final_convex<rmf_traffic::geometry::Circle>(r_v)
    }
  };
  traits.get_differential()->set_reversible(reversible);

  // Nav graph
  const auto graph = rmf_fleet_adapter::agv::parse_graph(graph_file, traits);

  // Add fleet
  auto easy_fleet = adapter->add_fleet(fleet_name, traits, graph, server_uri);

  // Settle fleet handle setups
  easy_fleet->fleet_state_topic_publish_period(std::nullopt); // disable fleet state publishing, er KIV also hor

  // Battery system
  const YAML::Node battery_config = fleet_config["battery_system"];
  const double voltage = battery_config["voltage"].as<std::double>();
  const double capacity = battery_config["capacity"].as<std::double>();
  const double charging_current = battery_config["charging_current"].as<std::double>();
  auto battery_system_optional = rmf_battery::agv::BatterySystem::make(
    voltage, capacity, charging_current);
  if (!battery_system_optional)
  {
    RCLCPP_ERROR(
      node->get_logger(),
      "Invalid values supplied for battery system");

    return nullptr;
  }
  auto battery_system = std::make_shared<rmf_battery::agv::BatterySystem>(
    *battery_system_optional);

  // Mechanical system
  const YAML::Node mechanical_config = fleet_config["battery_system"];
  const double mass = mechanical_config["mass"].as<std::double>();
  const double moment_of_inertia = mechanical_config["moment_of_inertia"].as<std::double>();
  const double friction_coefficient = mechanical_config["friction_coefficient"].as<std::double>();
  auto mechanical_system_optional = rmf_battery::agv::MechanicalSystem::make(
    mass, moment_of_inertia, friction_coefficient);
  if (!mechanical_system_optional)
  {
    RCLCPP_ERROR(
      node->get_logger(),
      "Invalid values supplied for mechanical system");

    return nullptr;
  }
  rmf_battery::agv::MechanicalSystem& mechanical_system =
    *mechanical_system_optional;

  std::shared_ptr<rmf_battery::agv::SimpleMotionPowerSink> motion_sink =
    std::make_shared<rmf_battery::agv::SimpleMotionPowerSink>(
    *battery_system, mechanical_system);

  // Ambient power system
  const YAML::Node ambient_config = fleet_config["ambient_system"];
  const double ambient_power_drain = ambient_config["power"].as<std::double>();
  auto ambient_power_system = rmf_battery::agv::PowerSystem::make(
    ambient_power_drain);
  if (!ambient_power_system)
  {
    RCLCPP_ERROR(
      node->get_logger(),
      "Invalid values supplied for ambient power system");

    return nullptr;
  }
  std::shared_ptr<rmf_battery::agv::SimpleDevicePowerSink> ambient_sink =
    std::make_shared<rmf_battery::agv::SimpleDevicePowerSink>(
    *battery_system, *ambient_power_system);

  // Tool power system
  const YAML::Node tool_config = fleet_config["tool_system"];
  const double tool_power_drain = tool_config["power"].as<std::double>();
  auto tool_power_system = rmf_battery::agv::PowerSystem::make(
    tool_power_drain);
  if (!tool_power_system)
  {
    RCLCPP_ERROR(
      node->get_logger(),
      "Invalid values supplied for tool power system");

    return nullptr;
  }
  std::shared_ptr<rmf_battery::agv::SimpleDevicePowerSink> tool_sink =
    std::make_shared<rmf_battery::agv::SimpleDevicePowerSink>(
    *battery_system, *tool_power_system);

  // Drain battery
  const bool drain_battery = fleet_config["account_for_battery_drain"].as<bool>();

  // Recharge threshold
  const double recharge_threshold = fleet_config["recharge_threshold"].as<std::double>();

  // Recharge state of charge
  const double recharge_soc = fleet_config["recharge_soc"].as<std::double>();

  // Finishing request
  const YAML::Node task_capabilities = fleet_config["task_capabilities"];
  const double finishing_request_string = task_capabilities["finishing_request"].as<std::string>();
  rmf_task::ConstRequestFactoryPtr finishing_request = nullptr;
  if (finishing_request_string == "charge")
  {
    finishing_request =
      std::make_shared<rmf_task::requests::ChargeBatteryFactory>();
    RCLCPP_INFO(
      node->get_logger(),
      "Fleet is configured to perform ChargeBattery as finishing request");
  }
  else if (finishing_request_string == "park")
  {
    finishing_request =
      std::make_shared<rmf_task::requests::ParkRobotFactory>();
    RCLCPP_INFO(
      node->get_logger(),
      "Fleet is configured to perform ParkRobot as finishing request");
  }
  else if (finishing_request_string == "nothing")
  {
    RCLCPP_INFO(
      node->get_logger(),
      "Fleet is not configured to perform any finishing request");
  }
  else
  {
    RCLCPP_WARN(
      node->get_logger(),
      "Provided finishing request [%s] is unsupported. The valid "
      "finishing requests are [charge, park, nothing]. The task planner will "
      " default to [nothing].",
      finishing_request_string.c_str());
  }

  // Set task planner params
  if (!easy_fleet->set_task_planner_params(
      battery_system,
      motion_sink,
      ambient_sink,
      tool_sink,
      recharge_threshold,
      recharge_soc,
      drain_battery,
      finishing_request))
  {
    RCLCPP_ERROR(
      node->get_logger(),
      "Failed to initialize task planner parameters");

    return nullptr;
  }

  std::unordered_set<uint8_t> task_types;
  if (task_capabilities["loop"].as<std::bool>())  // TODO change to if in config
  {
    task_types.insert(rmf_task_msgs::msg::TaskType::TYPE_LOOP);
  }

  // If the perform_deliveries parameter is true, then we just blindly accept
  // all delivery requests.
  if (task_capabilities["delivery"].as<std::bool>())  // TODO change to if in config
  {
    task_types.insert(rmf_task_msgs::msg::TaskType::TYPE_DELIVERY);
    connections->fleet->accept_delivery_requests(
      [](const rmf_task_msgs::msg::Delivery&) { return true; });
  }

  if (task_capabilities["clean"].as<std::bool>())  // TODO change to if in config
  {
    task_types.insert(rmf_task_msgs::msg::TaskType::TYPE_CLEAN);
  }

  easy_fleet->accept_task_requests(
    [task_types](const rmf_task_msgs::msg::TaskProfile& msg)
    {
      if (task_types.find(msg.description.task_type.type) != task_types.end())
        return true;

      return false;
    });

  const auto consider =
    [](const nlohmann::json& /*description*/,
      rmf_fleet_adapter::agv::FleetUpdateHandle::Confirmation& confirm)
    {
      // We accept all actions since full_control may be used for different
      // types of robots.
      confirm.accept();
    };

  // Configure this fleet to perform any kind of teleop action
  easy_fleet->add_performable_action(
    "teleop",
    consider);

  if (node->declare_parameter<bool>("disable_delay_threshold", false))
  {
    easy_fleet->default_maximum_delay(rmf_utils::nullopt);
  }
  else
  {
    easy_fleet->default_maximum_delay(
      rmf_fleet_adapter::get_parameter_or_default_time(
        *node, "delay_threshold", 10.0));
  }

  // TODO add some publishers/subscribers here???? im not sure. see how again later. 

  return easy_fleet;
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
