/*
 * Copyright (C) 2020 Open Source Robotics Foundation
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

#include "MockAdapterFixture.hpp"

#include <agv/internal_FleetUpdateHandle.hpp>
#include <agv/internal_RobotUpdateHandle.hpp>
#include <rmf_traffic_ros2/Time.hpp>
#include <rmf_traffic/geometry/Circle.hpp>

namespace rmf_fleet_adapter {
namespace phases {
namespace test {

std::size_t MockAdapterFixture::Data::_node_counter = 0;

//==============================================================================
MockAdapterFixture::MockAdapterFixture()
: data(std::make_shared<Data>())
{
  data->_context = std::make_shared<rclcpp::Context>();
  data->_context->init(0, nullptr);

  data->adapter = std::make_shared<agv::test::MockAdapter>(
    "test_node_" + std::to_string(data->_node_counter++),
    rclcpp::NodeOptions().context(data->_context));

  data->ros_node = data->adapter->node();

  const std::string test_map_name = "test_map";
  data->graph.add_waypoint(test_map_name, {0.0, -10.0}).set_charger(true); // 0
  data->graph.add_waypoint(test_map_name, {0.0, -5.0});  // 1
  data->graph.add_waypoint(test_map_name, {5.0, -5.0}).set_holding_point(true);  // 2
  data->graph.add_waypoint(test_map_name, {-10.0, 0.0}); // 3
  data->graph.add_waypoint(test_map_name, {-5.0, 0.0}); // 4
  data->graph.add_waypoint(test_map_name, {0.0, 0.0}); // 5
  data->graph.add_waypoint(test_map_name, {5.0, 0.0}); // 6
  data->graph.add_waypoint(test_map_name, {10.0, 0.0}); // 7
  data->graph.add_waypoint(test_map_name, {0.0, 5.0}); // 8
  data->graph.add_waypoint(test_map_name, {5.0, 5.0}).set_holding_point(true); // 9
  data->graph.add_waypoint(test_map_name, {0.0, 10.0}); // 10

  /*
   *                   10
   *                   |
   *                  (D)
   *                   |
   *                   8------9
   *                   |      |
   *                   |      |
   *     3------4------5------6--(D)--7
   *                   |      |
   *                   |      |
   *                   1------2
   *                   |
   *                   |
   *                   0
   **/

  auto add_bidir_lane = [&](const std::size_t w0, const std::size_t w1)
    {
      data->graph.add_lane(w0, w1);
      data->graph.add_lane(w1, w0);
    };

  auto add_dock_lane = [&](
    const std::size_t w0,
    const std::size_t w1,
    std::string dock_name)
    {
      using Lane = rmf_traffic::agv::Graph::Lane;
      data->graph.add_lane({w0,
          Lane::Event::make(Lane::Dock(dock_name, std::chrono::seconds(
            10)))}, w1);
      data->graph.add_lane(w1, w0);
    };

  add_bidir_lane(0, 1);  // 0   1
  add_bidir_lane(1, 2);  // 2   3
  add_bidir_lane(1, 5);  // 4   5
  add_bidir_lane(2, 6);  // 6   7
  add_bidir_lane(3, 4);  // 8   9
  add_bidir_lane(4, 5);  // 10 11
  add_bidir_lane(5, 6);  // 12 13
  add_dock_lane(6, 7, "A");  // 14 15
  add_bidir_lane(5, 8);  // 16 17
  add_bidir_lane(6, 9);  // 18 19
  add_bidir_lane(8, 9);  // 20 21
  add_dock_lane(8, 10, "B"); // 22 23

  rmf_traffic::Profile profile{
    rmf_traffic::geometry::make_final_convex<
      rmf_traffic::geometry::Circle>(1.0)
  };

  const rmf_traffic::agv::VehicleTraits traits{
    {0.7, 0.3},
    {1.0, 0.45},
    profile
  };

  data->fleet = data->adapter->add_fleet("test_fleet", traits, data->graph);

  {
    const auto& pimpl = agv::FleetUpdateHandle::Implementation::get(*data->fleet);
    data->node = pimpl.node;
  }

  data->adapter->start();
}

//==============================================================================
auto MockAdapterFixture::add_robot(
  const std::string& name,
  rmf_utils::optional<rmf_traffic::Profile> input_profile) -> RobotInfo
{
  const auto& fleet_impl = rmf_fleet_adapter::agv::FleetUpdateHandle::Implementation::get(*data->fleet);
  const auto& worker = fleet_impl.worker;
  std::cout << "Initial worker queue dump:" << std::endl;
  worker.dump_queue_info();

  rmf_traffic::Profile profile =
    [&]() -> rmf_traffic::Profile
    {
      if (input_profile)
        return *input_profile;

      return rmf_traffic::Profile{
      rmf_traffic::geometry::make_final_convex<
        rmf_traffic::geometry::Circle>(1.0)
      };
    } ();

  const auto now = rmf_traffic_ros2::convert(data->adapter->node()->now());
  const rmf_traffic::agv::Plan::StartSet starts = {{now, 0, 0.0}};

  RobotInfo info;
  info.command = std::make_shared<rmf_fleet_adapter_test::MockRobotCommand>(
    data->node, data->graph);

  bool was_robot_added_yet = false;
  std::size_t failures_to_add = 0;
  while (!was_robot_added_yet)
  {
    std::promise<bool> robot_added;
    auto future = robot_added.get_future();
    data->fleet->add_robot(info.command, name, profile, starts,
      [&info, &robot_added](rmf_fleet_adapter::agv::RobotUpdateHandlePtr updater)
      {
        std::cout << "Calling update handle receiver" << std::endl;
        const auto& pimpl = agv::RobotUpdateHandle::Implementation::get(*updater);
        info.context = pimpl.context.lock();
        info.command->updater = updater;
        std::cout << "Setting promise" << std::endl;
        robot_added.set_value(true);
        std::cout << "Promise is set" << std::endl;
      });

    std::cout << "immediate worker dump:" << std::endl;
    worker.dump_queue_info();

    if (future.wait_for(std::chrono::seconds(5)) == std::future_status::ready)
    {
      was_robot_added_yet = future.get();
    }
    else
    {
      std::cout << " >>> FAILED TO ADD ROBOT, TRYING AGAIN. Dump:" << std::endl;
      worker.dump_queue_info();
      ++failures_to_add;
    }

    if (failures_to_add > 5)
      break;
  }

  if (failures_to_add > 0)
  {
    std::cout << "Failed to add " << failures_to_add << " times" << std::endl;
    throw std::runtime_error(
      "Failed to add " + std::to_string(failures_to_add) + " times");
  }

  return info;
}

//==============================================================================
MockAdapterFixture::~MockAdapterFixture()
{
  std::weak_ptr<rclcpp::Node> weak_node = data->node;
  data.reset();

  std::size_t wait_count = 0;
  while (const auto node = weak_node.lock())
  {
    ++wait_count;
    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    if (wait_count > 200)
      throw std::runtime_error("Node is not dying during test teardown");
  }

  using namespace std::chrono_literals;
//  std::this_thread::sleep_for(5000ms);
}

} // namespace test
} // namespace phases
} // namespace rmf_fleet_adapter
