// /*
//  * Copyright (C) 2022 Open Source Robotics Foundation
//  *
//  * Licensed under the Apache License, Version 2.0 (the "License");
//  * you may not use this file except in compliance with the License.
//  * You may obtain a copy of the License at
//  *
//  *     http://www.apache.org/licenses/LICENSE-2.0
//  *
//  * Unless required by applicable law or agreed to in writing, software
//  * distributed under the License is distributed on an "AS IS" BASIS,
//  * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
//  * See the License for the specific language governing permissions and
//  * limitations under the License.
//  *
// */

// #ifndef RMF_FLEET_ADAPTER__AGV__EASYCOMMANDHANDLE_HPP
// #define RMF_FLEET_ADAPTER__AGV__EASYCOMMANDHANDLE_HPP

// #include <rmf_fleet_adapter/agv/RobotUpdateHandle.hpp>

// #include <rmf_traffic/agv/Planner.hpp>
// #include "Node.hpp"

// namespace rmf_fleet_adapter {
// namespace agv {

// //==============================================================================
// class EasyCommandHandle
// {
// public:

//   using Duration = rmf_traffic::Duration;

//   using ArrivalEstimator =
//     std::function<void(std::size_t path_index, Duration remaining_time)>;
//   using RequestCompleted = std::function<void()>;

//   EasyCommandHandle(
//     rclcpp::Node node,
//     const std::string fleet_name,
//     const std::string robot_name,
//     const rmf_traffic::agv::Graph graph,
//     const rmf_traffic::agv::VehicleTraits traits,
//     const std::string map_name,
//     std::vector<Plan::Start> starts="", // remember change default to starts[0]
//     const Eigen::Vector3d position=position, // remember change default
//     const std::string charger_waypoint="", // change default
//     // const int max_delay, // check again
//     const int update_frequency=1 // change default
//   );

//     // NEEEEEEED WHOLE CONFIG FILE. FIGURE OUT HOW TO GET IT

//   rclcpp::Node node;
//   const std::string fleet_name;
//   const std::string robot_name;
//   const rmf_traffic::agv::Graph graph;
//   const rmf_traffic::agv::VehicleTraits traits;
//   const std::string map_name;
//   // index of charger waypoint: waypoints = graph.find_waypoint(charger_waypoint)
// //   assert (waypoint);
// //   std::size_t charger_waypoint_index = waypoint.index;
//   const int update_frequency;
//   RobotUpdateHandle update_handle;
//   double battery_soc;
//   Eigen::Vector3d position;
//   bool initialized = false;
//   // state = idle

//   std::vector<Eigen::Vector3d> remaining_waypoints;
//   RequestCompleted path_finished_callback;
//   ArrivalEstimator next_arrival_estimator;
//   std::size_t path_index;
//   //docking finished callback
//   // perform filtering = self.config["filter_waypoints"]

//   // RMF location trackers
//   std::size_t last_known_lane_index;
//   std::size_t last_known_waypoint_index;
//   // if robot is waiting at a waypoint. This is a Graph::Waypoint index
//   std::size_t on_waypoint;
//   // if robot is travelling on a lane. This is a Graph::Lane index
//   std::size_t on_lane;
//   rmf_traffic::agv::Plan::Waypoint target_waypoint;
//   // The graph index of the waypoint the robot is currently docking into
//   std::size_t dock_waypoint_index;

//   void follow_new_path(
//     const std::vector<rmf_traffic::agv::Plan::Waypoint>& waypoints,
//     ArrivalEstimator next_arrival_estimator,
//     RequestCompleted path_finished_callback) = 0;

//   void stop() = 0;

//   void dock(
//     const std::string& dock_name,
//     RequestCompleted docking_finished_callback) = 0;

//   Eigen::Vector3d get_position();

//   double get_battery_soc();

//   std::size_t get_current_lane();

//   std::optional<double> get_speed_limit(
//     rmf_traffic::agv::Plan::Waypoint target_waypoint);

//   void set_updater();

//   void update();

//   void update_state();

//   using ActionExecution =
//     rmf_fleet_adapter::agv::RobotUpdateHandle::ActionExecution;

//   void set_action_execution(
//     ActionExecution action_execution);

//   void complete_robot_action();

//   // Destructor
// //   ~EasyCommandHandle();
// }

// } // namespace agv
// } // namespace rmf_fleet_adapter

// #endif // RMF_FLEET_ADAPTER__AGV__EASYCOMMANDHANDLE_HPP