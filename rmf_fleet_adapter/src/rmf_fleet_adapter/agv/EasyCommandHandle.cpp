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

// #include "EasyCommandHandle.hpp"
// #include "internal_EasyFullControl.hpp"

// namespace rmf_fleet_adapter {
// namespace agv {

// //==============================================================================
// void EasyCommandHandle::follow_new_path(
//   const std::vector<rmf_traffic::agv::Plan::Waypoint>& waypoints,
//   ArrivalEstimator next_arrival_estimator,
//   RequestCompleted path_finished_callback)
// {
//   // DO SOMETHING HERE

//   // PROBABLY SET UP THE VECTOR AND ID

//   EasyFullControl::navigate(
//     const std::string robot_name,
//     const Eigen::Vector3d target_pose,
//     std::string nav_id);
// }

// //==============================================================================
// void EasyCommandHandle::stop()
// {
//   // DO SOMETHING HERE

//   EasyFullControl::stop(
//     const std::string robot_name);
// }

// //==============================================================================
// void EasyCommandHandle::dock(
//   const std::String& dock_name,
//   RequestCompleted docking_finished_callback)
// {
//   // DO SOMETHING HERE
// }

// //==============================================================================
// Eigen::Vector3d EasyCommandHandle::get_position()
// {
//   // DO SOMETHING HERE
// }

// //==============================================================================
// double EasyCommandHandle::get_battery_soc()
// {
//   // DO SOMETHING HERE
// }

// //==============================================================================
// std::size_t EasyCommandHandle::get_current_lane()
// {
//   // DO SOMETHING HERE
// }

// //==============================================================================
// std::optional<double> EasyCommandHandle::get_speed_limit(
//   rmf_traffic::agv::Plan::Waypoint target_waypoint)
// {
//   std::optional<double> speed_limit = std::nullopt;
//   for (const auto& lane_idx : target_waypoint.approach_lanes())
//   {
//     const auto& lane = _graph->get_lane(lane_idx);
//     const auto& lane_limit = lane.properties().speed_limit();
//     if (lane_limit.has_value())
//     {
//       if (speed_limit.has_value())
//         speed_limit = std::min(speed_limit.value(), lane_limit.value());
//       else
//         speed_limit = lane_limit.value();
//     }
//   }
//   return speed_limit;
// }

// //==============================================================================
// void EasyCommandHandle::set_updater()
// {
//   // DO SOMETHING HERE
// }

// //==============================================================================
// void EasyCommandHandle::update()
// {
//   // DO SOMETHING HERE
// }

// //==============================================================================
// void EasyCommandHandle::update_state()
// {
//   // DO SOMETHING HERE
// }

// //==============================================================================
// void EasyCommandHandle::set_action_execution(
//   ActionExecution action_execution)
// {
//   _action_execution = action_execution;
// }

// //==============================================================================
// void EasyCommandHandle::complete_robot_action()
// {
//   if (!_action_execution.has_value())
//     return;

//   _action_execution->finished();
//   _action_execution = std::nullopt;

//   RCLCPP_INFO(
//     _node->get_logger(),
//     "Robot [%s] has completed the action it was performing",
//     _travel_info.robot_name.c_str());
// }

// //==============================================================================
// // EasyFullControlPtr EasyFullControl::Implementation::make(
// //   RobotUpdateHandlePtr update_handle_,
// //   std::shared_ptr<Node> node_)
// // {
// //   std::shared_ptr<EasyFullControl> handle(new EasyFullControl);
// //   handle->_pimpl = rmf_utils::make_unique_impl<Implementation>(
// //     std::move(update_handle_),
// //     std::move(node_));

// //   return handle;
// // }

// // // //==============================================================================
// // EasyFullControl::Implementation::Implementation(
// //   RobotUpdateHandlePtr update_handle_,
// //   std::shared_ptr<Node> node_)
// // : update_handle(std::move(update_handle_)),
// //   node(std::move(node_))
// // {
// //   // Do nothing
// // }

// //==============================================================================
// EasyFullControl::EasyFullControl()
// {
//   // Do nothing
// }

// } // namespace agv
// } // namespace rmf_fleet_adapter
