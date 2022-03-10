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

#ifndef RMF_FLEET_ADAPTER__AGV__EASYFULLCONTROL_HPP
#define RMF_FLEET_ADAPTER__AGV__EASYFULLCONTROL_HPP

#include <Eigen/Geometry>
#include <memory>

namespace rmf_fleet_adapter {
namespace agv {

//==============================================================================
class EasyFullControl : public std::enable_shared_from_this<EasyFullControl>
{
public:

  using NavigationCompleted = std::function<bool(std::string id)>;

  /// Set the navigation function from your Robot API
  ///
  /// \param[in] navigate
  ///  The API function for navigating your robot to a pose
  ///  Takes in the robot name, pose and string ID of the navigation task
  ///  Returns a NavigationCompleted to check status of navigation task

  void navigate(
    std::function<NavigationCompleted(
      const std::string name,
      const Eigen::Vector3d pose,
      std::string id)>);

  /// Stop the robot.
  ///
  /// \param[in] name
  ///   The name of the robot the stop command is sent to.
  void stop(const std::string name);

  using ActionCompleted = std::function<bool(std::string id)>;

  /// Set the start process function from your Robot API
  ///
  /// \param[in] start_process
  ///  The API function for requesting your robot to complete an action/process
  ///  Takes in the robot name, action name, and string ID of the action
  ///  Returns a ActionCompleted to check status of action task

  void perform_action(
    std::function<ActionCompleted(
      const std::string name,
      const std::string action,
      std::string id)>);


  class Implementation;

private:
  EasyFullControl();
  rmf_utils::unique_impl_ptr<Implementation> _pimpl;
};

using EasyFullControlPtr = std::shared_ptr<EasyFullControl>;

} // namespace agv
} // namespace rmf_fleet_adapter

#endif // RMF_FLEET_ADAPTER__AGV__EASYFULLCONTROL_HPP
