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
#include <rmf_fleet_adapter/agv/RobotCommandHandle.hpp>

#include <rmf_fleet_adapter/StandardNames.hpp>
#include <rmf_fleet_msgs/msg/mode_request.hpp>
#include <rmf_fleet_msgs/msg/lane_request.hpp>
#include <rmf_fleet_msgs/msg/closed_lanes.hpp>
#include <rmf_fleet_msgs/msg/interrupt_request.hpp>

#include <rmf_traffic_ros2/Time.hpp>

#include "Node.hpp"
#include <yaml-cpp/yaml.h>

namespace rmf_fleet_adapter {
namespace agv {

//==============================================================================
class EasyCommandHandle
  : public RobotCommandHandle,
  public std::enable_shared_from_this<EasyCommandHandle>
{
public:

  using Navigate = EasyFullControl::Navigate;
  using GetRobotState = EasyFullControl::GetRobotState;
  using ProcessCompleted = EasyFullControl::ProcessCompleted;

  enum robotStatus {
    IDLE = 0,
    WAITING = 1,
    MOVING = 2
  };

  EasyCommandHandle(
    rclcpp::Node& node,
    std::string& fleet_name,
    std::string& robot_name,
    std::shared_ptr<const Graph> graph,
    std::shared_ptr<const VehicleTraits> traits,
    std::string& map_name,
    std::optional<rmf_traffic::Duration> max_delay,
    std::string& charger_waypoint,
    GetRobotState get_state,
    std::function<ProcessCompleted(const Navigate command)> navigate,
    ProcessCompleted stop,
    RobotUpdateHandle::ActionExecutor action_executor)
  : _node(&node),
    _robot_name(std::move(robot_name)),
    _fleet_name(std::move(fleet_name)),
    _graph(graph),
    _traits(traits),
    _map_name(std::move(map_name)),
    _max_delay(std::move(max_delay)),
    _charger_waypoint(std::move(charger_waypoint)),
    _get_state(std::move(get_state)),
    _navigate(std::move(navigate)),
    _stop(std::move(stop)),
    _action_executor(std::move(action_executor))
  {
    // Set up subscriptions
    _mode_request_sub = _node->create_subscription<rmf_fleet_msgs::msg::ModeRequest>(
      "/action_execution_notice",
      rclcpp::SystemDefaultsQos(),
      [w = weak_from_this(), fleet_name](
        rmf_fleet_msgs::msg::ModeRequest::UniquePtr msg)
      {
        if (msg->fleet_name.empty() || msg->fleet_name != fleet_name || msg->robot_name->empty())
          return;
        if (msg->mode.mode == msg->mode.MODE_IDLE)
        {
          const auto self = w.lock();
          if (!self)
            return;

          w->complete_robot_action();
        }
      });
  }

  void follow_new_path(
    const std::vector<rmf_traffic::agv::Plan::Waypoint>& waypoints,
    ArrivalEstimator next_arrival_estimator,
    RequestCompleted path_finished_callback) final
  {
    auto lock = _lock();
    _clear_last_command();

    RCLCPP_INFO(_node->get_logger(), "Received new path for %s", _robot_name.c_str());

    _remaining_waypoints = waypoints;
    assert(!next_arrival_estimator);
    assert(!path_finished_callback);
    _next_arrival_estimator = std::move(next_arrival_estimator);
    _path_finished_callback = std::move(path_finished_callback);
    _interrupted = false;

    Eigen::Vector3d target_pose;
    while (!_remaining_waypoints.empty() || _status == MOVING || _status == WAITING)
    {
      // TODO: Check if we need to abort

      // State machine
      if (_status == IDLE)
      {
        // Assign the next waypoint
        _target_waypoint = _remaining_waypoints.front();
        _path_index = _target_waypoint->graph_index();
        // Move robot to next waypoint
        target_pose = _target_waypoint->position();

        // The speed limit is set as the minimum of all the approach lanes' limits
        std::optional<double> speed_limit = std::nullopt;
        for (const auto& lane_idx : _target_waypoint->approach_lanes())
        {
          const auto& lane = _graph->get_lane(lane_idx);
          const auto& lane_limit = lane.properties().speed_limit();
          if (lane_limit.has_value())
          {
            if (speed_limit.has_value())
              speed_limit = std::min(speed_limit.value(), lane_limit.value());
            else
              speed_limit = lane_limit.value();
          }
        }
        Navigate current_command;
        current_command.pose = target_pose;
        current_command.map_name = _map_name;
        current_command.speed_limit = speed_limit;
        ProcessCompleted navigate_cb = _navigate(current_command);

        if (navigate_cb())
        {
          _remaining_waypoints.erase(_remaining_waypoints.begin());
          _status = MOVING;
        }
        else
        {
          RCLCPP_INFO(_node->get_logger(),
            "Robot %s failed to navigate to [%f, %f, %f] coordinates. Retrying...",
            _robot_name.c_str(), target_pose.x(), target_pose.y(), target_pose.z());
        }

      }
      else if (_status == WAITING)
      {
        // sleep for 0.1?
        auto time_now = rmf_traffic_ros2::convert(_node->now());
        // with self lock?
        if (_target_waypoint.has_value())
        {
          auto waypoint_wait_time = _target_waypoint->time();
          if (waypoint_wait_time < time_now)
          {
            _status = IDLE;
          }
          else
          {
            if (_path_index.has_value())
            {
              auto d = rmf_traffic::time::to_seconds(waypoint_wait_time - time_now);
              RCLCPP_DEBUG(_node->get_logger(),
                "Waiting for %fs", d);
              _next_arrival_estimator(_path_index, rmf_traffic::time::from_seconds(0.0));
            }
          }
        }
      }
      else if (_status == MOVING)
      {
        // sleep for 0.1?
        // with self lock?
        if (_navigation_completed.has_value() && _navigation_completed())
        {
          RCLCPP_INFO(_node->get_logger(),
            "Robot %s has reached its target waypoint.", _robot_name.c_str());
          _status = WAITING;
          if (_target_waypoint->graph_index().has_value())
          {
            _on_waypoint = _target_waypoint->graph_index();
            _last_known_waypoint_index = _on_waypoint;
          }
          else
          {
            _on_waypoint = std::nullopt; // still on a lane
          }
        }
        else
        {
          // Update the lane the robot is on
          const auto lane = get_current_lane();
          if (lane.has_value())
          {
            _on_waypoint = std::nullopt;
            _on_lane = lane;
          }
          else
          {
            // The robot may either be on the previous waypoint or the target one
            if (_target_waypoint->graph_index().has_value() && dist(_position, target_pose) < 0.5)
            {
              _on_waypoint = _target_waypoint->graph_index();
            }
            else if (_last_known_waypoint_index.has_value() && dist(_position, _graph->get_waypoint(_last_known_waypoint_index).get_location()) < 0.5)
            {
              _on_waypoint = _last_known_waypoint_index;
            }
            else
            {
              _on_lane = std::nullopt; // update_off_grid() ?????
              _on_waypoint = std::nullopt;
            }
          }
        }

        // Calculate navigation remaining duration manually (?)
        double dist_to_target = dist(_position, _target_pose);
        double ori_delta = abs(_position.z - _target_pose.z);
        if (ori_delta > M_PI)
          ori_delta = ori_delta - (2 * M_PI);
        if (ori_delta < M_PI)
          ori_delta = (2 * M_PI) + ori_delta;

        // Calculate duration in seconds
        double duration = (dist_to_target / _traits->linear().get_nominal_velocity() + 
                           ori_delta / _traits->rotational().get_nominal_velocity());

        if (_path_index.has_value())
        {
          auto target_time = _target_waypoint->time();
          auto now = rmf_traffic_ros2::convert(_node->now());
          if (target_time < now + rmf_traffic::time::from_seconds(duration))
          {
            _next_arrival_estimator(_path_index, rmf_traffic::time::from_seconds(duration));
          }
          else
          {
            _next_arrival_estimator(_path_index, rmf_traffic::time::to_seconds(target_time - now));
          }
        }
      }
    }

    _path_finished_callback();
    RCLCPP_INFO(_node->get_logger(),
      "Robot %s has successfully navigated along requested path.", _robot_name.c_str());
  }

  void stop() final
  {
    auto lock = _lock();
    _clear_last_command();
    bool success = _stop();

    if (!success)
      RCLCPP_INFO(_node->get_logger(),
        "Robot %s failed to stop.", _robot_name.c_str());
  }

  void dock(
    const std::string& dock_name,
    RequestCompleted docking_finished_callback) final
  {
    // TODO
  }

  void set_updater(RobotUpdateHandlePtr updater)
  {
    _updater = std::move(updater);

    // Set the action_executor for the robot
    const auto action_executioner =
      [w = weak_from_this()](
      const std::string&,
      const nlohmann::json&,
      RobotUpdateHandle::ActionExecution execution)
      {
        // We do not do anything here. The user can can move the robot by
        // sending PathRequest msgs. Instead we simply store the completed
        // callback which will be called when we receive a RobotModeRequest.
        const auto self = w.lock();
        if (!self)
          return;
        self->set_action_execution(execution);
      };
    _updater->set_action_executor(action_executioner);

    // Set max delay
    if (_max_delay != std::nullopt)
    {
      RCLCPP_INFO(_node->get_logger(),
        "Setting max delay to %fs", _max_delay);
    }
    _updater->maximum_delay(_max_delay);

    // Set charger waypoint index
    if (_charger_waypoint.has_value())
    {
      auto waypoint = _graph->find_waypoint(_charger_waypoint);
      assert(waypoint == nullptr && "Charger waypoint provided does not exist in the navigation graph.");

      if (waypoint->index() < _graph->num_waypoints())
        _updater->set_charger_waypoint(_charger_waypoint);
    }
    else
    {
      RCLCPP_INFO(_node->get_logger(),
        "Invalid waypoint supplied for charger. "
        "Using default nearest charger in the map.");
    }

  }

  rmf_utils::optional<std::size_t> get_current_lane()
  {
    if (!_target_waypoint->has_value())
    {
      return std::nullopt;
    }

    const auto& current_lanes = _target_waypoint->approach_lanes();
    if (current_lanes.size() == 0)
    {
      return std::nullopt;
    }

    for (const auto& l : current_lanes)
    {
      const auto& lane = _graph->get_lane(l);
      const auto& current_waypoint = _graph->get_waypoint(_on_waypoint);
      const Eigen::Vector2d p = current_waypoint.get_location();

      const auto& wp0 =
        _graph->get_waypoint(lane.entry().waypoint_index());
      const Eigen::Vector2d p0 = wp0.get_location();

      const auto& wp1 =
        _graph->get_waypoint(lane.exit().waypoint_index());
      const Eigen::Vector2d p1 = wp1.get_location();

      const bool before_blocked_lane = (p-p0).dot(p1-p0) < 0.0;
      const bool after_blocked_lane = (p-p1).dot(p1-p0) >= 0.0;
      if (!before_blocked_lane && !after_blocked_lane)
      {
        return l;
      }
    }

    return std::nullopt;
  }

  void newly_closed_lanes(const std::unordered_set<std::size_t>& closed_lanes)
  {
    bool need_to_replan = false;
    const auto& current_lane = get_current_lane();

    // get_current_lane() already checks that _target_waypoint and approach_lanes() are not empty
    if (current_lane.has_value())
    {
      const auto& current_lanes = _target_waypoint->approach_lanes();
      for (const auto& l : current_lanes)
      {
        if (closed_lanes.count(l))
        {
          need_to_replan = true;
          // The robot is currently on a lane that has been closed.
          // We take this to mean that the robot needs to reverse.
          if (l == current_lane)
          {
            const auto& lane = _graph->get_lane(l);
            const auto& return_waypoint = lane.entry().waypoint_index();
            const auto* reverse_lane =
              _graph->lane_from(lane.entry().waypoint_index(),
                                lane.exit().waypoint_index())

            // Lock?
            if (reverse_lane)
            {
              // Update current lane to reverse back to start of the lane
              _on_lane = reverse_lane->index();
            }
            else
            {
              // Update current position and waypoint index to return to
              _target_waypoint = return_waypoint;
            }
          }
        }
      }
    }

    if (!need_to_replan && _target_waypoint->has_value())
    {
      // Check if the remainder of the current plan has been invalidated by the
      // lane closure.
      const auto next_index = *_target_waypoint->index();
      for (std::size_t i = next_index; i < _remaining_waypoints.size(); ++i)
      {
        for (const auto& lane : _remaining_waypoints[i].approach_lanes())
        {
          if (closed_lanes.count(lane))
          {
            need_to_replan = true;
            break;
          }
        }

        if (need_to_replan)
          break;
      }
    }

    if (need_to_replan)
      _updater->replan();
  }

  void set_action_execution(RobotUpdateHandle::ActionExecution action_execution)
  {
    _action_execution = action_execution;
  }

  void complete_robot_action()
  {
    if (!_action_execution.has_value())
      return;

    _action_execution->finished();
    _action_execution = std::nullopt;

    RCLCPP_INFO(
      _node->get_logger(),
      "Robot [%s] has completed the action it was performing",
      _robot_name.c_str());
  }

  void handle_interrupt_request(
    const rmf_fleet_msgs::msg::InterruptRequest& request)
  {
    const auto it = _interruptions.find(request.interrupt_id);
    if (it == _interruptions.end())
    {
      if (request.type == request.TYPE_RESUME)
        return;

      _interruptions.insert(
        {
          request.interrupt_id,
          _updater->interrupt(
            request.labels,
            [id = request.interrupt_id, name = _robot_name]()
            {
              std::cout << "[" << name << "] is interrupted for " << id
                        << "!" << std::endl;
            })
        });

      return;
    }
    else
    {
      if (request.type == request.TYPE_INTERRUPT)
        return;

      it->second.resume(request.labels);
      std::cout << "Asking [" << _robot_name << "] to resume for "
                << request.interrupt_id << std::endl;

      _interruptions.erase(it);
    }
  }

  double dist(Eigen::Vector3d a, Eigen::Vector3d b)
  {
    const Eigen::Vector3d dp = a - b;
    const double distance = dp.norm();

    return distance;
  }


private:
  rclcpp::Node* _node;
  const std::string& _robot_name;
  const std::string& _fleet_name;
  std::shared_ptr<const Graph> _graph;
  std::shared_ptr<const VehicleTraits> _traits;
  std::string _map_name;
  std::optional<rmf_traffic::Duration> _max_delay = std::nullopt;
  rmf_utils::optional<std::size_t> _charger_waypoint;
  GetRobotState _get_state;
  std::function<ProcessCompleted(const Navigate command)> _navigate;
  ProcessCompleted _stop;
  RobotUpdateHandle::ActionExecutor _action_executor;
  rmf_utils::optional<ProcessCompleted> _navigation_completed;

  RobotUpdateHandlePtr _updater;
  Eigen::Vector3d _position;
  robotStatus _status;
  rclcpp::Subscription<rmf_fleet_msgs::msg::ModeRequest> _mode_request_sub;

  using Interruption = RobotUpdateHandle::Interruption;
  std::unordered_map<std::string, Interruption> _interruptions;
  bool _interrupted = false;

  std::vector<rmf_traffic::agv::Plan::Waypoint> _requested_waypoints;
  std::vector<rmf_traffic::agv::Plan::Waypoint> _remaining_waypoints;
  ArrivalEstimator _next_arrival_estimator;
  RequestCompleted _path_finished_callback;
  RequestCompleted _dock_finished_callback;
  rmf_utils::optional<std::size_t> _path_index;

  rmf_utils::optional<std::size_t> _last_known_lane_index;
  rmf_utils::optional<std::size_t> _last_known_waypoint_index;
  rmf_utils::optional<Graph::Waypoint> _on_waypoint;
  rmf_utils::optional<Graph::Lane> _on_lane;
  rmf_utils::optional<rmf_traffic::agv::Plan::Waypoint> _target_waypoint;
  rmf_utils::optional<std::size_t> _dock_waypoint_index;
  rmf_utils::optional<std::size_t> _action_waypoint_index;

  void _clear_last_command()
  {
    _next_arrival_estimator = nullptr;
    _path_finished_callback = nullptr;
    _dock_finished_callback = nullptr;
    _status = IDLE;
  }

  std::recursive_mutex _mutex;
  std::unique_lock<std::recursive_mutex> _lock()
  {
    std::unique_lock<std::recursive_mutex> lock(_mutex, std::defer_lock);
    while (!lock.try_lock())
    {
      // Intentionally busy wait
    }

    return lock;
  }
};

using EasyCommandHandlePtr = std::shared_ptr<EasyCommandHandle>;

//==============================================================================
class EasyFullControl::Implementation
{
public:

  Implementation(
    Configuration config_,
    AdapterPtr adapter_)
  : config{std::move(config_)}
  {
    adapter = adapter_;
    auto success = initialize_fleet();

    if (success)
    {
      RCLCPP_INFO(adapter_->node()->get_logger(), "Start Fleet Adapter");
      adapter_->start().wait();

      RCLCPP_INFO(adapter_->node()->get_logger(), "Closing Fleet Adapter");
      rclcpp::shutdown();
    }
    else
    {
      RCLCPP_ERROR(adapter_->node()->get_logger(), "Unable to initialize fleet");
    }
  }

  bool initialize_fleet();

// private:
  const Configuration config;
  AdapterPtr adapter;
  std::string fleet_name;
  FleetUpdateHandlePtr fleet_handle;
  YAML::Node fleet_config;
  std::shared_ptr<const Graph> graph;
  std::shared_ptr<const VehicleTraits> traits;
  rclcpp::Subscription<rmf_fleet_msgs::msg::LaneRequest>::SharedPtr lane_closure_request_sub;
  rclcpp::Publisher<rmf_fleet_msgs::msg::ClosedLanes>::SharedPtr closed_lanes_pub;
  std::unordered_set<std::size_t> closed_lanes;
  std::unordered_map<std::string, EasyCommandHandlePtr> robots;
};

} // namespace agv
} // namespace rmf_fleet_adapter

#endif // SRC__RMF_FLEET_ADAPTER__AGV__INTERNAL_EASYFULLCONTROL_HPP
