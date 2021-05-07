/*
 * Copyright (C) 2021 Open Source Robotics Foundation
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

#ifndef SRC__RMF_FLEET_ADAPTER__AGV__INTERNAL_EVENT_HPP
#define SRC__RMF_FLEET_ADAPTER__AGV__INTERNAL_EVENT_HPP

#include <rmf_fleet_adapter/agv/Event.hpp>
#include "RobotContext.hpp"

#include <rxcpp/rx-observable.hpp>

namespace rmf_fleet_adapter {
namespace agv {

//==============================================================================
class Event
{
public:

  class Active
  {
  public:

    struct Empty
    {
      // Right now we don't do anything about status updates from events. We
      // only care about when the finished signal is issued. We can change this
      // in the future, but for now we'll just subscribe to an empty message
      // from the events.
    };

    virtual const rxcpp::observable<Empty>& observe() const = 0;
  };

  virtual std::shared_ptr<Active> begin(
    std::string requester_id,
    NodePtr node,
    std::function<void()> waiting_cb,
    rxcpp::schedulers::worker worker) const = 0;

};

} // namespace agv
} // namespace rmf_fleet_adapter

#endif // SRC__RMF_FLEET_ADAPTER__AGV__INTERNAL_EVENT_HPP
