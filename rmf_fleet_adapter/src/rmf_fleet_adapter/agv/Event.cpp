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

#include "internal_Event.hpp"
#include "../Task.hpp"

namespace rmf_fleet_adapter {
namespace agv {

//==============================================================================
class GraphEvent : public Event
{
public:

  class Active
    : public Event::Active,
      public std::enable_shared_from_this<Active>
  {
  public:

    Active(
      RobotContextPtr context,
      std::shared_ptr<rmf_fleet_adapter::Task::ActivePhase> phase)
    : _context(context),
      _phase(std::move(phase))
    {
      // Do nothing
    }

  private:

    void _connect()
    {
      _obs = _pub.get_observable();
      _status_subscription = _phase->observe()
        .observe_on(rxcpp::identity_same_worker(_context->worker()))
        .subscribe(
        [](const auto&)
      {
        // Do nothing for on_next
      },
        [w = weak_from_this()]()
      {
        if (const auto& self = w.lock())
        {
          // Call on_completed when the phase is completed
          self->_pub.get_subscriber().on_completed();
        }
      }
      );
    }

    RobotContextPtr _context;
    std::shared_ptr<Task::ActivePhase> _phase;
    rxcpp::observable<Empty> _obs;
    rxcpp::subjects::subject<Empty> _pub;
    rmf_rxcpp::subscription_guard _status_subscription;
  };

  GraphEvent(const rmf_traffic::agv::Graph::Lane::Event& event)
  {
    _create_phase(event);
  }

private:

  void _create_phase(const rmf_traffic::agv::Graph::Lane::Event& event);

  std::shared_ptr<Task::PendingPhase> _phase;
};

} // namespace agv
} // namespace rmf_fleet_adapter
