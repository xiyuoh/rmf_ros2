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

#include "../phases/DoorOpen.hpp"
#include "../phases/DoorClose.hpp"
#include "../phases/RequestLift.hpp"
#include "../phases/EndLiftSession.hpp"

namespace rmf_fleet_adapter {
namespace agv {

//==============================================================================
class PhaseEvent : public Event
{
public:

  class Active
    : public Event::Active,
      public std::enable_shared_from_this<Active>
  {
  public:

    static std::shared_ptr<Event::Active> make(
      rxcpp::schedulers::worker worker,
      std::shared_ptr<Task::PendingPhase> pending)
    {
      auto active = std::shared_ptr<Active>(
        new Active(std::move(worker), pending->begin()));

      active->_connect();
      return active;
    }

    const rxcpp::observable<Empty> & observe() const final
    {
      return _obs;
    }

  private:

    Active(
      rxcpp::schedulers::worker worker,
      std::shared_ptr<rmf_fleet_adapter::Task::ActivePhase> phase)
    : _worker(worker),
      _phase(std::move(phase))
    {
      // Do nothing
    }

    void _connect()
    {
      _obs = _pub.get_observable();
      _status_subscription = _phase->observe()
        .observe_on(rxcpp::identity_same_worker(_worker))
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
      });
    }

    rxcpp::schedulers::worker _worker;
    std::shared_ptr<Task::ActivePhase> _phase;
    rxcpp::observable<Empty> _obs;
    rxcpp::subjects::subject<Empty> _pub;
    rmf_rxcpp::subscription_guard _status_subscription;
  };

  using PhaseFactory =
    std::function<
      std::shared_ptr<Task::PendingPhase>(
        std::string requester_id,
        NodePtr node,
        std::function<void()> waiting_cb
      )
    >;

  PhaseEvent(PhaseFactory phase)
  : _phase(std::move(phase))
  {
    // Do nothing
  }

  std::shared_ptr<Event::Active> begin(
    std::string requester_id,
    NodePtr node,
    std::function<void()> waiting_cb,
    rxcpp::schedulers::worker worker) const final
  {
    return Active::make(
      std::move(worker),
      _phase(std::move(requester_id), std::move(node), std::move(waiting_cb))
    );
  }

private:
  PhaseFactory _phase;
};

namespace {
//==============================================================================
class GraphEventPhaseFactory : public rmf_traffic::agv::Graph::Lane::Executor
{
public:

  GraphEventPhaseFactory()
  {
    // Do nothing
  }

  void execute(const Dock&) final
  {
    // The traffic light API does not do anything to support docking events
    // TODO(MXG): Should we warn the user, or just let this slide?
  }

  void execute(const DoorOpen& open) final
  {
    phase = [open](
      std::string requester_id,
      NodePtr node,
      std::function<void()> waiting_cb)
    {
      return std::make_shared<phases::DoorOpen::PendingPhase>(
        node, open.name(), std::move(requester_id), std::move(waiting_cb));
    };
  }

  void execute(const DoorClose& close) final
  {
    phase = [close](
      std::string requester_id,
      NodePtr node,
      std::function<void()>)
    {
      return std::make_shared<phases::DoorClose::PendingPhase>(
        node, close.name(), std::move(requester_id));
    };
  }

  void execute(const LiftSessionBegin& begin) final
  {
    phase = [begin](
      std::string requester_id,
      NodePtr node,
      std::function<void()> waiting_cb)
    {
      return std::make_shared<phases::RequestLift::PendingPhase>(
        std::move(requester_id),
        std::move(node),
        begin.lift_name(),
        begin.floor_name(),
        std::move(waiting_cb),
        nullptr,
        rmf_traffic::Duration(0),
        phases::RequestLift::Located::Outside);
    };
  }

  void execute(const LiftMove& move) final
  {
    phase = [move](
      std::string requester_id,
      NodePtr node,
      std::function<void()> waiting_cb)
    {
      return std::make_shared<phases::RequestLift::PendingPhase>(
        std::move(requester_id),
        std::move(node),
        move.lift_name(),
        move.floor_name(),
        std::move(waiting_cb),
        nullptr,
        rmf_traffic::Duration(0),
        phases::RequestLift::Located::Inside);
    };
  }

  void execute(const LiftDoorOpen& open) final
  {
    phase = [open](
      std::string requester_id,
      NodePtr node,
      std::function<void()> waiting_cb)
    {
      return std::make_shared<phases::RequestLift::PendingPhase>(
        std::move(requester_id),
        std::move(node),
        open.lift_name(),
        open.floor_name(),
        std::move(waiting_cb),
        nullptr,
        rmf_traffic::Duration(0),
        phases::RequestLift::Located::Inside);
    };
  }

  void execute(const LiftSessionEnd& close) final
  {
    phase = [close](
      std::string requester_id,
      NodePtr node,
      std::function<void()>)
    {
      return std::make_shared<phases::EndLiftSession::Pending>(
        std::move(requester_id),
        std::move(node),
        close.lift_name(),
        close.floor_name());
    };
  }

  PhaseEvent::PhaseFactory phase;

};
} // anonymous namespace

//==============================================================================
ConstEventPtr make_event(const rmf_traffic::agv::Graph::Lane::Event& event)
{
  TODO: Create a GraphEventPhaseFactory and produce a phase factory for the
      event that we have been given.

      If the phase given by the factory is a nullptr, then we should return a
      nullptr.
}

} // namespace agv
} // namespace rmf_fleet_adapter
