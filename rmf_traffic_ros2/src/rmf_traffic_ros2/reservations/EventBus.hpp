/*
 * Copyright (C) 2019 Open Source Robotics Foundation
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
#ifndef __RMF_TRAFFIC_ROS2__INTERNAL__SRC__EVENT_BUS_HPP__
#define __RMF_TRAFFIC_ROS2__INTERNAL__SRC__EVENT_BUS_HPP__

namespace rmf_traffic_ros2 {

#include <unordered_map>
#include <mutex>
#include <condition_variable>

///=============================================================================
/// This class is helper function to faciltate synchronization between the ROS2
/// node and the actual database thread. It essentially acts as a 
template<typename T, typename Message>
class EventNotificationBus
{
public:
  ///===========================================================================
  /// \brief Wait for a fixed interval of time for a notification
  /// if not return a std::nullopt.
  /// \param[in] duration - duration till timeout. If the system times
  /// out return an `std::nullopt`
  /// \param[in] mailbox_id - the mail
  template<class Rep, class Period>
  std::optional<Message> wait_for(
    std::chrono::duration<Rep, Period> duration,
    T mailbox_id)
  {
    std::unique_lock<std::mutex> lock(_lock[mailbox_id]);
    auto res = _condvars[mailbox_id]
      .wait_for(
        lock,
        duration,
        [this, mailbox_id]()->bool{
          return _ready[mailbox_id].has_value();
        }
      );
    lock.unlock();
    if (!res)
    {
      return std::nullopt;
    }
    return _ready[mailbox_id];
  }

  ///===========================================================================
  /// Send a notification to 
  void notify_mailbox(T mailbox, Message m)
  {
    {
      std::lock_guard<std::mutex> guard(_lock[mailbox]);
      _ready[mailbox].insert_or_assign({m});
    }
    _condvars[mailbox].notify_all();
  }
private:
  std::unordered_map<T, std::condition_variable> _condvars;
  std::unordered_map<T, std::mutex> _lock;
  std::unordered_map<T, std::optional<Message>> _ready;
};
}

#endif