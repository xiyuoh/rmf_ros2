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

#include "Participant.hpp"

#include <unordered_map>

namespace rmf_traffic_ros2 {

using ParticipantId = rmf_traffic::reservations::ParticipantId;

class ParticipantRegistry
{
public:
  bool add(ParticipantId pid, std::shared_ptr<Participant> participant);

  void unregister_participant(ParticipantId pid);

  std::shared_ptr<Participant> get(ParticipantId pid);
private:
  std::unordered_map<ParticipantId, std::shared_ptr<Participant>> 
    _participant_table;
};
}