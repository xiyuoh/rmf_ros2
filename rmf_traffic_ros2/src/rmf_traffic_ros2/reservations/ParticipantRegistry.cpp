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
#include "ParticipantRegistry.hpp"

namespace rmf_traffic_ros2 {

bool ParticipantRegistry::add(
  ParticipantId pid,
  std::shared_ptr<Participant> participant)
{
  //TODO: This is where persistence should hook in.
  _participant_table.insert_or_assign(
    pid,
    participant
  );
  return true;
}

void ParticipantRegistry::unregister_participant(
  ParticipantId pid)
{
  _participant_table.erase(pid);
}

std::shared_ptr<Participant> ParticipantRegistry::get(ParticipantId pid)
{
  return _participant_table[pid];
}

}
