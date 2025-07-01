/******************************************************************************
 * Copyright 2023 The Apollo Authors. All Rights Reserved.
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
 *****************************************************************************/

#include "modules/planning/scenarios/edge_follow/edge_follow_scenario.h"

#include "cyber/common/log.h"
#include "modules/common_msgs/planning_msgs/pad_msg.pb.h"
#include "modules/planning/scenarios/edge_follow/edge_follow_stage.h"

namespace apollo {
namespace planning {

bool EdgeFollowScenario::IsTransferable(const Scenario* other_scenario,
                                        const Frame& frame) {
  if (frame.reference_line_info().empty()) {
    return false;
  }

  const auto action = frame.GetPadMsgDrivingAction();
  if (other_scenario &&
      other_scenario->Name() == "EdgeFollowScenario") {
    return action != PadMessage::EXIT_MISSION;
  }
  return action == PadMessage::ENTER_MISSION;
}

}  // namespace planning
}  // namespace apollo
