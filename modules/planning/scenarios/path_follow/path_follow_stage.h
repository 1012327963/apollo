#pragma once

#include "modules/planning/planning_interface_base/scenario_base/stage.h"

namespace apollo {
namespace planning {

class PathFollowStage : public Stage {
 public:
  StageResult Process(const common::TrajectoryPoint& planning_start_point,
                      Frame* frame) override;
};

CYBER_PLUGIN_MANAGER_REGISTER_PLUGIN(apollo::planning::PathFollowStage, Stage)

}  // namespace planning
}  // namespace apollo
