#pragma once

#include "modules/planning/planning_interface_base/scenario_base/stage.h"

namespace apollo {
namespace planning {

class LargeCurvatureStage : public Stage {
 public:
  StageResult Process(const common::TrajectoryPoint& planning_start_point,
                      Frame* frame) override;
};

}  // namespace planning
}  // namespace apollo
