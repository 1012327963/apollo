#include "modules/planning/scenarios/large_curvature/large_curvature_stage.h"

#include "cyber/common/log.h"
#include "modules/planning/planning_base/common/frame.h"

namespace apollo {
namespace planning {

using apollo::common::TrajectoryPoint;

StageResult LargeCurvatureStage::Process(
    const TrajectoryPoint& planning_start_point, Frame* frame) {
  ADEBUG << "stage: LargeCurvature";
  CHECK_NOTNULL(frame);

  frame->mutable_open_space_info()->set_is_on_open_space_trajectory(true);
  StageResult result = ExecuteTaskOnOpenSpace(frame);
  if (result.HasError()) {
    AERROR << "LargeCurvatureStage planning error";
    return result.SetStageStatus(StageStatusType::ERROR);
  }

  return result.SetStageStatus(StageStatusType::RUNNING);
}

}  // namespace planning
}  // namespace apollo
