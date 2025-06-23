#include "modules/planning/scenarios/path_follow/path_follow_stage.h"

#include "cyber/common/log.h"
#include "modules/planning/planning_base/common/frame.h"

namespace apollo {
namespace planning {

using apollo::common::TrajectoryPoint;

StageResult PathFollowStage::Process(
    const TrajectoryPoint& planning_start_point, Frame* frame) {
  ADEBUG << "stage: PathFollow";
  CHECK_NOTNULL(frame);

  StageResult result = ExecuteTaskOnReferenceLine(planning_start_point, frame);
  if (result.HasError()) {
    AERROR << "PathFollowStage planning error";
    return result.SetStageStatus(StageStatusType::ERROR);
  }

  return result.SetStageStatus(StageStatusType::RUNNING);
}

}  // namespace planning
}  // namespace apollo
