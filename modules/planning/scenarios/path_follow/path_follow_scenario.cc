#include "modules/planning/scenarios/path_follow/path_follow_scenario.h"

#include "cyber/common/log.h"
#include "modules/planning/scenarios/path_follow/path_follow_stage.h"

namespace apollo {
namespace planning {

bool PathFollowScenario::Init(std::shared_ptr<DependencyInjector> injector,
                              const std::string& name) {
  if (init_) {
    return true;
  }

  if (!Scenario::Init(injector, name)) {
    AERROR << "failed to init scenario" << Name();
    return false;
  }

  init_ = true;
  return true;
}

bool PathFollowScenario::IsTransferable(const Scenario* other_scenario,
                                        const Frame& frame) {
  if (!frame.local_view().planning_command->has_lane_follow_command()) {
    return false;
  }
  if (frame.reference_line_info().empty()) {
    return false;
  }
  return true;
}

}  // namespace planning
}  // namespace apollo
