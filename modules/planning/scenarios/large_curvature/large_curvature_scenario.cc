#include "modules/planning/scenarios/large_curvature/large_curvature_scenario.h"

#include "cyber/common/log.h"
#include "modules/common/configs/vehicle_config_helper.h"
#include "modules/planning/scenarios/large_curvature/large_curvature_stage.h"

namespace apollo {
namespace planning {

using apollo::common::VehicleConfigHelper;

bool LargeCurvatureScenario::Init(std::shared_ptr<DependencyInjector> injector,
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

bool LargeCurvatureScenario::IsTransferable(
    const Scenario* other_scenario, const Frame& frame) {
  if (!frame.local_view().planning_command->has_lane_follow_command()) {
    return false;
  }
  if (frame.reference_line_info().empty()) {
    return false;
  }

  const double min_turn_radius =
      VehicleConfigHelper::Instance()->GetConfig().vehicle_param().min_turn_radius();
  const double curvature_threshold = 1.0 / min_turn_radius;

  for (const auto& reference_line_info : frame.reference_line_info()) {
    const auto& ref_line = reference_line_info.reference_line();
    for (const auto& point : ref_line.reference_points()) {
      if (std::fabs(point.kappa()) > curvature_threshold) {
        return true;
      }
    }
  }

  return false;
}

}  // namespace planning
}  // namespace apollo
