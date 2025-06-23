#pragma once

#include <memory>
#include <string>

#include "cyber/plugin_manager/plugin_manager.h"
#include "modules/planning/planning_interface_base/scenario_base/scenario.h"

namespace apollo {
namespace planning {

class LargeCurvatureScenario : public Scenario {
 public:
  bool Init(std::shared_ptr<DependencyInjector> injector,
            const std::string& name) override;

  ScenarioContext* GetContext() override { return &context_; }

  bool IsTransferable(const Scenario* other_scenario,
                      const Frame& frame) override;

 private:
  bool init_ = false;
  ScenarioContext context_;
};

CYBER_PLUGIN_MANAGER_REGISTER_PLUGIN(apollo::planning::LargeCurvatureScenario,
                                     Scenario)

}  // namespace planning
}  // namespace apollo
