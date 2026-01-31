#pragma once

#include "modules/common_msgs/chassis_msgs/chassis.pb.h"
#include "modules/data/tools/smart_recorder/proto/smart_recorder_triggers.pb.h"
#include "modules/data/tools/smart_recorder/trigger_base.h"

namespace apollo {
namespace data {

// Trigger when vehicle stops for a while or switches to manual driving
class StopManualTrigger : public TriggerBase {
 public:
  StopManualTrigger();

  void Pull(const cyber::record::RecordMessage& msg) override;
  bool ShouldRestore(const cyber::record::RecordMessage& msg) const override { return false; };

  virtual ~StopManualTrigger() = default;

 private:
  apollo::canbus::Chassis::DrivingMode last_driving_mode_ =
      apollo::canbus::Chassis::COMPLETE_MANUAL;
  uint64_t stop_start_time_ = 0UL;
  bool stop_triggered_ = false;
};

}  // namespace data
}  // namespace apollo
