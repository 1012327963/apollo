#include "modules/data/tools/smart_recorder/stop_manual_trigger.h"

#include "cyber/common/log.h"
#include "modules/common/adapters/adapter_gflags.h"

namespace apollo {
namespace data {

using apollo::canbus::Chassis;

StopManualTrigger::StopManualTrigger() { trigger_name_ = "StopManualTrigger"; }

void StopManualTrigger::Pull(const cyber::record::RecordMessage& msg) {
  if (!trigger_obj_ || !trigger_obj_->enabled()) {
    return;
  }
  if (msg.channel_name == FLAGS_chassis_topic) {
    Chassis chassis_msg;
    chassis_msg.ParseFromString(msg.content);
    // Manual driving switch
    if (last_driving_mode_ != chassis_msg.driving_mode() &&
        chassis_msg.driving_mode() == Chassis::COMPLETE_MANUAL) {
      AINFO << "manual mode trigger is pulled: " << msg.time;
      TriggerIt(msg.time);
    }
    last_driving_mode_ = chassis_msg.driving_mode();

    // Check stop duration
    const float speed = chassis_msg.speed_mps();
    if (fabs(speed) < 0.1) {
      if (stop_start_time_ == 0) {
        stop_start_time_ = msg.time;
        stop_triggered_ = false;
      } else if (!stop_triggered_ &&
                 msg.time - stop_start_time_ >= SecondsToNanoSeconds(5.0)) {
        AINFO << "stop 5s trigger is pulled: " << msg.time;
        TriggerIt(msg.time);
        stop_triggered_ = true;
      }
    } else {
      stop_start_time_ = 0;
    }
  }
}

}  // namespace data
}  // namespace apollo
