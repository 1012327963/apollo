#include "modules/perception/fusion_localization/fusion_localization_component.h"

#include "cyber/common/log.h"
#include "modules/common/util/measure.h"

namespace apollo {
namespace perception {
namespace localization {

bool FusionLocalizationComponent::Init() {
  if (!GetProtoConfig(&config_)) {
    AERROR << "Load fusion localization config failed.";
    return false;
  }
  secondary_reader_ = node_->CreateReader<apollo::localization::LocalizationEstimate>(
      config_.secondary_localization_channel());
  writer_ = node_->CreateWriter<apollo::localization::LocalizationEstimate>(
      config_.output_localization_channel());
  status_writer_ = node_->CreateWriter<apollo::localization::LocalizationStatus>(
      config_.status_channel());
  return true;
}

bool FusionLocalizationComponent::Proc(
    const std::shared_ptr<apollo::localization::LocalizationEstimate>& msg) {
  secondary_reader_->Observe();
  auto secondary = secondary_reader_->GetLatestObserved();
  auto fused = *msg;
  apollo::localization::LocalizationStatus status;
  status.mutable_header()->CopyFrom(msg->header());
  status.set_measurement_time(msg->measurement_time());
  status.set_fusion_status(apollo::localization::MeasureState::OK);
  if (secondary != nullptr) {
    double dx = msg->pose().position().x() - secondary->pose().position().x();
    double dy = msg->pose().position().y() - secondary->pose().position().y();
    double dz = msg->pose().position().z() - secondary->pose().position().z();
    double dist = std::sqrt(dx * dx + dy * dy + dz * dz);
    if (dist > config_.safety_threshold()) {
      status.set_fusion_status(apollo::localization::MeasureState::WARNNING);
      status.set_state_message("Localization difference exceeds threshold");
    }
    // simple average fuse
    fused.mutable_pose()->mutable_position()->set_x(
        0.5 * (msg->pose().position().x() + secondary->pose().position().x()));
    fused.mutable_pose()->mutable_position()->set_y(
        0.5 * (msg->pose().position().y() + secondary->pose().position().y()));
    fused.mutable_pose()->mutable_position()->set_z(
        0.5 * (msg->pose().position().z() + secondary->pose().position().z()));
  }
  writer_->Write(fused);
  status_writer_->Write(status);
  return true;
}

}  // namespace localization
}  // namespace perception
}  // namespace apollo
