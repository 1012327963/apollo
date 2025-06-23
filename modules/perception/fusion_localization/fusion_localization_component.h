#pragma once

#include <memory>

#include "cyber/component/component.h"
#include "modules/common_msgs/localization_msgs/localization.pb.h"
#include "modules/common_msgs/localization_msgs/localization_status.pb.h"
#include "modules/perception/fusion_localization/proto/fusion_localization.pb.h"

namespace apollo {
namespace perception {
namespace localization {

class FusionLocalizationComponent final
    : public cyber::Component<apollo::localization::LocalizationEstimate> {
 public:
  bool Init() override;
  bool Proc(const std::shared_ptr<apollo::localization::LocalizationEstimate>& msg) override;

 private:
  std::shared_ptr<cyber::Reader<apollo::localization::LocalizationEstimate>> secondary_reader_;
  std::shared_ptr<cyber::Writer<apollo::localization::LocalizationEstimate>> writer_;
  std::shared_ptr<cyber::Writer<apollo::localization::LocalizationStatus>> status_writer_;

  FusionLocalizationConfig config_;
};

CYBER_REGISTER_COMPONENT(FusionLocalizationComponent);

}  // namespace localization
}  // namespace perception
}  // namespace apollo
