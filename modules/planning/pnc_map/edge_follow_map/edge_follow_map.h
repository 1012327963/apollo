#pragma once

#include "modules/planning/pnc_map/lane_follow_map/lane_follow_map.h"

namespace apollo {
namespace planning {

// Simple alias of LaneFollowMap for edge-follow scenario.
class EdgeFollowMap : public LaneFollowMap {
 public:
  EdgeFollowMap() = default;
  ~EdgeFollowMap() override = default;
};

CYBER_PLUGIN_MANAGER_REGISTER_PLUGIN(apollo::planning::EdgeFollowMap, PncMapBase)

}  // namespace planning
}  // namespace apollo
