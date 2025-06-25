#pragma once

#include "modules/planning/pnc_map/lane_follow_map/lane_follow_map.h"

namespace apollo {
namespace planning {

// Pnc map plugin used by the edge-follow scenario. It derives from
// `LaneFollowMap` and can be further extended to customize reference line
// generation along the road edge.
class EdgeFollowMap : public LaneFollowMap {
 public:
  EdgeFollowMap();
  ~EdgeFollowMap() override = default;

  bool GetRouteSegments(
      const common::VehicleState& vehicle_state,
      std::list<apollo::hdmap::RouteSegments>* const route_segments) override;
};

CYBER_PLUGIN_MANAGER_REGISTER_PLUGIN(apollo::planning::EdgeFollowMap, PncMapBase)

}  // namespace planning
}  // namespace apollo
