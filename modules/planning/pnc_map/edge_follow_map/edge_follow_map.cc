#include "modules/planning/pnc_map/edge_follow_map/edge_follow_map.h"

#include "cyber/common/log.h"

namespace apollo {
namespace planning {

EdgeFollowMap::EdgeFollowMap() : LaneFollowMap() {}

bool EdgeFollowMap::GetRouteSegments(
    const common::VehicleState& vehicle_state,
    std::list<apollo::hdmap::RouteSegments>* const route_segments) {
  if (!LaneFollowMap::GetRouteSegments(vehicle_state, route_segments)) {
    return false;
  }

  for (auto& segments : *route_segments) {
    for (auto& segment : segments) {
      segment.start_s = std::max(0.0, segment.start_s);
      segment.end_s =
          std::min(segment.lane->total_length(), segment.end_s);
    }
  }
  AINFO << "EdgeFollowMap generated " << route_segments->size() << " segment group(s)";
  return true;
}

}  // namespace planning
}  // namespace apollo
