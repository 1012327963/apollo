#include "modules/external_command/command_processor/command_processor_base/util/loop_distance_tool.h"

#include <memory>
#include <vector>

#include "modules/common_msgs/routing_msgs/routing.pb.h"

namespace apollo {
namespace external_command {

LoopDistanceTool::LoopDistanceTool(
    const std::shared_ptr<apollo::routing::Routing>& routing)
    : routing_(routing) {}

double LoopDistanceTool::CalculateTotalDistance(
    const apollo::external_command::Pose& start_position,
    const std::vector<apollo::external_command::Pose>& waypoints,
    int loops) const {
  if (loops <= 0 || routing_ == nullptr) {
    return 0.0;
  }

  auto routing_request = std::make_shared<apollo::routing::RoutingRequest>();

  auto* start_waypoint = routing_request->add_waypoint();
  apollo::routing::LaneWaypoint start_lane_waypoint;
  auto* start_point_enu = start_lane_waypoint.mutable_pose();
  start_point_enu->set_x(start_position.x());
  start_point_enu->set_y(start_position.y());
  if (start_position.has_heading()) {
    start_lane_waypoint.set_heading(start_position.heading());
  }
  start_waypoint->CopyFrom(start_lane_waypoint);

  for (int i = 0; i < loops; ++i) {
    for (const auto& pose : waypoints) {
      auto* waypoint = routing_request->add_waypoint();
      apollo::routing::LaneWaypoint lane_waypoint;
      auto* point_enu = lane_waypoint.mutable_pose();
      point_enu->set_x(pose.x());
      point_enu->set_y(pose.y());
      if (pose.has_heading()) {
        lane_waypoint.set_heading(pose.heading());
      }
      waypoint->CopyFrom(lane_waypoint);
    }
  }

  auto routing_response = std::make_shared<apollo::routing::RoutingResponse>();
  if (!routing_->Process(routing_request, routing_response.get())) {
    return -1.0;
  }

  return routing_response->measurement().distance();
}

}  // namespace external_command
}  // namespace apollo
