#pragma once

#include <memory>
#include <vector>

#include "modules/common_msgs/external_command_msgs/geometry.pb.h"

#include "modules/routing/routing.h"

namespace apollo {
namespace external_command {

class LoopDistanceTool {
 public:
  explicit LoopDistanceTool(
      const std::shared_ptr<apollo::routing::Routing>& routing);

  // Calculate the distance of repeatedly driving through the given waypoints.
  // The first waypoint segment starts from |start_position| and the
  // |waypoints| sequence is appended |loops| times in the routing request.
  // Returns the total distance measured by the routing module or -1.0 if the
  // routing process fails.
  double CalculateTotalDistance(
      const apollo::external_command::Pose& start_position,
      const std::vector<apollo::external_command::Pose>& waypoints,
      int loops) const;

 private:
  std::shared_ptr<apollo::routing::Routing> routing_;
};

}  // namespace external_command
}  // namespace apollo
