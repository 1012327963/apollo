/******************************************************************************
 * Copyright 2018 The Apollo Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/

/**
 * @file ego_info.h
 **/

#pragma once

#include <vector>

#include "modules/common/vehicle_state/proto/vehicle_state.pb.h"
#include "modules/common_msgs/config_msgs/vehicle_config.pb.h"
#include "modules/common_msgs/routing_msgs/geometry.pb.h"

#include "cyber/common/macros.h"
#include "modules/planning/planning_base/common/obstacle.h"
#include "modules/planning/planning_base/gflags/planning_gflags.h"
#include "modules/planning/planning_base/reference_line/reference_line.h"
#include "modules/planning/planning_base/reference_line/reference_line_provider.h"

namespace apollo {
namespace external_command {

class EgoInfo {
 public:
  EgoInfo();

  ~EgoInfo() = default;

  bool Update(const common::TrajectoryPoint& start_point,
              const common::VehicleState& vehicle_state);

  void Clear();

  common::TrajectoryPoint start_point() const { return start_point_; }

  common::VehicleState vehicle_state() const { return vehicle_state_; }

  double front_clear_distance() const { return front_clear_distance_; }

  common::math::Box2d ego_box() const { return ego_box_; }

  void CalculateFrontObstacleClearDistance(
      const std::vector<const Obstacle*>& obstacles);

  void CalculateCurrentRouteInfo(
      const ReferenceLineProvider* reference_line_provider);

  // Retrieve the remaining distance to the routing destination after calling
  // `CalculateCurrentRouteInfo`. This value is provided by the reference line
  // provider and represents how far the vehicle still needs to travel along
  // the planned route.
  double GetDistanceToDestination() const { return distance_to_destination_; }

  // Configure loop information so that EgoInfo can report the remaining
  // distance for a task consisting of multiple loops. |loop_distance| is the
  // length of a single loop and |loop_count| is the total number of loops to
  // drive.
  void SetLoopInfo(double loop_distance, int loop_count) {
    single_loop_distance_ = loop_distance;
    remaining_loop_count_ = loop_count;
  }

  // Notify EgoInfo that one loop has completed so the remaining distance can
  // be updated accordingly.
  void OnLoopComplete() {
    if (remaining_loop_count_ > 0) {
      --remaining_loop_count_;
    }
  }

  // Return the remaining distance for the whole looping task. This combines
  // the current routing distance with the distance for future loops that have
  // not started yet.
  double GetRemainingLoopDistance() const {
    if (remaining_loop_count_ <= 0) {
      return distance_to_destination_;
    }
    return distance_to_destination_ +
           static_cast<double>(remaining_loop_count_ - 1) *
               single_loop_distance_;
  }

 private:
  FRIEND_TEST(EgoInfoTest, EgoInfoSimpleTest);

  void set_vehicle_state(const common::VehicleState& vehicle_state) {
    vehicle_state_ = vehicle_state;
  }

  void set_start_point(const common::TrajectoryPoint& start_point) {
    start_point_ = start_point;
    const auto& param = ego_vehicle_config_.vehicle_param();
    start_point_.set_a(
        std::fmax(std::fmin(start_point_.a(), param.max_acceleration()),
                  param.max_deceleration()));
  }

  void CalculateEgoBox(const common::VehicleState& vehicle_state);

  // stitched point (at stitching mode)
  // or real vehicle point (at non-stitching mode)
  common::TrajectoryPoint start_point_;

  // ego vehicle state
  common::VehicleState vehicle_state_;

  double front_clear_distance_ = FLAGS_default_front_clear_distance;

  common::VehicleConfig ego_vehicle_config_;

  common::math::Box2d ego_box_;

  apollo::hdmap::LaneWaypoint adc_waypoint_;
  double distance_to_destination_ = 0;

  // Length of a single loop when executing a multi-loop task.
  double single_loop_distance_ = 0.0;

  // Remaining loop count excluding the current loop.
  int remaining_loop_count_ = 0;
};

}  // namespace external_command
}  // namespace apollo
