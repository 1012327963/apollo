/******************************************************************************
 * Copyright 2023 The Apollo Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/

#include "modules/planning/tasks/edge_follow_path/edge_follow_path.h"

#include <algorithm>

#include "modules/common/util/point_factory.h"
#include "modules/planning/planning_base/common/frame.h"
#include "modules/planning/planning_base/common/reference_line_info.h"

namespace apollo {
namespace planning {

using apollo::common::Status;
using apollo::common::util::PointFactory;

bool EdgeFollowPath::Init(const std::string& config_dir, const std::string& name,
                          const std::shared_ptr<DependencyInjector>& injector) {
  if (!Task::Init(config_dir, name, injector)) {
    return false;
  }
  return Task::LoadConfig<EdgeFollowPathConfig>(&config_);
}

Status EdgeFollowPath::Process(Frame* frame,
                               ReferenceLineInfo* reference_line_info) {
  std::vector<common::PathPoint> path_points;
  const auto& reference_line = reference_line_info->reference_line();
  double start_s = reference_line_info->AdcSlBoundary().start_s();
  double end_s = start_s + config_.forward_length();

  const Obstacle* target_obstacle = nullptr;
  double avoid_start_s = 0.0;
  double avoid_end_s = 0.0;
  const auto* path_decision = reference_line_info->path_decision();
  if (path_decision != nullptr) {
    for (const auto* obstacle : path_decision->obstacles().Items()) {
      if (obstacle->IsVirtual()) {
        continue;
      }
      const auto& sl = obstacle->PerceptionSLBoundary();
      if (sl.start_s() > end_s || sl.end_s() < start_s) {
        continue;
      }
      double left_width = 0.0;
      double right_width = 0.0;
      if (!reference_line.GetLaneWidth(sl.start_s(), &left_width, &right_width)) {
        continue;
      }
      double offset = 0.0;
      reference_line.GetOffsetToMap(sl.start_s(), &offset);
      double right_edge = -right_width - offset;
      double obs_l = 0.5 * (sl.start_l() + sl.end_l());
      if (std::fabs(obs_l - right_edge) > 1.0) {
        continue;
      }
      if (target_obstacle == nullptr || sl.start_s() < avoid_start_s) {
        target_obstacle = obstacle;
        avoid_start_s = sl.start_s();
        avoid_end_s = sl.end_s();
      }
    }
  }
  for (double s = start_s; s <= end_s; s += config_.path_resolution()) {
    double left_width = 0.0;
    double right_width = 0.0;
    if (!reference_line.GetLaneWidth(s, &left_width, &right_width)) {
      break;
    }
    double offset_to_center = 0.0;
    reference_line.GetOffsetToMap(s, &offset_to_center);
    double right_bound = -right_width - offset_to_center;
    double lane_center_l = -offset_to_center;
    double l = right_bound + config_.edge_buffer();

    if (target_obstacle) {
      double return_end_s = avoid_end_s + 10.0;
      if (s >= avoid_start_s - 1.0 && s <= return_end_s) {
        double ratio = 0.0;
        if (s <= avoid_end_s) {
          ratio = std::clamp((s - (avoid_start_s - 1.0)) /
                                  (avoid_end_s - (avoid_start_s - 1.0)),
                              0.0, 1.0);
        } else {
          ratio = std::clamp(1.0 - (s - avoid_end_s) / 10.0, 0.0, 1.0);
        }
        l = ratio * lane_center_l + (1.0 - ratio) * (right_bound + config_.edge_buffer());
      }
    }

    common::SLPoint sl;
    sl.set_s(s);
    sl.set_l(l);
    common::math::Vec2d xy;
    reference_line.SLToXY(sl, &xy);
    auto ref_pt = reference_line.GetReferencePoint(s);
    common::PathPoint path_point =
        PointFactory::ToPathPoint(xy.x(), xy.y(), 0.0, ref_pt.heading(), s - start_s);
    path_point.set_kappa(ref_pt.kappa());
    path_points.push_back(path_point);
  }
  if (path_points.empty()) {
    return Status::OK();
  }
  PathData path_data;
  path_data.SetReferenceLine(&reference_line);
  path_data.SetDiscretizedPath(DiscretizedPath(path_points));
  path_data.set_path_label("edge_follow");
  *reference_line_info->mutable_path_data() = path_data;
  reference_line_info->MutableCandidatePathData()->push_back(path_data);
  return Status::OK();
}

}  // namespace planning
}  // namespace apollo
