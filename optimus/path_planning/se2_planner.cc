// Copyright 2023 Milan Vukov. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
#include "optimus/path_planning/se2_planner.h"

#include <vector>

#include "optimus/check.h"

namespace optimus {
namespace {

constexpr int kMinPathLength = 2;

}  // namespace

bool SE2PlannerBase::SetGrid2D(const Grid2DMap* grid_2d) {
  return env_.SetGrid2D(grid_2d);
}

int SE2PlannerBase::GetStateIndex(const Pose2D& t) const {
  const int x = std::floor(t.x);
  const int y = std::floor(t.y);
  const auto angle_index = env_.action_set().GetAngleIndex(t.theta);
  if (angle_index == kInvalidIndex) {
    return kInvalidIndex;
  }
  return ((y * env_.grid_2d()->cols() + x) << env_.num_angle_bits()) +
         angle_index;
}

PlannerStatus SE2PlannerBase::ReconstructPath(
    const std::vector<int>& path_indices, std::vector<Pose2D>& path) const {
  if (static_cast<int>(path_indices.size()) < kMinPathLength) {
    return PlannerStatus::kInternalError;
  }

  int path_size = 0;
  int num_path_indices = path_indices.size();
  for (int el = 0; el < num_path_indices - 1; ++el) {
    const auto primitive_index =
        env_.GetPrimitiveIndex(path_indices[el], path_indices[el + 1]);
    const auto& primitive =
        env_.action_set().motion_primitives.at(primitive_index);

    const int primitive_length = primitive.x.size();
    const auto num_samples =
        el == num_path_indices - 2 ? primitive_length : primitive_length - 1;
    path_size += num_samples;
  }
  if (path_size < kMinPathLength) {
    return PlannerStatus::kInternalError;
  }
  path.resize(path_size);

  int path_sample = 0;
  for (int el = 0; el < num_path_indices - 1; ++el) {
    const auto current_start = path_indices[el];
    const auto current_goal = path_indices[el + 1];
    const auto primitive_index =
        env_.GetPrimitiveIndex(current_start, current_goal);
    const auto& primitive =
        env_.action_set().motion_primitives.at(primitive_index);

    const auto current_grid_xy = env_.ToGridCoords(current_start);
    const float current_grid_x = current_grid_xy.x;
    const float current_grid_y = current_grid_xy.y;
    const int primitive_length = primitive.x.size();
    const auto num_samples =
        el == num_path_indices - 2 ? primitive_length : primitive_length - 1;
    for (int ii = 0; ii < num_samples; ++ii, ++path_sample) {
      Pose2D& t = path[path_sample];
      t.x = current_grid_x + primitive.x[ii];
      t.y = current_grid_y + primitive.y[ii];
      t.theta = primitive.theta[ii];
    }
  }
  OPTIMUS_CHECK(path_sample == path_size);

  return PlannerStatus::kSuccess;
}

}  // namespace optimus
