// Copyright 2024 Milan Vukov. All rights reserved.
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
#ifndef OPTIMUS_PATH_PLANNING_ARASTAR_GRID_2D_PLANNER_H_
#define OPTIMUS_PATH_PLANNING_ARASTAR_GRID_2D_PLANNER_H_
#include <vector>

#include "optimus/path_planning/arastar_planner.h"
#include "optimus/path_planning/grid_2d_environment.h"
#include "optimus/path_planning/grid_2d_planner.h"

namespace optimus {

using AraStarGrid2DPlanner = Grid2DPlanner<AraStarPlanner<Grid2DEnvironment>>;

template <>
std::optional<float> AraStarGrid2DPlanner::GetPathCost() const {
  if (!goal_index_) {
    return std::nullopt;
  }
  return algorithm_.GetGValue(*goal_index_);
}

template <>
PlannerStatus AraStarGrid2DPlanner::ReplanPath(
    const Position2D&, const std::vector<Position2D>&,
    const UserCallback& user_callback, std::vector<Position2D>& path) {
  if (!start_index_) {
    return PlannerStatus::kInternalError;
  }
  std::vector<int> path_indices;  // TODO(mvukov) Make this a member variable?
  if (auto status =
          algorithm_.ReplanPath(*start_index_, {}, user_callback, path_indices);
      status != PlannerStatus::kSuccess) {
    return status;
  }
  ReconstructPath(path_indices, path);
  return PlannerStatus::kSuccess;
}

}  // namespace optimus

#endif  // OPTIMUS_PATH_PLANNING_ARASTAR_GRID_2D_PLANNER_H_
