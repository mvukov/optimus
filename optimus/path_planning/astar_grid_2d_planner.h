// Copyright 2022 Milan Vukov. All rights reserved.
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
#ifndef OPTIMUS_PATH_PLANNING_ASTAR_GRID_2D_PLANNER_H_
#define OPTIMUS_PATH_PLANNING_ASTAR_GRID_2D_PLANNER_H_
#include <vector>

#include "optimus/path_planning/astar_planner.h"
#include "optimus/path_planning/grid_2d_environment.h"
#include "optimus/path_planning/grid_2d_planner.h"

namespace optimus {

using AStarGrid2DPlanner = Grid2DPlanner<AStarPlanner<Grid2DEnvironment>>;

template <>
std::optional<float> AStarGrid2DPlanner::GetPathCost() const {
  if (!goal_index_) {
    return std::nullopt;
  }
  return algorithm_.GetGValue(*goal_index_);
}

template <>
PlannerStatus AStarGrid2DPlanner::ReplanPath(const Position2D&,
                                             const std::vector<Position2D>&,
                                             const UserCallback&,
                                             std::vector<Position2D>&) {
  return PlannerStatus::kNotImplemented;
}

}  // namespace optimus

#endif  // OPTIMUS_PATH_PLANNING_ASTAR_GRID_2D_PLANNER_H_
