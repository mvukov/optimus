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
#ifndef OPTIMUS_PATH_PLANNING_DSTAR_LITE_GRID_2D_PLANNER_H_
#define OPTIMUS_PATH_PLANNING_DSTAR_LITE_GRID_2D_PLANNER_H_

#include <unordered_set>
#include <vector>

#include "optimus/path_planning/dstar_lite_planner.h"
#include "optimus/path_planning/grid_2d_environment.h"
#include "optimus/path_planning/grid_2d_planner.h"

namespace optimus {

using DStarLiteGrid2DPlanner =
    Grid2DPlanner<DStarLitePlanner<Grid2DEnvironment>>;

template <>
std::optional<float> DStarLiteGrid2DPlanner::GetPathCost() const {
  if (!start_index_) {
    return std::nullopt;
  }
  return algorithm_.g_values().at(*start_index_);
}

template <>
PlannerStatus DStarLiteGrid2DPlanner::ReplanPath(
    const Position2D& start, const std::vector<Position2D>& changed_positions,
    const UserCallback& user_callback, std::vector<Position2D>& path) {
  start_index_.reset();
  const auto start_index = GetStateIndex(start);

  std::unordered_set<int> states_to_update;
  std::vector<int> predecessors(env_.GetMaxNumPredecessors(), kInvalidIndex);
  for (const auto& changed_position : changed_positions) {
    const auto changed_state = GetStateIndex(changed_position);
    if (changed_state == kInvalidIndex) {
      continue;
    }
    env_.GetPredecessors(changed_state, predecessors);
    states_to_update.insert(predecessors.begin(), predecessors.end());
  }

  std::vector<int> path_indices;  // TODO(mvukov) Make this a member variable?
  if (auto status = algorithm_.ReplanPath(start_index, states_to_update,
                                          user_callback, path_indices);
      status != PlannerStatus::kSuccess) {
    return status;
  }
  ReconstructPath(path_indices, path);
  start_index_ = start_index;
  return PlannerStatus::kSuccess;
}

}  // namespace optimus

#endif  // OPTIMUS_PATH_PLANNING_DSTAR_LITE_GRID_2D_PLANNER_H_
