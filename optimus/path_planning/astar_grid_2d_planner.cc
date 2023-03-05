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
#include "optimus/path_planning/astar_grid_2d_planner.h"

namespace optimus {

PlannerStatus AStarGrid2DPlanner::PlanPath(
    const Position& start, const Position& goal,
    const Algorithm::UserCallback& user_callback, std::vector<Position>& path) {
  const auto start_index = GetStateIndex(start);
  const auto goal_index = GetStateIndex(goal);
  std::vector<int> path_indices;  // TODO(mvukov) Make this a member variable?
  if (auto status = algorithm_.PlanPath(start_index, goal_index, user_callback,
                                        path_indices);
      status != PlannerStatus::kSuccess) {
    return status;
  }
  ReconstructPath(path_indices, path);
  return PlannerStatus::kSuccess;
}

int AStarGrid2DPlanner::GetStateIndex(const Position& p) const {
  return p.y() * env_.obstacle_data().cols() + p.x();
}

void AStarGrid2DPlanner::ReconstructPath(const std::vector<int>& path_indices,
                                         std::vector<Position>& path) const {
  const int grid_width = env_.obstacle_data().cols();
  const int path_size = path_indices.size();
  path.resize(path_size);
  for (int el = 0; el < path_size; ++el) {
    const auto index = path_indices[el];
    path[el] = {index % grid_width, index / grid_width};
  }
}

}  // namespace optimus
