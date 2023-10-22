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
#ifndef OPTIMUS_PATH_PLANNING_DSTAR_LITE_SE2_PLANNER_H_
#define OPTIMUS_PATH_PLANNING_DSTAR_LITE_SE2_PLANNER_H_

#include <unordered_set>
#include <vector>

#include "optimus/path_planning/dstar_lite_planner.h"
#include "optimus/path_planning/se2_environment.h"
#include "optimus/path_planning/se2_planner.h"

namespace optimus {

using DStarLiteSE2Planner = SE2Planner<DStarLitePlanner<SE2Environment>>;

template <>
std::optional<float> DStarLiteSE2Planner::GetPathCost() const {
  if (!start_index_) {
    return std::nullopt;
  }
  return algorithm_.g_values().at(*start_index_);
}

template <>
PlannerStatus DStarLiteSE2Planner::ReplanPath(
    const Pose2D& start, const std::vector<Position2D>& changed_positions,
    const UserCallback& user_callback, std::vector<Pose2D>& path) {
  start_index_.reset();

  const auto& position_change_affected_states =
      env_.action_set().position_change_affected_states;
  std::unordered_set<int> states_to_update;
  for (const auto& changed_position : changed_positions) {
    const auto x = std::floor(changed_position.x());
    const auto y = std::floor(changed_position.y());
    for (const auto& affected_state : position_change_affected_states) {
      const auto updated_x = x + affected_state.x_idx;
      const auto updated_y = y + affected_state.y_idx;
      if (updated_x < 0 || updated_y < 0) {
        continue;
      }
      states_to_update.insert(
          GetStateIndex(updated_x, updated_y, affected_state.angle_idx));
    }
  }

  const auto start_index = GetStateIndex(start);
  std::vector<int> path_indices;  // TODO(mvukov) Make this a member variable?
  if (auto status = algorithm_.ReplanPath(start_index, states_to_update,
                                          user_callback, path_indices);
      status != PlannerStatus::kSuccess) {
    return status;
  }
  if (auto status = ReconstructPath(path_indices, path);
      status != PlannerStatus::kSuccess) {
    return status;
  }

  start_index_ = start_index;
  return PlannerStatus::kSuccess;
}

}  // namespace optimus

#endif  // OPTIMUS_PATH_PLANNING_DSTAR_LITE_SE2_PLANNER_H_
