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
#ifndef OPTIMUS_PATH_PLANNING_GRID_2D_PLANNER_H_
#define OPTIMUS_PATH_PLANNING_GRID_2D_PLANNER_H_

#include <vector>

#include "optimus/path_planning/grid_2d_environment.h"
#include "optimus/path_planning/planner_algorithm.h"

namespace optimus {

class Grid2DPlannerBase {
 public:
  using Position = Eigen::Vector2i;

  explicit Grid2DPlannerBase(const Grid2DEnvironment::Config& config)
      : env_(config) {}

  virtual ~Grid2DPlannerBase() = default;

  [[nodiscard]] bool SetObstacleData(
      const Grid2DEnvironment::ObstacleData* obstacle_data);

  [[nodiscard]] virtual PlannerStatus PlanPath(
      const Position& start, const Position& goal,
      const UserCallback& user_callback, std::vector<Position>& path) = 0;

 protected:
  int GetStateIndex(const Position& t) const;
  void ReconstructPath(const std::vector<int>& path_indices,
                       std::vector<Position>& path) const;

  Grid2DEnvironment env_;
};

template <class Algorithm>
class Grid2DPlanner final : public Grid2DPlannerBase {
 public:
  explicit Grid2DPlanner(const Grid2DEnvironment::Config& config)
      : Grid2DPlannerBase(config), algorithm_(&env_) {}

  [[nodiscard]] PlannerStatus PlanPath(const Position& start,
                                       const Position& goal,
                                       const UserCallback& user_callback,
                                       std::vector<Position>& path) override;

 private:
  Algorithm algorithm_;
};

template <class A>
PlannerStatus Grid2DPlanner<A>::PlanPath(const Position& start,
                                         const Position& goal,
                                         const UserCallback& user_callback,
                                         std::vector<Position>& path) {
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

}  // namespace optimus

#endif  // OPTIMUS_PATH_PLANNING_GRID_2D_PLANNER_H_
