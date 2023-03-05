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

namespace optimus {

class AStarGrid2DPlanner {
 public:
  using Algorithm = AStarPlanner<Grid2DEnvironment>;

  using Position = Eigen::Vector2i;

  explicit AStarGrid2DPlanner(const Grid2DEnvironment::Config& config)
      : env_(config), algorithm_(&env_) {}

  [[nodiscard]] bool SetObstacleData(
      const Grid2DEnvironment::ObstacleData* obstacle_data) {
    return env_.SetObstacleData(obstacle_data);
  }

  [[nodiscard]] PlannerStatus PlanPath(
      const Position& start, const Position& goal,
      const Algorithm::UserCallback& user_callback,
      std::vector<Position>& path);

  const auto& env() const { return env_; }

 private:
  int GetStateIndex(const Position& t) const;
  void ReconstructPath(const std::vector<int>& path_indices,
                       std::vector<Position>& path) const;

  Grid2DEnvironment env_;
  Algorithm algorithm_;
};

}  // namespace optimus

#endif  // OPTIMUS_PATH_PLANNING_ASTAR_GRID_2D_PLANNER_H_
