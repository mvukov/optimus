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
#ifndef OPTIMUS_PATH_PLANNING_ASTAR_SE2_PLANNER_H_
#define OPTIMUS_PATH_PLANNING_ASTAR_SE2_PLANNER_H_

#include <vector>

#include "optimus/path_planning/astar_planner.h"
#include "optimus/path_planning/se2_environment.h"

namespace optimus {

class AStarSE2Planner {
 public:
  using Algorithm = AStarPlanner<SE2Environment>;

  struct Pose2D {
    float x;
    float y;
    float theta;
  };

  AStarSE2Planner(const SE2Environment::Config& config,
                  const ActionSet2D* action_set)
      : env_(config, action_set), algorithm_(&env_) {}

  [[nodiscard]] bool SetObstacleData(
      const SE2Environment::ObstacleData* obstacle_data) {
    return env_.SetObstacleData(obstacle_data);
  }

  [[nodiscard]] PlannerStatus PlanPath(
      const Pose2D& start, const Pose2D& goal,
      const Algorithm::UserCallback& user_callback, std::vector<Pose2D>& path);

  const auto& env() const { return env_; }

 private:
  int GetStateIndex(const Pose2D& t) const;
  PlannerStatus ReconstructPath(const std::vector<int>& path_indices,
                                std::vector<Pose2D>& path) const;

  SE2Environment env_;
  Algorithm algorithm_;
};

}  // namespace optimus

#endif  // OPTIMUS_PATH_PLANNING_ASTAR_SE2_PLANNER_H_
