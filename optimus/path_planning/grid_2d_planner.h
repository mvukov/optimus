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

#include <optional>
#include <utility>
#include <vector>

#include "optimus/path_planning/grid_2d.h"
#include "optimus/path_planning/grid_2d_environment.h"
#include "optimus/path_planning/planner_algorithm.h"

namespace optimus {

class Grid2DPlannerBase {
 public:
  explicit Grid2DPlannerBase(const Grid2DEnvironment::Config& config)
      : env_(config) {}

  virtual ~Grid2DPlannerBase() = default;

  [[nodiscard]] bool SetGrid2D(const Grid2DMap* grid_2d);

  virtual PlannerStatus PlanPath(const Position2D& start,
                                 const Position2D& goal,
                                 const UserCallback& user_callback,
                                 std::vector<Position2D>& path) = 0;

  virtual PlannerStatus ReplanPath(
      const Position2D& start, const std::vector<Position2D>& changed_positions,
      const UserCallback& user_callback, std::vector<Position2D>& path) = 0;

  virtual std::optional<float> GetPathCost() const = 0;

 protected:
  int GetStateIndex(const Position2D& t) const;
  void ReconstructPath(const std::vector<int>& path_indices,
                       std::vector<Position2D>& path) const;

  Grid2DEnvironment env_;
  std::optional<int> start_index_;
  std::optional<int> goal_index_;
};

template <class Algorithm>
class Grid2DPlanner final : public Grid2DPlannerBase {
 public:
  template <typename... Args>
  explicit Grid2DPlanner(const Grid2DEnvironment::Config& config,
                         Args&&... args)
      : Grid2DPlannerBase(config),
        algorithm_(&env_, std::forward<Args>(args)...) {}

  PlannerStatus PlanPath(const Position2D& start, const Position2D& goal,
                         const UserCallback& user_callback,
                         std::vector<Position2D>& path) final;

  PlannerStatus ReplanPath(const Position2D& start,
                           const std::vector<Position2D>& changed_states,
                           const UserCallback& user_callback,
                           std::vector<Position2D>& path) final;

  std::optional<float> GetPathCost() const final;

 private:
  Algorithm algorithm_;
};

template <class Algorithm>
PlannerStatus Grid2DPlanner<Algorithm>::PlanPath(
    const Position2D& start, const Position2D& goal,
    const UserCallback& user_callback, std::vector<Position2D>& path) {
  start_index_.reset();
  goal_index_.reset();
  const auto start_index = GetStateIndex(start);
  const auto goal_index = GetStateIndex(goal);
  std::vector<int> path_indices;  // TODO(mvukov) Make this a member variable?
  if (auto status = algorithm_.PlanPath(start_index, goal_index, user_callback,
                                        path_indices);
      status != PlannerStatus::kSuccess) {
    return status;
  }
  ReconstructPath(path_indices, path);
  start_index_ = start_index;
  goal_index_ = goal_index;
  return PlannerStatus::kSuccess;
}

}  // namespace optimus

#endif  // OPTIMUS_PATH_PLANNING_GRID_2D_PLANNER_H_
