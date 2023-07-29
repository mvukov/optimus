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
#ifndef OPTIMUS_PATH_PLANNING_SE2_PLANNER_H_
#define OPTIMUS_PATH_PLANNING_SE2_PLANNER_H_

#include <optional>
#include <vector>

#include "optimus/path_planning/planner_algorithm.h"
#include "optimus/path_planning/se2_environment.h"

namespace optimus {

class SE2PlannerBase {
 public:
  struct Pose2D {
    float x;
    float y;
    float theta;
  };

  SE2PlannerBase(const SE2Environment::Config& config,
                 const ActionSet2D* action_set)
      : env_(config, action_set) {}

  ~SE2PlannerBase() = default;

  [[nodiscard]] bool SetGrid2D(const Grid2DMap* grid_2d);

  virtual PlannerStatus PlanPath(const Pose2D& start, const Pose2D& goal,
                                 const UserCallback& user_callback,
                                 std::vector<Pose2D>& path) = 0;

  virtual std::optional<float> GetPathCost() const = 0;

 protected:
  int GetStateIndex(const Pose2D& t) const;
  PlannerStatus ReconstructPath(const std::vector<int>& path_indices,
                                std::vector<Pose2D>& path) const;

  SE2Environment env_;
  std::optional<int> start_index_;
  std::optional<int> goal_index_;
};

template <class Algorithm>
class SE2Planner final : public SE2PlannerBase {
 public:
  explicit SE2Planner(const SE2Environment::Config& config,
                      const ActionSet2D* action_set)
      : SE2PlannerBase(config, action_set), algorithm_(&env_) {}

  PlannerStatus PlanPath(const Pose2D& start, const Pose2D& goal,
                         const UserCallback& user_callback,
                         std::vector<Pose2D>& path) final;

  std::optional<float> GetPathCost() const final;

 private:
  Algorithm algorithm_;
};

template <class Algorithm>
PlannerStatus SE2Planner<Algorithm>::PlanPath(const Pose2D& start,
                                              const Pose2D& goal,
                                              const UserCallback& user_callback,
                                              std::vector<Pose2D>& path) {
  if (!env_.Validate()) {
    return PlannerStatus::kInternalError;
  }

  start_index_.reset();
  goal_index_.reset();
  const auto start_index = GetStateIndex(start);
  const auto goal_index = GetStateIndex(goal);
  std::vector<int> path_indices;
  if (auto status = algorithm_.PlanPath(start_index, goal_index, user_callback,
                                        path_indices);
      status != PlannerStatus::kSuccess) {
    return status;
  }

  start_index_ = start_index;
  goal_index_ = goal_index;
  return ReconstructPath(path_indices, path);
}

}  // namespace optimus

#endif  // OPTIMUS_PATH_PLANNING_SE2_PLANNER_H_
