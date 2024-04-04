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
#include <memory>

#include "gmock/gmock.h"
#include "gtest/gtest.h"

#include "optimus/path_planning/arastar_planner.h"
#include "optimus/path_planning/grid_2d_environment.h"

namespace optimus {

using ::testing::_;
using ::testing::Eq;
using ::testing::Test;

class TestAraStarPlannerOn2DGrid : public Test {
 public:
  using Planner = AraStarPlanner<Grid2DEnvironment>;

  TestAraStarPlannerOn2DGrid() {
    grid_2d_ = Grid2D(5, 3);
    grid_2d_.setConstant(0);
    grid_2d_map_ = std::make_unique<Grid2DMap>(grid_2d_.data(), grid_2d_.rows(),
                                               grid_2d_.cols());

    Grid2DEnvironment::Config env_config;

    env_config.valid_state_threshold = 1;
    env_ = std::make_unique<Grid2DEnvironment>(env_config);
    EXPECT_TRUE(env_->SetGrid2D(grid_2d_map_.get()));

    planner_ = std::make_unique<Planner>(planner_config_, env_.get());
  }

  void SetObstacles(const std::vector<int>& indices) {
    for (auto index : indices) {
      *(grid_2d_.data() + index) = 2;
    }
  }

  AraStarPlannerConfig planner_config_;
  Grid2D grid_2d_;
  std::unique_ptr<Grid2DMap> grid_2d_map_;
  std::unique_ptr<Grid2DEnvironment> env_;
  std::unique_ptr<Planner> planner_;
  std::vector<int> path_;
  std::vector<int> expected_path_;
};

TEST_F(TestAraStarPlannerOn2DGrid,
       GivenFeasibleProblem_WhenPlanPath_EnsurePlanningSucceeds) {
  SetObstacles({4, 7});

  EXPECT_THAT(planner_->PlanPath(3, 14, {}, path_),
              Eq(PlannerStatus::kSuccess));
  expected_path_ = {3, 6, 10, 14};
  EXPECT_THAT(path_, Eq(expected_path_));
}

TEST_F(TestAraStarPlannerOn2DGrid,
       GivenInfeasibleProblem_WhenPlanPath_EnsurePlanningFails) {
  SetObstacles({1, 4, 7, 10, 13});

  EXPECT_THAT(planner_->PlanPath(3, 14, {}, path_),
              Eq(PlannerStatus::kInfeasibleProblem));
}

}  // namespace optimus
