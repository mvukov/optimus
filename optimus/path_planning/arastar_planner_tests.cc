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

using ::testing::Eq;
using ::testing::Test;

class TestAraStarPlannerConfig : public Test {
 public:
  AraStarPlannerConfig config_;
};

TEST_F(TestAraStarPlannerConfig,
       GivenDefaultConfig_WhenValidate_EnsureSuccess) {
  EXPECT_TRUE(config_.Validate());
}

TEST_F(TestAraStarPlannerConfig,
       GivenValidConfig_WhenEpsilonStartLessThanOne_EnsureValidateFails) {
  config_.epsilon_start = 0.99;
  EXPECT_FALSE(config_.Validate());
}

TEST_F(
    TestAraStarPlannerConfig,
    GivenValidConfig_WhenEpsilonDecreaseRateLessThanZero_EnsureValidateFails) {
  config_.epsilon_decrease_rate = -0.1;
  EXPECT_FALSE(config_.Validate());
}

class TestAraStarPlannerOn2DGrid : public Test {
 public:
  using Planner = AraStarPlanner<Grid2DEnvironment>;

  TestAraStarPlannerOn2DGrid() {
    grid_2d_ = Grid2D(7, 6);
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

  void SetObstaclesAsInPaper() {
    // Sets up an env such as on Fig. 4 in the paper
    // [1] "ARA*: Formal Analysis"
    // http://www.cs.cmu.edu/~ggordon/mlikhach-ggordon-thrun.ara-tr.pdf
    // clang-format off
    SetObstacles({6, 8, 9, 10,
                  12, 14, 15, 16,
                  20,
                  25, 26, 28,
                  34,
                  36, 37, 38, 39, 40});
    // clang-format on
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
  SetObstaclesAsInPaper();

  EXPECT_THAT(planner_->PlanPath(0, 41, {}, path_),
              Eq(PlannerStatus::kSuccess));
  expected_path_ = {0, 1, 2, 3, 4, 11, 17, 23, 29, 35, 41};
  EXPECT_THAT(path_, Eq(expected_path_));
  EXPECT_THAT(planner_->epsilon(), Eq(1.));
}

TEST_F(TestAraStarPlannerOn2DGrid,
       GivenInfeasibleProblem_WhenPlanPath_EnsurePlanningFails) {
  SetObstacles({1, 7, 13, 19, 25, 31, 37});

  EXPECT_THAT(planner_->PlanPath(0, 41, {}, path_),
              Eq(PlannerStatus::kInfeasibleProblem));
}

TEST_F(TestAraStarPlannerOn2DGrid,
       GivenFeasibleProblem_WhenPlanPathStopped_EnsureCorrectPath) {
  SetObstaclesAsInPaper();

  auto callback = [](UserCallbackEvent event) {
    return event != UserCallbackEvent::kSolutionFound;
  };
  EXPECT_THAT(planner_->PlanPath(0, 41, callback, path_),
              Eq(PlannerStatus::kSuccess));
  // The expected path is the same as in [1] Fig. 4 on the left.
  expected_path_ = {0, 7, 13, 19, 24, 31, 32, 27, 22, 29, 35, 41};
  EXPECT_THAT(path_, Eq(expected_path_));
  EXPECT_THAT(planner_->epsilon(), Eq(planner_config_.epsilon_start));
}

}  // namespace optimus
