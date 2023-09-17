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
#include <memory>

#include "gmock/gmock.h"
#include "gtest/gtest.h"

#include "optimus/path_planning/dstar_lite_planner.h"
#include "optimus/path_planning/grid_2d_environment.h"
#include "optimus/path_planning/planner_environment_mocks.h"

namespace optimus {

using ::testing::_;
using ::testing::Eq;
using ::testing::InSequence;
using ::testing::Return;
using ::testing::Test;

class TestDStarLitePlannerOn2DGrid : public Test {
 public:
  using Planner = DStarLitePlanner<Grid2DEnvironment>;

  TestDStarLitePlannerOn2DGrid() {
    grid_2d_ = Grid2D(5, 3);
    grid_2d_.setConstant(0);
    grid_2d_map_ = std::make_unique<Grid2DMap>(grid_2d_.data(), grid_2d_.rows(),
                                               grid_2d_.cols());

    Grid2DEnvironment::Config env_config;

    env_config.valid_state_threshold = 1;
    env_ = std::make_unique<Grid2DEnvironment>(env_config);
    EXPECT_TRUE(env_->SetGrid2D(grid_2d_map_.get()));

    planner_ = std::make_unique<Planner>(env_.get());
  }

  void SetObstacles(const std::vector<int>& indices) {
    for (auto index : indices) {
      *(grid_2d_.data() + index) = 2;
    }
  }

  Grid2D grid_2d_;
  std::unique_ptr<Grid2DMap> grid_2d_map_;
  std::unique_ptr<Grid2DEnvironment> env_;
  std::unique_ptr<Planner> planner_;
  std::vector<int> path_;
  std::vector<int> expected_path_;
};

TEST_F(TestDStarLitePlannerOn2DGrid,
       GivenFeasibleProblem_WhenPlanPath_EnsurePlanningSucceeds) {
  SetObstacles({4, 7});

  EXPECT_THAT(planner_->PlanPath(3, 14, {}, path_),
              Eq(PlannerStatus::kSuccess));
  expected_path_ = {3, 6, 10, 14};
  EXPECT_THAT(path_, Eq(expected_path_));
}

TEST_F(TestDStarLitePlannerOn2DGrid,
       GivenInfeasibleProblem_WhenPlanPath_EnsurePlanningFails) {
  SetObstacles({1, 4, 7, 10, 13});

  EXPECT_THAT(planner_->PlanPath(3, 14, {}, path_),
              Eq(PlannerStatus::kInfeasibleProblem));
}

class TestDStarLiteReplanningOn2DGrid : public TestDStarLitePlannerOn2DGrid {
 public:
  TestDStarLiteReplanningOn2DGrid() {
    SetObstacles({4, 7});
    EXPECT_THAT(planner_->PlanPath(3, 14, {}, path_),
                Eq(PlannerStatus::kSuccess));
  }

  auto Replan(int start) {
    SetObstacles(new_obstacles_);
    std::unordered_set<int> states_to_update;
    std::vector<int> predecessors(env_->GetMaxNumPredecessors(), kInvalidIndex);
    for (auto new_obstacle : new_obstacles_) {
      states_to_update.insert(new_obstacle);
      env_->GetPredecessors(new_obstacle, predecessors);
      states_to_update.insert(predecessors.begin(), predecessors.end());
    }
    return planner_->ReplanPath(start, states_to_update, {}, path_);
  }

  std::vector<int> new_obstacles_;
};

TEST_F(TestDStarLiteReplanningOn2DGrid,
       WhenPlanningRequested_EnsurePlanningFromSameStartAndGoalSucceeds) {
  EXPECT_THAT(Replan(3), Eq(PlannerStatus::kSuccess));

  expected_path_ = {3, 6, 10, 14};
  EXPECT_THAT(path_, Eq(expected_path_));
}

TEST_F(TestDStarLiteReplanningOn2DGrid,
       GivenNewStartPoint_WhenFeasible_EnsureReplanningSucceeds) {
  EXPECT_THAT(Replan(6), Eq(PlannerStatus::kSuccess));

  expected_path_ = {6, 10, 14};
  EXPECT_THAT(path_, Eq(expected_path_));
}

TEST_F(TestDStarLiteReplanningOn2DGrid,
       GivenEnvChanges_WhenFeasible_EnsureReplanningSucceeds) {
  new_obstacles_ = {10};
  EXPECT_THAT(Replan(3), Eq(PlannerStatus::kSuccess));

  expected_path_ = {3, 6, 9, 13, 14};
  EXPECT_THAT(path_, Eq(expected_path_));
}

TEST_F(TestDStarLiteReplanningOn2DGrid,
       GivenEnvChangesAndNewStartPoint_WhenFeasible_EnsureReplanningSucceeds) {
  new_obstacles_ = {10};
  EXPECT_THAT(Replan(6), Eq(PlannerStatus::kSuccess));

  expected_path_ = {6, 9, 13, 14};
  EXPECT_THAT(path_, Eq(expected_path_));
}

TEST_F(
    TestDStarLiteReplanningOn2DGrid,
    GivenInfesibleNewSituationAndNewStartPoint_WhenReplanning_EnsureReplanningFails) {  // NOLINT
  new_obstacles_ = {1, 10, 13};

  EXPECT_THAT(Replan(6), Eq(PlannerStatus::kInfeasibleProblem));
}

TEST_F(TestDStarLiteReplanningOn2DGrid,
       GivenNewUnexploredStartPoint_WhenFeasible_EnsureReplanningSucceeds) {
  EXPECT_THAT(Replan(2), Eq(PlannerStatus::kSuccess));

  expected_path_ = {2, 5, 8, 11, 14};
  EXPECT_THAT(path_, Eq(expected_path_));
}

class TestDStarLitePlanner : public Test {
 public:
  using Planner = DStarLitePlanner<PlannerEnvironmentMock>;

  TestDStarLitePlanner() {
    env_.SetUpValidEnv();
    env_.SetUpExpectedValidationCalls();
    planner_ = std::make_unique<Planner>(&env_);
  }

  void SetUpInit() {
    ON_CALL(user_callback_, Call).WillByDefault(Return(true));
    ON_CALL(env_, GetStateSpaceSize).WillByDefault(Return(3));
    ON_CALL(env_, GetMaxNumNeighbors).WillByDefault(Return(1));
    ON_CALL(env_, GetMaxNumPredecessors).WillByDefault(Return(1));

    EXPECT_CALL(env_, GetStateSpaceSize);
    EXPECT_CALL(env_, GetMaxNumNeighbors);
    EXPECT_CALL(env_, GetMaxNumPredecessors);
    EXPECT_CALL(env_, GetHeuristicCost(0, 2)).WillOnce(Return(2));

    EXPECT_CALL(env_, GetStateSpaceSize);
    EXPECT_CALL(env_, GetHeuristicCost(0, 0)).WillOnce(Return(0));
  }

  PlannerEnvironmentMock env_;
  std::unique_ptr<Planner> planner_;
  ::testing::MockFunction<bool(UserCallbackEvent)> user_callback_;
  std::vector<int> path_;
};

// TEST_F(TestDStarLitePlanner, WhenFeasibleProblem_ThenPlanPathSucceeds) {
//   InSequence in_sequence;

//   SetUpInit();

//   // Iteration
//   EXPECT_CALL(user_callback_, Call);
//   EXPECT_CALL(env_, GetHeuristicCost(0, 2)).WillOnce(Return(2));
//   EXPECT_CALL(env_, GetPredecessors(2, _))
//       .WillOnce([](int, std::vector<int>& p) { p = {1}; });
//   EXPECT_CALL(env_, GetNeighborsAndCosts(1, _, _))
//       .WillOnce([](int, std::vector<int>& n, std::vector<float>& c) {
//         n = {2};
//         c = {0.1};
//       });
//   EXPECT_CALL(env_, GetMaxNumNeighbors);
//   EXPECT_CALL(env_, GetHeuristicCost(0, 1)).WillOnce(Return(0.5));

//   // Iteration
//   EXPECT_CALL(user_callback_, Call);
//   EXPECT_CALL(env_, GetHeuristicCost(0, 1)).WillOnce(Return(0.5));
//   EXPECT_CALL(env_, GetPredecessors(1, _))
//       .WillOnce([](int, std::vector<int>& p) { p = {0}; });
//   EXPECT_CALL(env_, GetNeighborsAndCosts(0, _, _))
//       .WillOnce([](int, std::vector<int>& n, std::vector<float>& c) {
//         n = {1};
//         c = {0.1};
//       });
//   EXPECT_CALL(env_, GetMaxNumNeighbors);
//   EXPECT_CALL(env_, GetHeuristicCost(0, 0)).WillOnce(Return(0));

//   // Iteration
//   EXPECT_CALL(user_callback_, Call);
//   EXPECT_CALL(env_, GetHeuristicCost(0, 0)).WillOnce(Return(0));
//   EXPECT_CALL(env_, GetPredecessors(0, _))
//       .WillOnce([](int, std::vector<int>& p) { p = {}; });

//   // Reconstruction
//   EXPECT_CALL(user_callback_, Call).Times(2);

//   EXPECT_THAT(planner_->PlanPath(0, 2, user_callback_.AsStdFunction(),
//               path_), Eq(PlannerStatus::kSuccess));
//   EXPECT_THAT(path_, Eq(std::vector<int>{0, 1, 2}));
// }

// TEST_F(TestDStarLitePlanner, WhenInfeasibleProblem_ThenPlanPathFails) {
//   InSequence in_sequence;

//   SetUpInit();

//   // Iteration
//   EXPECT_CALL(user_callback_, Call);
//   EXPECT_CALL(env_, GetHeuristicCost(0, 2)).WillOnce(Return(2));
//   EXPECT_CALL(env_, GetPredecessors(2, _))
//       .WillOnce([](int, std::vector<int>& p) { p = {1}; });
//   EXPECT_CALL(env_, GetNeighborsAndCosts(1, _, _))
//       .WillOnce([](int, std::vector<int>& n, std::vector<float>& c) {
//         n = {2};
//         c = {0.1};
//       });
//   EXPECT_CALL(env_, GetMaxNumNeighbors);
//   EXPECT_CALL(env_, GetHeuristicCost(0, 1)).WillOnce(Return(0.5));

//   // Iteration
//   EXPECT_CALL(user_callback_, Call);
//   EXPECT_CALL(env_, GetHeuristicCost(0, 1)).WillOnce(Return(0.5));
//   EXPECT_CALL(env_, GetPredecessors(1, _))
//       .WillOnce([](int, std::vector<int>& p) { p = {0}; });
//   EXPECT_CALL(env_, GetNeighborsAndCosts(0, _, _))
//       .WillOnce([](int, std::vector<int>& n, std::vector<float>& c) {
//         n = {1};
//         c = {kInfCost};
//       });
//   EXPECT_CALL(env_, GetMaxNumNeighbors);
//   EXPECT_CALL(env_, GetHeuristicCost(0, 0)).WillOnce(Return(0));

//   // Iteration
//   EXPECT_CALL(user_callback_, Call);
//   EXPECT_CALL(env_, GetHeuristicCost(0, 0)).WillOnce(Return(0));

//   EXPECT_THAT(planner_->PlanPath(0, 2, user_callback_.AsStdFunction(),
//               path_), Eq(PlannerStatus::kInfeasibleProblem));
// }

}  // namespace optimus
