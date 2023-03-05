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
#include <memory>

#include "gmock/gmock.h"
#include "gtest/gtest.h"

#include "optimus/path_planning/astar_planner.h"
#include "optimus/path_planning/planner_environment_mocks.h"

namespace optimus {

using ::testing::_;
using ::testing::Eq;
using ::testing::InSequence;
using ::testing::Return;
using ::testing::Test;

class TestAStarPlanner : public Test {
 public:
  TestAStarPlanner() {
    env_.SetUpValidEnv();
    env_.SetUpExpectedValidationCalls();
    planner_ = std::make_unique<AStarPlanner<PlannerEnvironmentMock>>(&env_);
  }

  PlannerEnvironmentMock env_;
  std::unique_ptr<AStarPlanner<PlannerEnvironmentMock>> planner_;
  ::testing::MockFunction<bool()> user_callback_;
  std::vector<int> path_;
};

TEST_F(TestAStarPlanner, WhenFeasibleProblem_ThenPlanPathSucceeds) {
  InSequence in_sequence;

  ON_CALL(user_callback_, Call).WillByDefault(Return(true));
  ON_CALL(env_, GetMaxNumNeighbors).WillByDefault(Return(1));

  EXPECT_CALL(env_, GetStateSpaceSize).WillOnce(Return(3));
  EXPECT_CALL(env_, GetMaxNumNeighbors);
  EXPECT_CALL(env_, GetHeuristicCost(0, 2)).WillOnce(Return(2));

  EXPECT_CALL(user_callback_, Call);
  std::vector<int> neighbors0 = {1};
  EXPECT_CALL(env_, GetNeighborsAndCosts(0, _, _))
      .WillOnce([](int, std::vector<int>& n, std::vector<float>& c) {
        n = {1};
        c = {0.5};
      });
  EXPECT_CALL(env_, GetMaxNumNeighbors);
  EXPECT_CALL(env_, GetHeuristicCost(1, 2)).WillOnce(Return(1));

  EXPECT_CALL(user_callback_, Call);
  std::vector<int> neighbors1 = {2};
  EXPECT_CALL(env_, GetNeighborsAndCosts(1, _, _))
      .WillOnce([](int, std::vector<int>& n, std::vector<float>& c) {
        n = {2};
        c = {0.1};
      });
  EXPECT_CALL(env_, GetMaxNumNeighbors);
  EXPECT_CALL(env_, GetHeuristicCost(2, 2)).WillOnce(Return(0.2));

  EXPECT_CALL(user_callback_, Call).Times(4);

  EXPECT_THAT(planner_->PlanPath(0, 2, user_callback_.AsStdFunction(), path_),
              Eq(PlannerStatus::kSuccess));
  EXPECT_THAT(path_, Eq(std::vector<int>{0, 1, 2}));
}

TEST_F(TestAStarPlanner, WhenInfeasibleProblem_ThenPlanPathFails) {
  InSequence in_sequence;

  ON_CALL(user_callback_, Call).WillByDefault(Return(true));
  ON_CALL(env_, GetMaxNumNeighbors).WillByDefault(Return(1));

  EXPECT_CALL(env_, GetStateSpaceSize).WillOnce(Return(3));
  EXPECT_CALL(env_, GetMaxNumNeighbors);
  EXPECT_CALL(env_, GetHeuristicCost(0, 2)).WillOnce(Return(2));

  EXPECT_CALL(user_callback_, Call);
  EXPECT_CALL(env_, GetNeighborsAndCosts(0, _, _))
      .WillOnce([](int, std::vector<int>& n, std::vector<float>& c) {
        n = {1};
        c = {kInfCost};
      });
  EXPECT_CALL(env_, GetMaxNumNeighbors);

  EXPECT_THAT(planner_->PlanPath(0, 2, user_callback_.AsStdFunction(), path_),
              Eq(PlannerStatus::kInfeasibleProblem));
}

}  // namespace optimus
