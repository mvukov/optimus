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

#include "optimus/path_planning/planner_algorithm.h"
#include "optimus/path_planning/planner_environment_mocks.h"

namespace optimus {

using ::testing::_;
using ::testing::Eq;
using ::testing::Return;
using ::testing::Test;

class PlannerAlgorithmMock
    : public PlannerAlgorithm<PlannerAlgorithmMock, PlannerEnvironmentMock> {
  using Base = PlannerAlgorithm<PlannerAlgorithmMock, PlannerEnvironmentMock>;

 public:
  using Base::Base;

  MOCK_METHOD(PlannerStatus, PlanPathImpl,
              (int start, int goal, const UserCallback& user_callback,
               std::vector<int>& path));
};

class TestPlannerAlgorithm : public Test {
 public:
  TestPlannerAlgorithm() {
    planner_ = std::make_unique<PlannerAlgorithmMock>(&env_);
  }

  PlannerEnvironmentMock env_;
  std::unique_ptr<PlannerAlgorithmMock> planner_;
  std::vector<int> path_;
};

TEST_F(TestPlannerAlgorithm, WhenInvalidEnv_ThenPlanPathFails) {
  EXPECT_CALL(env_, Validate()).Times(1).WillOnce(Return(false));
  EXPECT_THAT(planner_->PlanPath(0, 0, {}, path_),
              Eq(PlannerStatus::kInternalError));
}

TEST_F(TestPlannerAlgorithm, WhenStartStateIndexInvalid_ThenPlanPathFails) {
  EXPECT_CALL(env_, Validate()).Times(1).WillOnce(Return(true));
  EXPECT_CALL(env_, IsStateIndexValid(1)).Times(1).WillOnce(Return(false));
  EXPECT_THAT(planner_->PlanPath(1, 0, {}, path_),
              Eq(PlannerStatus::kInternalError));
}

TEST_F(TestPlannerAlgorithm, WhenStartStateInvalid_ThenPlanPathFails) {
  EXPECT_CALL(env_, Validate()).Times(1).WillOnce(Return(true));
  EXPECT_CALL(env_, IsStateIndexValid(1)).Times(1).WillOnce(Return(true));
  EXPECT_CALL(env_, IsStateValid(1)).Times(1).WillOnce(Return(false));
  EXPECT_THAT(planner_->PlanPath(1, 0, {}, path_),
              Eq(PlannerStatus::kInternalError));
}

TEST_F(TestPlannerAlgorithm, WhenGoalStateIndexInvalid_ThenPlanPathFails) {
  EXPECT_CALL(env_, Validate()).Times(1).WillOnce(Return(true));
  EXPECT_CALL(env_, IsStateIndexValid(1)).Times(1).WillOnce(Return(true));
  EXPECT_CALL(env_, IsStateValid(1)).Times(1).WillOnce(Return(true));
  EXPECT_CALL(env_, IsStateIndexValid(2)).Times(1).WillOnce(Return(false));
  EXPECT_THAT(planner_->PlanPath(1, 2, {}, path_),
              Eq(PlannerStatus::kInternalError));
}

TEST_F(TestPlannerAlgorithm, WhenGoalStateInvalid_ThenPlanPathFails) {
  EXPECT_CALL(env_, Validate()).Times(1).WillOnce(Return(true));
  EXPECT_CALL(env_, IsStateIndexValid(1)).Times(1).WillOnce(Return(true));
  EXPECT_CALL(env_, IsStateValid(1)).Times(1).WillOnce(Return(true));
  EXPECT_CALL(env_, IsStateIndexValid(2)).Times(1).WillOnce(Return(true));
  EXPECT_CALL(env_, IsStateValid(2)).Times(1).WillOnce(Return(false));
  EXPECT_THAT(planner_->PlanPath(1, 2, {}, path_),
              Eq(PlannerStatus::kInternalError));
}

TEST_F(TestPlannerAlgorithm, WhenEnvInitFails_ThenPlanPathFails) {
  EXPECT_CALL(env_, Validate()).Times(1).WillOnce(Return(true));
  EXPECT_CALL(env_, IsStateIndexValid(_)).Times(2).WillRepeatedly(Return(true));
  EXPECT_CALL(env_, IsStateValid(_)).Times(2).WillRepeatedly(Return(true));
  EXPECT_CALL(env_, Initialize()).Times(1).WillOnce(Return(false));
  EXPECT_THAT(planner_->PlanPath(0, 0, {}, path_),
              Eq(PlannerStatus::kInternalError));
}

TEST_F(TestPlannerAlgorithm,
       GivenValidEnv_WhenPlanPathImplFails_ThenPlanPathFails) {
  env_.SetUpValidEnv();
  env_.SetUpExpectedValidationCalls();

  EXPECT_CALL(*planner_, PlanPathImpl(1, 3, _, path_))
      .Times(1)
      .WillOnce(Return(PlannerStatus::kInternalError));

  EXPECT_THAT(planner_->PlanPath(1, 3, {}, path_),
              Eq(PlannerStatus::kInternalError));
}

TEST_F(TestPlannerAlgorithm,
       GivenValidEnv_WhenPlanPathImplSucceeds_ThenPlanPathSucceeds) {
  env_.SetUpValidEnv();
  env_.SetUpExpectedValidationCalls();

  EXPECT_CALL(*planner_, PlanPathImpl(2, 7, _, path_))
      .Times(1)
      .WillOnce(Return(PlannerStatus::kSuccess));

  EXPECT_THAT(planner_->PlanPath(2, 7, {}, path_), Eq(PlannerStatus::kSuccess));
}

}  // namespace optimus
