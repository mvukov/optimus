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
#ifndef OPTIMUS_PATH_PLANNING_PLANNER_ENVIRONMENT_MOCKS_H_
#define OPTIMUS_PATH_PLANNING_PLANNER_ENVIRONMENT_MOCKS_H_

#include <vector>

#include "gmock/gmock.h"

namespace optimus {

class PlannerEnvironmentMock {
 public:
  MOCK_METHOD(bool, Validate, (), (const));
  MOCK_METHOD(bool, Initialize, ());
  MOCK_METHOD(bool, IsStateIndexValid, (int), (const));
  MOCK_METHOD(bool, IsStateValid, (int), (const));

  MOCK_METHOD(int, GetStateSpaceSize, (), (const));
  MOCK_METHOD(int, GetMaxNumNeighbors, (), (const));
  MOCK_METHOD(float, GetHeuristicCost, (int, int), (const));
  // TODO(mvukov) Should this also be a const method???
  MOCK_METHOD(void, GetNeighborsAndCosts,
              (int, std::vector<int>&, std::vector<float>&));

  void SetUpValidEnv() {
    using ::testing::Return;
    ON_CALL(*this, Validate).WillByDefault(Return(true));
    ON_CALL(*this, Initialize).WillByDefault(Return(true));
    ON_CALL(*this, IsStateIndexValid).WillByDefault(Return(true));
    ON_CALL(*this, IsStateValid).WillByDefault(Return(true));
  }

  void SetUpExpectedValidationCalls() {
    EXPECT_CALL(*this, Validate).Times(1);
    EXPECT_CALL(*this, Initialize).Times(1);
    EXPECT_CALL(*this, IsStateIndexValid).Times(2);
    EXPECT_CALL(*this, IsStateValid).Times(2);
  }
};

}  // namespace optimus

#endif  // OPTIMUS_PATH_PLANNING_PLANNER_ENVIRONMENT_MOCKS_H_
