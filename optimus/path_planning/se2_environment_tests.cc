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
#include <algorithm>
#include <memory>

#include "gmock/gmock.h"
#include "gtest/gtest.h"

#include "optimus/path_planning/se2_environment.h"

namespace optimus {

using ::testing::Eq;
using ::testing::Test;

class TestSE2EnvironmentConfig : public Test {
 public:
  SE2Environment::Config config_;
};

TEST_F(TestSE2EnvironmentConfig, WhenDefaultConfigValid_EnsureValidatePasses) {
  EXPECT_TRUE(config_.Validate());
}

TEST_F(TestSE2EnvironmentConfig,
       GivenValidConfig_WhenSwathCostMultiplier_EnsureValidateFails) {
  config_.swath_cost_multiplier = -1;
  EXPECT_FALSE(config_.Validate());
}

TEST_F(TestSE2EnvironmentConfig,
       GivenValidConfig_WhenLengthCostMultiplier_EnsureValidateFails) {
  config_.length_cost_multiplier = -0.1;
  EXPECT_FALSE(config_.Validate());
}

TEST_F(TestSE2EnvironmentConfig,
       GivenValidConfig_WhenAbsAngleDiffCostMultiplier_EnsureValidateFails) {
  config_.abs_angle_diff_cost_multiplier = -1.3;
  EXPECT_FALSE(config_.Validate());
}

class TestSE2Environment : public Test {
 public:
  void CreateValidObstacles() {
    obstacles_.resize(6, 4);
    obstacles_.setConstant(0);
  }

  // TODO(mvukov) Factor out to a common test header.
  void CreateValidActionSet() {
    auto& angles = action_set_.angles;
    angles = {-3.0 * M_PI / 4, -M_PI / 2, -M_PI / 4, 0,
              M_PI / 4,        M_PI / 2,  M_PI};
    const int num_angles = angles.size();
    auto& primitive_group_start_indices =
        action_set_.primitive_group_start_indices;
    primitive_group_start_indices.resize(num_angles);
    std::generate(primitive_group_start_indices.begin(),
                  primitive_group_start_indices.end(),
                  [n = 0]() mutable { return n++; });
    auto& primitives = action_set_.motion_primitives;
    primitives.resize(num_angles);
    for (int el = 0; el < num_angles; ++el) {
      auto& p = primitives.at(el);
      const auto angle = angles.at(el);
      p.length = 1.0;
      p.abs_angle_diff = 0;
      p.x = {0, 1};
      p.y = {0, 1};
      p.theta = {angle, angle};
      p.swath_x = {0};
      p.swath_y = {0};
      p.end_angle_idx = el;
      p.end_x_idx = 1;
      p.end_y_idx = 1;
    }
  }

  void CreateValidEnvironment() {
    CreateValidObstacles();
    CreateValidActionSet();
    env_ = std::make_unique<SE2Environment>(env_config_, &action_set_);
    env_->SetObstacleData(&obstacles_);
  }

  SE2Environment::Config env_config_;
  ActionSet2D action_set_;
  std::unique_ptr<SE2Environment> env_;
  SE2Environment::ObstacleData obstacles_;
};

TEST_F(TestSE2Environment,
       GivenValidActionSet_WhenObstaclesNotSet_EnsureValidateFails) {
  CreateValidActionSet();
  env_ = std::make_unique<SE2Environment>(env_config_, &action_set_);

  EXPECT_FALSE(env_->Validate());
}

TEST_F(TestSE2Environment, GivenValidEnv_EnsureValidatePasses) {
  CreateValidEnvironment();

  EXPECT_TRUE(env_->Validate());
}

class TestSimpleSE2Enviroment : public TestSE2Environment {
 public:
  void SetUp() override {
    CreateValidEnvironment();
    ASSERT_TRUE(env_->Initialize());
  }
};

TEST_F(TestSimpleSE2Enviroment, EnsureNumAngleBits3) {
  EXPECT_THAT(env_->num_angle_bits(), Eq(3));
}

TEST_F(TestSimpleSE2Enviroment, EnsureAngleMask7) {
  EXPECT_THAT(env_->angle_mask(), Eq(7));
}

TEST_F(TestSimpleSE2Enviroment, EnsureStateSpaceSize24Times8) {
  EXPECT_THAT(env_->GetStateSpaceSize(), Eq(24 * 8));
}

TEST_F(TestSimpleSE2Enviroment, EnsureMaxNumNeighbors1) {
  EXPECT_THAT(env_->GetMaxNumNeighbors(), Eq(1));
}

TEST_F(TestSimpleSE2Enviroment,
       WhenNumNeighborsIncreased_EnsureMaxNumNeighbors2) {
  auto& primitives = action_set_.motion_primitives;
  primitives.push_back(primitives.front());
  EXPECT_TRUE(env_->Initialize());

  EXPECT_THAT(env_->GetMaxNumNeighbors(), Eq(2));
}

TEST_F(TestSimpleSE2Enviroment,
       WhenValidStateIndex_EnsureIsStateIndexValidPasses) {
  EXPECT_TRUE(env_->IsStateIndexValid((22 << 3) + 5));
}

TEST_F(TestSimpleSE2Enviroment,
       WhenInvalidStateXYIndex_EnsureIsStateIndexValidFails) {
  EXPECT_FALSE(env_->IsStateIndexValid((25 << 3) + 5));
}

TEST_F(TestSimpleSE2Enviroment,
       WhenInvalidStateAngleIndex_EnsureIsStateIndexValidFails) {
  EXPECT_FALSE(env_->IsStateIndexValid((21 << 3) + 7));
}

TEST_F(TestSimpleSE2Enviroment, WhenValidState_EnsureIsStateValidPasses) {
  EXPECT_TRUE(env_->IsStateValid(2));
}

TEST_F(TestSimpleSE2Enviroment, WhenInValidState_EnsureIsStateValidFails) {
  obstacles_(0, 3) = 5;
  EXPECT_FALSE(env_->IsStateValid(3 << 3));
}

}  // namespace optimus
