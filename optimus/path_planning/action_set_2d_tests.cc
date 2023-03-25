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
#include <cmath>

#include "gmock/gmock.h"
#include "gtest/gtest.h"

#include "optimus/path_planning/action_set_2d.h"

namespace optimus {

using ::testing::Eq;
using ::testing::Test;

MotionPrimitive2D CreateValidPrimitive() {
  MotionPrimitive2D primitive;
  primitive.swath_x = {1, 2};
  primitive.swath_y = {3, 4};
  primitive.length = 1.2;
  primitive.abs_angle_diff = 0.6;
  primitive.end_x_idx = 1;
  primitive.end_y_idx = 2;
  primitive.end_angle_idx = 3;
  primitive.x = {0.1, 0.2};
  primitive.y = {0.34, 0.56};
  primitive.theta = {0.78, 0.9};
  return primitive;
}

class TestMotionPrimitive2D : public ::testing::Test {};

TEST_F(TestMotionPrimitive2D,
       GivenValidPrimitives_WhenValidateCalled_EnsureValidationPass) {
  const auto primitive = CreateValidPrimitive();

  EXPECT_TRUE(primitive.Validate());
}

// TODO(mvukov) Add failure tests.

class TestActionSet2D : public ::testing::Test {
 public:
  void CreateValidActionSet() {
    auto& angles = action_set_.angles;
    angles = {-3.0 * M_PI / 4, -M_PI / 2, -M_PI / 4,    0,
              M_PI / 4,        M_PI / 2,  3 * M_PI / 4, M_PI};
    const auto num_angles = angles.size();

    auto& primitive_group_start_indices =
        action_set_.primitive_group_start_indices;
    primitive_group_start_indices.resize(num_angles);
    std::generate(primitive_group_start_indices.begin(),
                  primitive_group_start_indices.end(),
                  [n = 0]() mutable { return n++; });
    const auto valid_primitive = CreateValidPrimitive();
    action_set_.motion_primitives.resize(num_angles, valid_primitive);
  }

  ActionSet2D action_set_;
};

TEST_F(TestActionSet2D, WhenEmptyAngles_EnsureValidationFails) {
  EXPECT_FALSE(action_set_.Validate());
}

TEST_F(TestActionSet2D,
       GivenValidAngles_WhenValidateCalled_EnsureValidationPass) {
  CreateValidActionSet();

  EXPECT_TRUE(action_set_.Validate());
}

TEST_F(TestActionSet2D,
       GivenValidAngles_WhenAnglesNotSorted_EnsureValidationFails) {
  CreateValidActionSet();

  action_set_.angles.front() = 5;

  EXPECT_FALSE(action_set_.Validate());
}

TEST_F(TestActionSet2D,
       GivenValidAngles_WhenFirstAnglePi_EnsureValidationFails) {
  CreateValidActionSet();

  action_set_.angles.front() = -M_PI;

  EXPECT_FALSE(action_set_.Validate());
}

TEST_F(TestActionSet2D,
       GivenValidAngles_WhenLastAngleNotPi_EnsureValidationFails) {
  CreateValidActionSet();

  action_set_.angles.back() = M_PI - 0.1;

  EXPECT_FALSE(action_set_.Validate());
}

TEST_F(
    TestActionSet2D,
    GivenValidStartIndices_WhenNumStartIndicesNeqNumAngles_EnsureValidationFails) {  // NOLINT
  CreateValidActionSet();

  action_set_.primitive_group_start_indices.push_back(
      action_set_.primitive_group_start_indices.back());

  EXPECT_FALSE(action_set_.Validate());
}

TEST_F(TestActionSet2D,
       GivenValidStartIndices_WhenStartIndicesNotSorter_EnsureValidationFails) {
  CreateValidActionSet();

  action_set_.primitive_group_start_indices.back() = 1;

  EXPECT_FALSE(action_set_.Validate());
}

TEST_F(
    TestActionSet2D,
    GivenValidStartIndices_WhenFirstStartIndexNotZero_EnsureValidationFails) {
  CreateValidActionSet();

  action_set_.primitive_group_start_indices.front() = 1;

  EXPECT_FALSE(action_set_.Validate());
}

TEST_F(
    TestActionSet2D,
    GivenValidStartIndices_WhenLastStartIndexGtNumMotionPrimitives_EnsureValidationFails) {  // NOLINT
  CreateValidActionSet();

  action_set_.primitive_group_start_indices.back() =
      action_set_.motion_primitives.size();

  EXPECT_FALSE(action_set_.Validate());
}

class TestGetAngleIndex : public TestActionSet2D {
 public:
  TestGetAngleIndex() { CreateValidActionSet(); }
};

TEST_F(TestGetAngleIndex, WhenAngleMin3PiOver_EnsureIndex0) {
  EXPECT_THAT(action_set_.GetAngleIndex(-3.0 * M_PI / 4), Eq(0));
}

TEST_F(TestGetAngleIndex, WhenAngleMin3PiOverMinSmallDeviation_EnsureIndex0) {
  EXPECT_THAT(action_set_.GetAngleIndex(-3.0 * M_PI / 4 - 0.1), Eq(0));
}

TEST_F(TestGetAngleIndex, WhenAngleMin3PiOverPlusSmallDeviation_EnsureIndex0) {
  EXPECT_THAT(action_set_.GetAngleIndex(-3.0 * M_PI / 4 + 0.1), Eq(0));
}

TEST_F(TestGetAngleIndex, WhenAngleMinPiHalf_EnsureIndex0) {
  EXPECT_THAT(action_set_.GetAngleIndex(-M_PI / 2), Eq(1));
}

TEST_F(TestGetAngleIndex, WhenAngleMinPi_EnsureIndex7) {
  EXPECT_THAT(action_set_.GetAngleIndex(-M_PI), Eq(7));
}

TEST_F(TestGetAngleIndex, WhenAnglePi_EnsureIndex7) {
  EXPECT_THAT(action_set_.GetAngleIndex(M_PI), Eq(7));
}

TEST_F(TestGetAngleIndex, WhenAngle3Pi_EnsureIndex7) {
  EXPECT_THAT(action_set_.GetAngleIndex(3.0 * M_PI), Eq(7));
}

TEST_F(TestGetAngleIndex, WhenAnglePiPlusSmallDeviation_EnsureIndex7) {
  EXPECT_THAT(action_set_.GetAngleIndex(M_PI + 0.1), Eq(7));
}

TEST_F(TestGetAngleIndex, WhenAnglePiMinusSmallDeviation_EnsureIndex7) {
  EXPECT_THAT(action_set_.GetAngleIndex(M_PI - 0.1), Eq(7));
}

TEST_F(TestGetAngleIndex, WhenAngleMinPiOver8_EnsureIndex3) {
  EXPECT_THAT(action_set_.GetAngleIndex(-M_PI / 8.0), Eq(3));
}

TEST_F(TestGetAngleIndex, WhenAnglePlusPiOver8_EnsureIndex4) {
  EXPECT_THAT(action_set_.GetAngleIndex(M_PI / 8.0), Eq(4));
}

}  // namespace optimus
