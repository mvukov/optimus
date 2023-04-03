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
#include "gmock/gmock.h"
#include "gtest/gtest.h"

#include "optimus/path_planning/common_utils.h"

namespace optimus {

using ::testing::Eq;

TEST(TestGetNumBits, WhenGiven0_Ensure1Bit) {
  EXPECT_THAT(GetNumBits(0), Eq(0));
}

TEST(TestGetNumBits, WhenGiven2_Ensure2Bits) {
  EXPECT_THAT(GetNumBits(2), Eq(2));
}

TEST(TestGetNumBits, WhenGiven128_Ensure8Bits) {
  EXPECT_THAT(GetNumBits(128), Eq(8));
}

TEST(TestGetNumBits, WhenGiven127_Ensure7Bits) {
  EXPECT_THAT(GetNumBits(127), Eq(7));
}

TEST(TestGetNumBits, WhenGiven96_Ensure8Bits) {
  EXPECT_THAT(GetNumBits(96), Eq(7));
}

TEST(TestGetNumBits, WhenGivenMin1_Ensure32Bits) {
  EXPECT_THAT(GetNumBits(-1), Eq(32));
}

TEST(TestGetBitMask, WhenNumBits0_EnsureMaskIs0) {
  EXPECT_THAT(GetBitMask(0), Eq(0));
}

TEST(TestGetBitMask, WhenNumBits1_EnsureMaskIs1) {
  EXPECT_THAT(GetBitMask(1), Eq(1));
}

TEST(TestGetBitMask, WhenNumBits8_EnsureMaskIs255) {
  EXPECT_THAT(GetBitMask(8), Eq(255));
}

TEST(TestIsInf, WhenInfCost_EnsureIsInfPasses) { EXPECT_TRUE(IsInf(kInfCost)); }

TEST(TestIsInf, WhenFiniteValue_EnsureIsInfFails) {
  EXPECT_FALSE(IsInf(std::numeric_limits<float>::max()));
}

}  // namespace optimus
