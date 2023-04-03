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
#ifndef OPTIMUS_PATH_PLANNING_COMMON_UTILS_H_
#define OPTIMUS_PATH_PLANNING_COMMON_UTILS_H_

#include <cmath>
#include <cstdint>
#include <limits>

namespace optimus {

static constexpr float kInfCost = std::numeric_limits<float>::infinity();
static constexpr int kInvalidIndex = -1;

static inline bool AreEqual(
    float a, float b, float tolerance = std::numeric_limits<float>::epsilon()) {
  return std::abs(a - b) <= tolerance;
}

static inline bool IsLess(
    float a, float b, float tolerance = std::numeric_limits<float>::epsilon()) {
  return a + tolerance < b;
}

static inline bool IsGreater(
    float a, float b, float tolerance = std::numeric_limits<float>::epsilon()) {
  return a > b + tolerance;
}

static inline bool IsInf(float a) { return std::isinf(a); }

static inline int GetNumBits(int v) {
  if (v < 0) {
    return sizeof(int) * 8;
  }
  int num_bits = 0;
  while (v != 0) {
    ++num_bits;
    v >>= 1;
  }
  return num_bits;
}

static inline int GetBitMask(int num_bits) {
  if (num_bits == 0) {
    return 0;
  }
  return (1 << num_bits) - 1;
}

}  // namespace optimus

#endif  // OPTIMUS_PATH_PLANNING_COMMON_UTILS_H_
