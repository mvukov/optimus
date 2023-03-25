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
#include "optimus/path_planning/action_set_2d.h"

#include <algorithm>
#include <cmath>
#include <limits>

#include "optimus/path_planning/common_utils.h"

namespace optimus {
namespace {
constexpr float kTwoPi = 2.0 * M_PI;
}  // namespace

bool MotionPrimitive2D::Validate() const {
  if (swath_x.empty()) {
    return false;
  }
  if (swath_x.size() != swath_y.size()) {
    return false;
  }
  if (length <= 0) {
    return false;
  }
  if (abs_angle_diff < 0) {
    return false;
  }
  if (end_x_idx == 0 && end_y_idx == 0) {
    return false;
  }
  if (static_cast<int>(x.size()) < kMinLength) {
    return false;
  }
  if (x.size() != y.size()) {
    return false;
  }
  if (x.size() != theta.size()) {
    return false;
  }
  return true;
}

bool ActionSet2D::Validate() const {
  if (angles.empty()) {
    return false;
  }
  if (!std::is_sorted(angles.begin(), angles.end())) {
    return false;
  }
  if (angles.front() <= -M_PI) {
    return false;
  }
  if (!AreEqual(angles.back(), M_PI)) {
    return false;
  }

  if (primitive_group_start_indices.size() != angles.size()) {
    return false;
  }
  if (!std::is_sorted(primitive_group_start_indices.begin(),
                      primitive_group_start_indices.end())) {
    return false;
  }
  if (primitive_group_start_indices.front() != 0) {
    return false;
  }
  if (primitive_group_start_indices.back() >=
      static_cast<int>(motion_primitives.size())) {
    return false;
  }

  // TODO(mvukov) Cross-check angles, primitive_group_start_indices and
  // motion_primitives.

  for (const auto& primitive : motion_primitives) {
    if (!primitive.Validate()) {
      return false;
    }
  }
  return true;
}

int ActionSet2D::GetAngleIndex(float angle) const {
  const auto wrapped_angle = std::remainder(angle, kTwoPi);
  const auto normalized_angle =
      AreEqual(wrapped_angle, -M_PI) ? M_PI : wrapped_angle;
  const int num_angles = angles.size();

  const auto low_0 = (angles[0] + angles[num_angles - 1] - kTwoPi) / 2.0f;
  if (normalized_angle < low_0) {
    return num_angles - 1;
  }

  auto low = std::numeric_limits<float>::lowest();
  auto high = std::numeric_limits<float>::max();
  for (int el = 0; el < num_angles; ++el) {
    if (el == 0) {
      low = low_0;
      high = (angles[el] + angles[el + 1]) / 2.0f;
    } else if (el == num_angles - 1) {
      low = (angles[el] + angles[el - 1]) / 2.0f;
      high = (angles[el] + angles[0] + kTwoPi) / 2.0f;
    } else {
      low = (angles[el] + angles[el - 1]) / 2.0f;
      high = (angles[el] + angles[el + 1]) / 2.0f;
    }
    if (low <= normalized_angle && normalized_angle < high) {
      return el;
    }
  }
  return kInvalidIndex;
}

}  // namespace optimus
