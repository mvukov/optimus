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
#ifndef OPTIMUS_PATH_PLANNING_ACTION_SET_2D_H_
#define OPTIMUS_PATH_PLANNING_ACTION_SET_2D_H_

#include <map>
#include <vector>

namespace optimus {

struct MotionPrimitive2D {
  static constexpr int kMinLength = 2;

  std::vector<int> swath_x;
  std::vector<int> swath_y;
  float length = 0;
  float abs_angle_diff = 0;
  int end_x_idx = 0;
  int end_y_idx = 0;
  int end_angle_idx = 0;
  std::vector<float> x;
  std::vector<float> y;
  std::vector<float> theta;

  [[nodiscard]] bool Validate() const;
};

struct ActionSet2D {
  // Sorted with unique values, all values in (-pi, pi].
  std::vector<float> angles;
  // The n-th value is index in motion_primitives corresponding to the start
  // of the group of primitives with the n-th start angle in angles array.
  std::vector<int> primitive_group_start_indices;
  // Primitives are grouped and sorted for start angle indices.
  std::vector<MotionPrimitive2D> motion_primitives;

  struct State {
    int x_idx;
    int y_idx;
    int angle_idx;
  };
  // Assumes the current x = 0 and y = 0.
  // Maps angle indices to predecessor states.
  std::map<int, std::vector<State>> angles_to_predecessors;

  [[nodiscard]] bool Validate() const;
  [[nodiscard]] int GetAngleIndex(float angle) const;
};

}  // namespace optimus

#endif  // OPTIMUS_PATH_PLANNING_ACTION_SET_2D_H_
