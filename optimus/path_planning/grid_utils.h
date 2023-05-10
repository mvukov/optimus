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
#ifndef OPTIMUS_PATH_PLANNING_GRID_UTILS_H_
#define OPTIMUS_PATH_PLANNING_GRID_UTILS_H_

#include <cmath>
#include <vector>

namespace optimus {

static inline float GetHypot(int start, int goal, int grid_width) {
  const int x_start = start % grid_width;
  const int y_start = start / grid_width;
  const int x_goal = goal % grid_width;
  const int y_goal = goal / grid_width;
  const float x_diff = x_start - x_goal;
  const float y_diff = y_start - y_goal;
  return std::hypot(x_diff, y_diff);
}

void Get8NeighborsOn2dGrid(int pivot, int grid_width, int grid_height,
                           std::vector<int>& neighbors);

void Set8PivotToNeighborCosts(std::vector<float>& costs);

}  // namespace optimus

#endif  // OPTIMUS_PATH_PLANNING_GRID_UTILS_H_
