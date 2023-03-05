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
#include "optimus/path_planning/grid_utils.h"

#include <algorithm>
#include <array>
#include <utility>

#include "optimus/path_planning/common_utils.h"

namespace optimus {

void Get8NeighborsOn2dGrid(int pivot, int grid_width, int grid_height,
                           std::vector<int>& neighbors) {
  constexpr int kMaxNumNeighbors = 8;
  static const std::array<std::pair<int, int>, kMaxNumNeighbors> kNeighbors = {
      {{-1, -1}, {0, -1}, {1, -1}, {-1, 0}, {1, 0}, {-1, 1}, {0, 1}, {1, 1}},
  };
  const int x = pivot % grid_width;
  const int y = pivot / grid_width;

  std::fill(neighbors.begin(), neighbors.end(), kInvalidIndex);
  const int max_num_neighbors =
      std::min(static_cast<int>(neighbors.size()), kMaxNumNeighbors);
  for (int el = 0; el < max_num_neighbors; ++el) {
    const auto& local_neighbor = kNeighbors[el];
    const auto neighbor_x = x + local_neighbor.first;
    const auto neighbor_y = y + local_neighbor.second;
    if (neighbor_x < 0 || neighbor_x >= grid_width || neighbor_y < 0 ||
        neighbor_y >= grid_height) {
      continue;
    }
    neighbors[el] = neighbor_y * grid_width + neighbor_x;
  }
}

}  // namespace optimus
