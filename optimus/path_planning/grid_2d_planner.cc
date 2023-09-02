// Copyright 2023 Milan Vukov. All rights reserved.
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
#include "optimus/path_planning/grid_2d_planner.h"

#include "optimus/path_planning/grid_2d.h"

namespace optimus {

bool Grid2DPlannerBase::SetGrid2D(const Grid2DMap* grid_2d) {
  return env_.SetGrid2D(grid_2d);
}

int Grid2DPlannerBase::GetStateIndex(const Position2D& p) const {
  const auto x = std::floor(p.x());
  if (x < 0) {
    return kInvalidIndex;
  }
  const auto y = std::floor(p.y());
  if (y < 0) {
    return kInvalidIndex;
  }
  return y * env_.grid_2d()->cols() + x;
}

void Grid2DPlannerBase::ReconstructPath(const std::vector<int>& path_indices,
                                        std::vector<Position2D>& path) const {
  const int grid_width = env_.grid_2d()->cols();
  const int path_size = path_indices.size();
  path.resize(path_size);
  for (int el = 0; el < path_size; ++el) {
    const auto index = path_indices[el];
    constexpr auto kCellOffset = 0.5f;
    path[el] = {static_cast<float>(index % grid_width) + kCellOffset,
                static_cast<float>(index / grid_width) + kCellOffset};
  }
}

}  // namespace optimus
