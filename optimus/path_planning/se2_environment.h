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
#ifndef OPTIMUS_PATH_PLANNING_SE2_ENVIRONMENT_H_
#define OPTIMUS_PATH_PLANNING_SE2_ENVIRONMENT_H_

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <unordered_map>
#include <vector>

#include "optimus/path_planning/action_set_2d.h"
#include "optimus/path_planning/common_utils.h"
#include "optimus/path_planning/grid_2d.h"
#include "optimus/path_planning/grid_utils.h"

namespace optimus {

class SE2Environment {
 public:
  struct Config {
    Grid2DScalar valid_state_threshold = 0;
    float swath_cost_multiplier = 0;
    float length_cost_multiplier = 1.0f;
    float abs_angle_diff_cost_multiplier = 0;

    [[nodiscard]] bool Validate() const;
  };

  struct GridCoords {
    int x = 0;
    int y = 0;
  };

  // action_set must not change in runtime.
  SE2Environment(const Config& config, const ActionSet2D* action_set);

  bool SetGrid2D(const Grid2DMap* grid_2d);

  bool Validate() const;

  bool Initialize();

  bool IsStateIndexValid(int index) const {
    const auto xy_index = index >> num_angle_bits_;
    const auto angle_index = index & angle_mask_;
    return xy_index >= 0 && xy_index < grid_2d_->size() &&
           angle_index < static_cast<int>(action_set_->angles.size());
  }

  bool IsStateValid(int index) const {
    const auto xy_index = index >> num_angle_bits_;
    return *(grid_2d_->data() + xy_index) <= config_.valid_state_threshold;
  }

  int GetStateSpaceSize() const { return state_space_size_; }

  int GetMaxNumNeighbors() const { return max_num_neighbors_; }

  float GetHeuristicCost(int start, int goal) const {
    return GetHypot(start >> num_angle_bits_, goal >> num_angle_bits_,
                    grid_2d_->cols());
  }

  void GetNeighborsAndCosts(int pivot, std::vector<int>& neighbors,
                            std::vector<float>& pivot_to_neighbor_costs);

  GridCoords ToGridCoords(int state_index) const {
    const auto xy_index = state_index >> num_angle_bits_;
    const int grid_width = grid_2d_->cols();
    return {xy_index % grid_width, xy_index / grid_width};
  }

  static std::uint64_t GetEdgeIndex(int from, int to) {
    return (static_cast<std::uint64_t>(from) << 32) + to;
  }

  int GetPrimitiveIndex(int from, int to) const {
    return edges_to_motion_primitive_indices_.at(GetEdgeIndex(from, to));
  }

  [[nodiscard]] int GetMaxNumPredecessors() const { return max_num_neighbors_; }

  void GetPredecessors(int pivot, std::vector<int>& predecessors) const;

  const auto* grid_2d() const { return grid_2d_; }
  const auto& action_set() const { return *action_set_; }
  const auto& config() const { return config_; }
  int num_angle_bits() const { return num_angle_bits_; }
  int angle_mask() const { return angle_mask_; }
  const auto& edges_to_motion_primitive_indices() const {
    return edges_to_motion_primitive_indices_;
  }

 private:
  const Config config_;
  const Grid2DMap* grid_2d_ = nullptr;
  const ActionSet2D* action_set_ = nullptr;
  int num_angle_bits_ = 0;
  int angle_mask_ = 0;
  int state_space_size_ = 0;
  int max_num_neighbors_ = 0;
  // TODO(mvukov) Try abseil flat map.
  std::unordered_map<std::uint64_t, int> edges_to_motion_primitive_indices_;
};

}  // namespace optimus

#endif  // OPTIMUS_PATH_PLANNING_SE2_ENVIRONMENT_H_
