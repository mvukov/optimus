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
#include "optimus/path_planning/se2_environment.h"

#include "optimus/check.h"

namespace optimus {

bool SE2Environment::Config::Validate() const {
  if (swath_cost_multiplier < 0.) {
    return false;
  }
  if (length_cost_multiplier < 0.) {
    return false;
  }
  if (abs_angle_diff_cost_multiplier < 0.) {
    return false;
  }
  return true;
}

SE2Environment::SE2Environment(const Config& config,
                               const ActionSet2D* action_set)
    : config_(config), action_set_(action_set) {
  OPTIMUS_CHECK(config.Validate());
  OPTIMUS_CHECK(action_set != nullptr);

  const auto num_angles = action_set_->angles.size();
  num_angle_bits_ = GetNumBits(num_angles - 1);
  angle_mask_ = GetBitMask(num_angle_bits_);
}

bool SE2Environment::SetObstacleData(const ObstacleData* obstacle_data) {
  if (obstacle_data == nullptr) {
    return false;
  }
  obstacle_data_ = obstacle_data;
  return true;
}

bool SE2Environment::Validate() const {
  if (obstacle_data_ == nullptr) {
    return false;
  }
  if (obstacle_data_->cols() <= 0 || obstacle_data_->rows() <= 0) {
    return false;
  }
  if (!action_set_->Validate()) {
    return false;
  }
  return true;
}

bool SE2Environment::Initialize() {
  OPTIMUS_CHECK(obstacle_data_ != nullptr);

  // TODO(mvukov) For num_angles != 2^n the actual state space is in fact
  // smaller.
  state_space_size_ = obstacle_data_->size() * (angle_mask_ + 1);
  if (state_space_size_ <= 0) {
    return false;
  }

  const auto& primitive_group_start_indices =
      action_set_->primitive_group_start_indices;
  const int num_angles = primitive_group_start_indices.size();
  const int num_primitives = action_set_->motion_primitives.size();
  max_num_neighbors_ = 0;
  for (int el = 0; el < num_angles - 1; ++el) {
    max_num_neighbors_ =
        std::max(max_num_neighbors_, primitive_group_start_indices[el + 1] -
                                         primitive_group_start_indices[el]);
  }
  max_num_neighbors_ =
      std::max(max_num_neighbors_,
               num_primitives - primitive_group_start_indices[num_angles - 1]);

  edges_to_motion_primitive_indices_.clear();
  edges_to_motion_primitive_indices_.reserve(state_space_size_ *
                                             max_num_neighbors_);
  return true;
}

void SE2Environment::GetNeighborsAndCosts(
    int pivot, std::vector<int>& neighbors,
    std::vector<float>& pivot_to_neighbor_costs) {
  OPTIMUS_CHECK(obstacle_data_ != nullptr);

  std::fill(neighbors.begin(), neighbors.end(), kInvalidIndex);
  std::fill(pivot_to_neighbor_costs.begin(), pivot_to_neighbor_costs.end(),
            kInfCost);
  if (static_cast<int>(neighbors.size()) != GetMaxNumNeighbors() ||
      neighbors.size() != pivot_to_neighbor_costs.size()) {
    return;
  }

  const auto xy_coords = ToGridCoords(pivot);
  const auto angle_index = pivot & angle_mask_;

  const int grid_width = obstacle_data_->cols();
  const int grid_height = obstacle_data_->rows();

  const int num_angles = action_set_->angles.size();
  const auto primitive_index_start =
      action_set_->primitive_group_start_indices.at(angle_index);
  const int primitive_index_end =
      angle_index < num_angles - 1
          ? action_set_->primitive_group_start_indices.at(angle_index + 1)
          : action_set_->motion_primitives.size();

  int offset = 0;
  for (int primitive_index = primitive_index_start;
       primitive_index < primitive_index_end; ++primitive_index) {
    const auto& p = action_set_->motion_primitives[primitive_index];

    bool discard = false;
    const auto swath_size = p.swath_x.size();
    float cost = 0;
    for (size_t el = 0; el < swath_size; ++el) {
      const auto candidate_x = xy_coords.x + p.swath_x[el];
      const auto candidate_y = xy_coords.y + p.swath_y[el];
      if (candidate_x < 0 || candidate_x >= grid_width || candidate_y < 0 ||
          candidate_y >= grid_height) {
        discard = true;
        break;
      }
      const auto candidate_cost = (*obstacle_data_)(candidate_y, candidate_x);
      // TODO(mvukov) Should call IsStateValid here!
      if (candidate_cost > config_.valid_state_threshold) {
        discard = true;
        break;
      }
      cost += candidate_cost;
    }
    if (discard) {
      continue;
    }

    cost *= config_.swath_cost_multiplier;
    cost += config_.length_cost_multiplier * p.length;
    cost += config_.abs_angle_diff_cost_multiplier * p.abs_angle_diff;

    const auto neighbor_index = (((xy_coords.y + p.end_y_idx) * grid_width +
                                  (xy_coords.x + p.end_x_idx))
                                 << num_angle_bits_) +
                                p.end_angle_idx;
    neighbors[offset] = neighbor_index;
    pivot_to_neighbor_costs[offset] = cost;
    ++offset;
    edges_to_motion_primitive_indices_[GetEdgeIndex(pivot, neighbor_index)] =
        primitive_index;
  }
}

}  // namespace optimus
