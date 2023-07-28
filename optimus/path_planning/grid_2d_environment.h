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
#ifndef OPTIMUS_PATH_PLANNING_GRID_2D_ENVIRONMENT_H_
#define OPTIMUS_PATH_PLANNING_GRID_2D_ENVIRONMENT_H_

#include <vector>

#include "optimus/path_planning/common_utils.h"
#include "optimus/path_planning/grid_2d.h"
#include "optimus/path_planning/grid_utils.h"

namespace optimus {

class Grid2DEnvironment {
 public:
  enum class Type {
    k8WayConnected,
  };

  struct Config {
    Type type = Type::k8WayConnected;
    Grid2DScalar valid_state_threshold = 0;
  };

  explicit Grid2DEnvironment(const Config& config)
      : config_(config), max_num_neighbors_(GetNumNeighbors(config_.type)) {}

  bool SetGrid2D(const Grid2DMap* grid_2d) {
    if (grid_2d == nullptr) {
      return false;
    }
    grid_2d_ = grid_2d;
    return true;
  }

  [[nodiscard]] bool Validate() const {
    if (grid_2d_ == nullptr) {
      return false;
    }
    return grid_2d_->rows() > 0 && grid_2d_->cols() > 0;
  }

  [[nodiscard]] bool Initialize() { return true; }  // NOLINT

  [[nodiscard]] bool IsStateIndexValid(int index) const {
    return index >= 0 && index < grid_2d_->size();
  }

  [[nodiscard]] bool IsStateValid(int index) const {
    return *(grid_2d_->data() + index) <= config_.valid_state_threshold;
  }

  [[nodiscard]] int GetStateSpaceSize() const { return grid_2d_->size(); }

  [[nodiscard]] int GetMaxNumNeighbors() const { return max_num_neighbors_; }

  [[nodiscard]] float GetHeuristicCost(int start, int goal) const {
    return GetHypot(start, goal, grid_2d_->cols());
  }

  void GetNeighborsAndCosts(int pivot, std::vector<int>& neighbors,
                            std::vector<float>& pivot_to_neighbor_costs) {
    Get8NeighborsOn2dGrid(pivot, grid_2d_->cols(), grid_2d_->rows(), neighbors);
    Set8PivotToNeighborCosts(pivot_to_neighbor_costs);

    for (size_t el = 0; el < neighbors.size(); ++el) {
      const auto neighbor = neighbors[el];
      if (neighbor == kInvalidIndex || !IsStateValid(neighbor)) {
        pivot_to_neighbor_costs[el] = kInfCost;
      }
    }
  }

  [[nodiscard]] int GetMaxNumPredecessors() const { return max_num_neighbors_; }

  void GetPredecessors(int pivot, std::vector<int>& predecessors) const {
    Get8NeighborsOn2dGrid(pivot, grid_2d_->cols(), grid_2d_->rows(),
                          predecessors);
  }

  const auto* grid_2d() const { return grid_2d_; }

 private:
  [[nodiscard]] static int GetNumNeighbors(Type type) {
    switch (type) {
      case Type::k8WayConnected:
        return 8;
    }
    return 0;
  }

  const Config config_;
  const Grid2DMap* grid_2d_ = nullptr;
  const int max_num_neighbors_;
};

}  // namespace optimus

#endif  // OPTIMUS_PATH_PLANNING_GRID_2D_ENVIRONMENT_H_
