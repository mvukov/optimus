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

#include <cstdint>
#include <vector>

#include "Eigen/Core"

#include "optimus/path_planning/common_utils.h"
#include "optimus/path_planning/grid_utils.h"

namespace optimus {

class Grid2DEnvironment {
 public:
  using ObstacleDataScalar = std::uint8_t;

  enum class Type {
    k8WayConnected,
  };

  struct Config {
    Type type = Type::k8WayConnected;
    ObstacleDataScalar valid_state_threshold = 0;
  };

  using ObstacleData = Eigen::Matrix<ObstacleDataScalar, Eigen::Dynamic,
                                     Eigen::Dynamic, Eigen::RowMajor>;

  explicit Grid2DEnvironment(const Config& config)
      : config_(config), max_num_neighbors_(GetNumNeighbors(config_.type)) {}

  bool SetObstacleData(const ObstacleData* obstacle_data) {
    if (obstacle_data == nullptr) {
      return false;
    }
    obstacle_data_ = obstacle_data;
    return true;
  }

  [[nodiscard]] bool Validate() const {
    if (obstacle_data_ == nullptr) {
      return false;
    }
    return obstacle_data_->rows() > 0 && obstacle_data_->cols() > 0;
  }

  [[nodiscard]] bool Initialize() { return true; }  // NOLINT

  [[nodiscard]] bool IsStateIndexValid(int index) const {
    return index >= 0 && index < obstacle_data_->size();
  }

  [[nodiscard]] bool IsStateValid(int index) const {
    return *(obstacle_data_->data() + index) <= config_.valid_state_threshold;
  }

  [[nodiscard]] int GetStateSpaceSize() const { return obstacle_data_->size(); }

  [[nodiscard]] int GetMaxNumNeighbors() const { return max_num_neighbors_; }

  [[nodiscard]] float GetHeuristicCost(int start, int goal) const {
    return GetHypot(start, goal, obstacle_data_->cols());
  }

  void GetNeighborsAndCosts(int pivot, std::vector<int>& neighbors,
                            std::vector<float>& pivot_to_neighbor_costs) {
    Get8NeighborsOn2dGrid(pivot, obstacle_data_->cols(), obstacle_data_->rows(),
                          neighbors);
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
    Get8NeighborsOn2dGrid(pivot, obstacle_data_->cols(), obstacle_data_->rows(),
                          predecessors);
  }

  const auto& obstacle_data() const { return *obstacle_data_; }

 private:
  [[nodiscard]] static int GetNumNeighbors(Type type) {
    switch (type) {
      case Type::k8WayConnected:
        return 8;
    }
    return 0;
  }

  const Config config_;
  const ObstacleData* obstacle_data_ = nullptr;
  const int max_num_neighbors_;
};

}  // namespace optimus

#endif  // OPTIMUS_PATH_PLANNING_GRID_2D_ENVIRONMENT_H_
