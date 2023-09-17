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
#ifndef OPTIMUS_PATH_PLANNING_ASTAR_PLANNER_H_
#define OPTIMUS_PATH_PLANNING_ASTAR_PLANNER_H_

#include <vector>

#include "boost/unordered/unordered_flat_map.hpp"
#include "boost/unordered/unordered_flat_set.hpp"

#include "optimus/path_planning/planner_algorithm.h"
#include "optimus/path_planning/priority_queue_utils.h"

namespace optimus {

template <class Environment>
class AStarPlanner
    : public PlannerAlgorithm<AStarPlanner<Environment>, Environment> {
  using Base = PlannerAlgorithm<AStarPlanner<Environment>, Environment>;

 public:
  using Base::Base;

  PlannerStatus PlanPathImpl(int start, int goal,
                             const UserCallback& user_callback,
                             std::vector<int>& path);

  auto GetGValue(int index) const { return indices_to_g_values_.at(index); }

 private:
  void Reset();
  PlannerStatus Expand(int goal, const UserCallback& user_callback);
  void UpdateIndex(int goal, int pivot, int neighbor,
                   float pivot_to_neighbor_cost);
  bool ReconstructShortestPath(int goal, const UserCallback& user_callback,
                               std::vector<int>& path);

  PriorityQueue open_queue_;

  // It is way faster to have those below as std::vector at the expense of
  // increased memory consumption.
  boost::unordered::unordered_flat_set<int> closed_;
  boost::unordered::unordered_flat_map<int, int> indices_to_parent_indices_;
  boost::unordered::unordered_flat_map<int, float> indices_to_g_values_;

  std::vector<int> neighbors_;
  std::vector<float> pivot_to_neighbor_costs_;
};

template <class E>
PlannerStatus AStarPlanner<E>::PlanPathImpl(int start, int goal,
                                            const UserCallback& user_callback,
                                            std::vector<int>& path) {
  Reset();

  open_queue_.insert(start, Key{this->env_->GetHeuristicCost(start, goal), 0});
  indices_to_parent_indices_[start] = start;
  indices_to_g_values_[start] = 0;

  if (auto status = Expand(goal, user_callback);
      status != PlannerStatus::kSuccess) {
    return status;
  }
  if (!ReconstructShortestPath(goal, user_callback, path)) {
    return PlannerStatus::kInternalError;
  }
  return PlannerStatus::kSuccess;
}

template <class E>
void AStarPlanner<E>::Reset() {
  const auto state_space_size = this->env_->GetStateSpaceSize();
  open_queue_.clear();
  closed_.clear();
  closed_.reserve(state_space_size);
  indices_to_parent_indices_.clear();
  indices_to_parent_indices_.reserve(state_space_size);
  indices_to_g_values_.clear();
  indices_to_g_values_.reserve(state_space_size);

  const auto max_num_neighbors = this->env_->GetMaxNumNeighbors();
  neighbors_.resize(max_num_neighbors, kInvalidIndex);
  std::fill(neighbors_.begin(), neighbors_.end(), kInvalidIndex);
  pivot_to_neighbor_costs_.resize(max_num_neighbors, kInfCost);
  std::fill(pivot_to_neighbor_costs_.begin(), pivot_to_neighbor_costs_.end(),
            kInfCost);
}

template <class E>
PlannerStatus AStarPlanner<E>::Expand(int goal,
                                      const UserCallback& user_callback) {
  while (!open_queue_.empty()) {
    auto pivot = open_queue_.top();
    open_queue_.pop();
    const auto pivot_index = pivot.index;

    if (pivot_index == goal) {
      return PlannerStatus::kSuccess;
    }

    if (user_callback && !user_callback(UserCallbackEvent::kSearch)) {
      return PlannerStatus::kUserAbort;
    }

    closed_.insert(pivot_index);
    this->env_->GetNeighborsAndCosts(pivot_index, neighbors_,
                                     pivot_to_neighbor_costs_);
    const auto max_num_neighbors = this->env_->GetMaxNumNeighbors();
    for (int el = 0; el < max_num_neighbors; ++el) {
      const auto neighbor = neighbors_[el];
      if (neighbor == kInvalidIndex || closed_.count(neighbor) > 0) {
        continue;
      }
      if (indices_to_g_values_.count(neighbor) == 0) {
        indices_to_g_values_[neighbor] = kInfCost;
        indices_to_parent_indices_[neighbor] = kInvalidIndex;
      }
      UpdateIndex(goal, pivot_index, neighbor, pivot_to_neighbor_costs_[el]);
    }
  }

  return PlannerStatus::kInfeasibleProblem;
}

template <class E>
void AStarPlanner<E>::UpdateIndex(int goal, int pivot, int neighbor,
                                  float pivot_to_neighbor_cost) {
  const auto new_g_value_ =
      indices_to_g_values_[pivot] + pivot_to_neighbor_cost;
  if (new_g_value_ < indices_to_g_values_[neighbor]) {
    indices_to_g_values_[neighbor] = new_g_value_;
    indices_to_parent_indices_[neighbor] = pivot;

    open_queue_.InsertOrUpdate(
        neighbor,
        Key{new_g_value_ + this->env_->GetHeuristicCost(neighbor, goal),
            new_g_value_});
  }
}

template <class E>
bool AStarPlanner<E>::ReconstructShortestPath(int goal,
                                              const UserCallback& user_callback,
                                              std::vector<int>& path) {
  auto path_length = 1;
  auto current = goal;
  while (true) {
    if (user_callback && !user_callback(UserCallbackEvent::kReconstruction)) {
      return false;
    }
    auto parent = indices_to_parent_indices_.at(current);
    if (parent == current) {
      break;
    }
    ++path_length;
    current = parent;
  }

  path.resize(path_length);
  path.back() = goal;
  auto offset = path.size() - 2;
  current = goal;
  while (true) {
    auto parent = indices_to_parent_indices_.at(current);
    if (parent == current) {
      break;
    }
    path[offset] = parent;
    --offset;
    current = parent;
  }
  return true;
}

}  // namespace optimus

#endif  // OPTIMUS_PATH_PLANNING_ASTAR_PLANNER_H_
