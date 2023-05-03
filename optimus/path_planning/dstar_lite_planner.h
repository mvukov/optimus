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
#ifndef OPTIMUS_PATH_PLANNING_DSTAR_LITE_PLANNER_H_
#define OPTIMUS_PATH_PLANNING_DSTAR_LITE_PLANNER_H_

#include <algorithm>
#include <vector>

#include "optimus/path_planning/planner_algorithm.h"
#include "optimus/path_planning/priority_queue_utils.h"

namespace optimus {

template <class Environment>
class DStarLitePlanner
    : public PlannerAlgorithm<DStarLitePlanner<Environment>, Environment> {
  using Base = PlannerAlgorithm<DStarLitePlanner<Environment>, Environment>;

 public:
  using Base::Base;

  PlannerStatus PlanPathImpl(int start, int goal,
                             const UserCallback& user_callback,
                             std::vector<int>& path);

  PlannerStatus ReplanPathImpl(int start,
                               const std::vector<int>& changed_states,
                               const UserCallback& user_callback,
                               std::vector<int>& path);

 private:
  void Reset();

  PlannerStatus CalculateShortestPath(const UserCallback& user_callback);
  Key CalculateKey(int index) const;
  void UpdateVertex(int pivot);
  void UpdatePredecessors(int pivot);
  PlannerStatus ReconstructShortestPath(const UserCallback& user_callback,
                                        std::vector<int>& path);

  PriorityQueue open_queue_;
  std::vector<bool> open_states_;
  std::vector<float> g_values_;
  std::vector<float> rhs_values_;
  std::vector<int> best_next_states_;
  int start_ = kInvalidIndex;
  int last_start_ = kInvalidIndex;
  int goal_ = kInvalidIndex;
  float key_modifier_ = 0;

  std::vector<int> neighbors_;
  std::vector<float> pivot_to_neighbor_costs_;
  std::vector<int> predecessors_;
};

template <class E>
PlannerStatus DStarLitePlanner<E>::PlanPathImpl(
    int start, int goal, const UserCallback& user_callback,
    std::vector<int>& path) {
  Reset();

  start_ = start;
  last_start_ = start;
  goal_ = goal;
  key_modifier_ = 0;

  rhs_values_[goal] = 0;
  open_queue_.emplace(goal, CalculateKey(goal));
  open_states_[goal] = true;

  if (auto status = CalculateShortestPath(user_callback);
      status != PlannerStatus::kSuccess) {
    return status;
  }
  return ReconstructShortestPath(user_callback, path);
}

template <class E>
void DStarLitePlanner<E>::Reset() {
  const auto state_space_size = this->env_->GetStateSpaceSize();
  open_queue_.clear();
  open_queue_.reserve(state_space_size);

  open_states_.resize(state_space_size, false);
  std::fill(open_states_.begin(), open_states_.end(), false);

  g_values_.resize(state_space_size, kInfCost);
  std::fill(g_values_.begin(), g_values_.end(), kInfCost);

  rhs_values_.resize(state_space_size, kInfCost);
  std::fill(rhs_values_.begin(), rhs_values_.end(), kInfCost);

  best_next_states_.resize(state_space_size, kInvalidIndex);
  std::fill(best_next_states_.begin(), best_next_states_.end(), kInvalidIndex);

  const auto max_num_neighbors = this->env_->GetMaxNumNeighbors();
  neighbors_.resize(max_num_neighbors, kInvalidIndex);
  std::fill(neighbors_.begin(), neighbors_.end(), kInvalidIndex);
  pivot_to_neighbor_costs_.resize(max_num_neighbors, kInfCost);
  std::fill(pivot_to_neighbor_costs_.begin(), pivot_to_neighbor_costs_.end(),
            kInfCost);

  predecessors_.resize(this->env_->GetMaxNumPredecessors(), kInvalidIndex);
  std::fill(predecessors_.begin(), predecessors_.end(), kInvalidIndex);
}

template <class E>
Key DStarLitePlanner<E>::CalculateKey(int index) const {
  const auto g_rhs_min = std::min(g_values_[index], rhs_values_[index]);
  return {
      g_rhs_min + this->env_->GetHeuristicCost(start_, index) + key_modifier_,
      g_rhs_min};
}

template <class E>
PlannerStatus DStarLitePlanner<E>::CalculateShortestPath(
    const UserCallback& user_callback) {
  // Each vertex can be expanded at most twice.
  const auto max_num_iterations = 2 * this->env_->GetStateSpaceSize();
  auto num_iterations = 0;
  const auto start_key = CalculateKey(start_);
  while (!open_queue_.empty() &&
         (open_queue_.top().key < start_key ||
          !AreEqual(rhs_values_[start_], g_values_[start_]))) {
    const auto pivot = open_queue_.top();
    open_queue_.pop();

    const auto pivot_index = pivot.index;
    if (!open_states_[pivot_index]) {
      continue;
    }
    open_states_[pivot_index] = false;

    if (user_callback && !user_callback()) {
      return PlannerStatus::kUserAbort;
    }

    const auto new_pivot_key = CalculateKey(pivot_index);
    if (pivot.key < new_pivot_key) {
      open_queue_.emplace(pivot_index, new_pivot_key);
      open_states_[pivot_index] = true;
    } else if (IsGreater(g_values_[pivot_index], rhs_values_[pivot_index])) {
      // Locally overconsistent case, the new path is better than the old one.
      g_values_[pivot_index] = rhs_values_[pivot_index];
      UpdatePredecessors(pivot_index);
    } else if (IsLess(g_values_[pivot_index], rhs_values_[pivot_index])) {
      // Locally underconsistent case, the new path is worse than the old one.
      g_values_[pivot_index] = kInfCost;
      UpdatePredecessors(pivot_index);
      UpdateVertex(pivot_index);
    }

    if (pivot_index == start_ &&
        AreEqual(g_values_[pivot_index], rhs_values_[pivot_index])) {
      break;
    }

    if (++num_iterations == max_num_iterations) {
      return PlannerStatus::kInfeasibleProblem;
    }
  }

  if (IsInf(g_values_[start_])) {
    return PlannerStatus::kInfeasibleProblem;
  }
  return PlannerStatus::kSuccess;
}

template <class E>
void DStarLitePlanner<E>::UpdateVertex(int pivot) {
  if (pivot == kInvalidIndex) {
    return;
  }
  if (pivot != goal_) {
    // TODO(mvukov) we need GetSuccessorsAndCosts here.
    this->env_->GetNeighborsAndCosts(pivot, neighbors_,
                                     pivot_to_neighbor_costs_);
    auto rhs = kInfCost;
    const auto max_num_neighbors = this->env_->GetMaxNumNeighbors();
    for (int el = 0; el < max_num_neighbors; ++el) {
      const auto successor = neighbors_[el];
      if (successor == kInvalidIndex) {
        continue;
      }
      auto successor_rhs = g_values_[successor] + pivot_to_neighbor_costs_[el];
      if (rhs > successor_rhs) {
        rhs = successor_rhs;
        best_next_states_[pivot] = successor;
      }
    }
    rhs_values_[pivot] = rhs;
  }

  if (!AreEqual(g_values_[pivot], rhs_values_[pivot])) {
    open_queue_.emplace(pivot, CalculateKey(pivot));
    open_states_[pivot] = true;
  } else {
    open_states_[pivot] = false;
  }
}

template <class E>
void DStarLitePlanner<E>::UpdatePredecessors(int pivot) {
  this->env_->GetPredecessors(pivot, predecessors_);
  for (auto predecessor : predecessors_) {
    UpdateVertex(predecessor);
  }
}

template <class E>
PlannerStatus DStarLitePlanner<E>::ReconstructShortestPath(
    const UserCallback& user_callback, std::vector<int>& path) {
  path.clear();
  path.push_back(start_);
  auto pivot = start_;
  while (pivot != goal_) {
    if (user_callback && !user_callback()) {
      return PlannerStatus::kUserAbort;
    }
    auto next_state = best_next_states_[pivot];
    if (next_state == kInvalidIndex) {
      return PlannerStatus::kInternalError;
    }
    path.push_back(next_state);
    pivot = next_state;
  }
  return PlannerStatus::kSuccess;
}

template <class E>
PlannerStatus DStarLitePlanner<E>::ReplanPathImpl(
    int start, const std::vector<int>& changed_states,
    const UserCallback& user_callback, std::vector<int>& path) {
  if (goal_ == kInvalidIndex) {
    return PlannerStatus::kInternalError;
  }

  if (changed_states.empty() && start_ == start) {
    return ReconstructShortestPath(user_callback, path);
  }

  if (!changed_states.empty() && start_ == start) {
    g_values_[start] = rhs_values_[start] = kInfCost;
  }

  for (auto state : changed_states) {
    UpdatePredecessors(state);
  }

  if (open_queue_.empty()) {
    open_queue_.emplace(goal_, Key{g_values_[goal_], rhs_values_[goal_]});
    open_states_[goal_] = true;
  }

  start_ = start;
  key_modifier_ += this->env_->GetHeuristicCost(last_start_, start);
  last_start_ = start;

  if (auto status = CalculateShortestPath(user_callback);
      status != PlannerStatus::kSuccess) {
    return status;
  }
  return ReconstructShortestPath(user_callback, path);
}

}  // namespace optimus

#endif  // OPTIMUS_PATH_PLANNING_DSTAR_LITE_PLANNER_H_
