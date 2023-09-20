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
#include <unordered_set>
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
                               const std::unordered_set<int>& states_to_update,
                               const UserCallback& user_callback,
                               std::vector<int>& path);

  const auto& g_values() const { return g_values_; }

 private:
  void Reset();

  PlannerStatus CalculateShortestPath(const UserCallback& user_callback);
  Key CalculateKey(int index) const;
  void UpdateVertex(int pivot);
  void UpdatePredecessors(int pivot);
  PlannerStatus ReconstructShortestPath(const UserCallback& user_callback,
                                        std::vector<int>& path);

  PriorityQueue open_queue_;
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
  open_queue_.insert(goal, CalculateKey(goal));

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
  while (!open_queue_.empty() &&
         (open_queue_.top().key < CalculateKey(start_) ||
          !AreEqual(rhs_values_[start_], g_values_[start_]))) {
    if (user_callback && !user_callback(UserCallbackEvent::kSearch)) {
      return PlannerStatus::kUserAbort;
    }

    const auto pivot = open_queue_.top();
    const auto pivot_index = pivot.index;
    const auto new_pivot_key = CalculateKey(pivot_index);
    if (pivot.key < new_pivot_key) {
      // TODO(mvukov) Just update.
      open_queue_.InsertOrUpdate(pivot_index, new_pivot_key);
    } else {
      open_queue_.pop();
      if (IsGreater(g_values_[pivot_index], rhs_values_[pivot_index])) {
        // Locally overconsistent case, the new path is better than the old one.
        g_values_[pivot_index] = rhs_values_[pivot_index];
        UpdatePredecessors(pivot_index);
      } else {
        // Locally underconsistent case, the new path is worse than the old one.
        g_values_[pivot_index] = kInfCost;
        UpdateVertex(pivot_index);
        UpdatePredecessors(pivot_index);
      }
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
  if (pivot == goal_) {
    return;
  }

  // TODO(mvukov) we need GetSuccessorsAndCosts here.
  this->env_->GetNeighborsAndCosts(pivot, neighbors_, pivot_to_neighbor_costs_);
  auto min_rhs = kInfCost;
  auto min_rhs_successor = kInvalidIndex;
  const auto max_num_neighbors = this->env_->GetMaxNumNeighbors();
  for (int el = 0; el < max_num_neighbors; ++el) {
    const auto successor = neighbors_[el];
    if (successor == kInvalidIndex) {
      continue;
    }
    auto successor_rhs = g_values_[successor] + pivot_to_neighbor_costs_[el];
    if (min_rhs > successor_rhs) {
      min_rhs = successor_rhs;
      min_rhs_successor = successor;
    }
  }
  rhs_values_[pivot] = min_rhs;
  best_next_states_[pivot] = min_rhs_successor;

  if (!AreEqual(g_values_[pivot], rhs_values_[pivot])) {
    open_queue_.InsertOrUpdate(pivot, CalculateKey(pivot));
  } else {
    open_queue_.Remove(pivot);
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
    if (user_callback && !user_callback(UserCallbackEvent::kReconstruction)) {
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
    int start, const std::unordered_set<int>& states_to_update,
    const UserCallback& user_callback, std::vector<int>& path) {
  if (goal_ == kInvalidIndex) {
    return PlannerStatus::kInternalError;
  }

  if (states_to_update.empty() && start_ == start) {
    return ReconstructShortestPath(user_callback, path);
  }

  for (auto state : states_to_update) {
    UpdateVertex(state);
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
