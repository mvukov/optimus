// Copyright 2024 Milan Vukov. All rights reserved.
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
#ifndef OPTIMUS_PATH_PLANNING_ARASTAR_PLANNER_H_
#define OPTIMUS_PATH_PLANNING_ARASTAR_PLANNER_H_
#include <algorithm>
#include <unordered_set>
#include <vector>

#include "boost/unordered/unordered_flat_map.hpp"
#include "boost/unordered/unordered_flat_set.hpp"

#include "optimus/path_planning/astar_planner_internal.h"
#include "optimus/path_planning/planner_algorithm.h"
#include "optimus/path_planning/priority_queue_utils.h"

namespace optimus {

struct AraStarPlannerConfig {
  float epsilon_start = 3.;
  float epsilon_decrease_rate = 0.5;

  bool Validate() const;
};

// An implementation of ARA* (Anytime Repairing A*) algorithm.
template <class Environment>
class AraStarPlanner
    : public PlannerAlgorithm<AraStarPlanner<Environment>, Environment> {
  using Base = PlannerAlgorithm<AraStarPlanner<Environment>, Environment>;

 public:
  AraStarPlanner(Environment* env, const AraStarPlannerConfig& config)
      : Base(env), config_(config) {}

  PlannerStatus PlanPathImpl(int start, int goal,
                             const UserCallback& user_callback,
                             std::vector<int>& path);

  PlannerStatus ReplanPathImpl(
      int /*start*/, const std::unordered_set<int>& /*states_to_update*/,
      const UserCallback& user_callback, std::vector<int>& path);

  auto GetGValue(int index) const { return indices_to_g_values_.at(index); }

  auto epsilon() const { return epsilon_; }

 private:
  void Reset();
  Key ComputeKey(int index) const;
  PlannerStatus RunPathImprovementLoop(const UserCallback& user_callback,
                                       std::vector<int>& path);
  PlannerStatus ImprovePath(const UserCallback& user_callback);

  AraStarPlannerConfig config_;

  PriorityQueue open_queue_;
  boost::unordered::unordered_flat_set<int> inconsistent_indices_;

  boost::unordered::unordered_flat_set<int> closed_indices_;
  std::vector<bool> visited_indices_;

  // It is way faster to have those below as std::vector at the expense of
  // increased memory consumption.
  boost::unordered::unordered_flat_map<int, int> indices_to_parent_indices_;
  boost::unordered::unordered_flat_map<int, float> indices_to_g_values_;

  int goal_ = kInvalidIndex;
  float epsilon_ = 1.;
  std::vector<int> neighbors_;
  std::vector<float> pivot_to_neighbor_costs_;
};

template <class E>
PlannerStatus AraStarPlanner<E>::PlanPathImpl(int start, int goal,
                                              const UserCallback& user_callback,
                                              std::vector<int>& path) {
  Reset();

  goal_ = goal;
  epsilon_ = config_.epsilon_start;

  indices_to_g_values_[goal] = kInfCost;
  indices_to_g_values_[start] = 0;
  open_queue_.insert(start, ComputeKey(start));

  indices_to_parent_indices_[start] = start;
  visited_indices_[start] = true;

  if (auto status = ImprovePath(user_callback);
      status != PlannerStatus::kSuccess) {
    return status;
  }
  if (user_callback && !user_callback(UserCallbackEvent::kSolutionFound)) {
    return internal::ReconstructShortestPath(goal_, user_callback,
                                             indices_to_parent_indices_, path);
  }
  return RunPathImprovementLoop(user_callback, path);
}

template <class E>
PlannerStatus AraStarPlanner<E>::ReplanPathImpl(
    int /*start*/, const std::unordered_set<int>& /*states_to_update*/,
    const UserCallback& user_callback, std::vector<int>& path) {
  if (goal_ == kInvalidIndex) {
    return PlannerStatus::kInternalError;
  }
  return RunPathImprovementLoop(user_callback, path);
}

template <class E>
void AraStarPlanner<E>::Reset() {
  const auto state_space_size = this->env_->GetStateSpaceSize();
  open_queue_.clear();
  inconsistent_indices_.clear();
  closed_indices_.clear();
  closed_indices_.reserve(state_space_size);
  visited_indices_.clear();
  visited_indices_.resize(state_space_size, false);
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
Key AraStarPlanner<E>::ComputeKey(int index) const {
  const auto g_value = indices_to_g_values_.at(index);
  return {g_value + epsilon_ * this->env_->GetHeuristicCost(index, goal_),
          g_value};
}

template <class E>
PlannerStatus AraStarPlanner<E>::RunPathImprovementLoop(
    const UserCallback& user_callback, std::vector<int>& path) {
  constexpr float kMinEpsilon = 1.f;
  while (IsGreater(epsilon_, kMinEpsilon)) {
    epsilon_ = std::max(kMinEpsilon, epsilon_ - config_.epsilon_decrease_rate);
    for (auto index : open_queue_.GetIndices()) {
      open_queue_.InsertOrUpdate(index, ComputeKey(index));
    }
    for (auto index : inconsistent_indices_) {
      open_queue_.InsertOrUpdate(index, ComputeKey(index));
    }
    inconsistent_indices_.clear();
    closed_indices_.clear();

    if (auto status = ImprovePath(user_callback);
        status == PlannerStatus::kSuccess) {
      if (user_callback && !user_callback(UserCallbackEvent::kSolutionFound)) {
        break;
      }
    } else {
      return status;
    }
  }
  return internal::ReconstructShortestPath(goal_, user_callback,
                                           indices_to_parent_indices_, path);
}

template <class E>
PlannerStatus AraStarPlanner<E>::ImprovePath(
    const UserCallback& user_callback) {
  while (!open_queue_.empty() && ComputeKey(goal_) > open_queue_.top().key) {
    if (user_callback && !user_callback(UserCallbackEvent::kSearch)) {
      return PlannerStatus::kUserAbort;
    }

    const auto pivot = open_queue_.top();
    open_queue_.pop();

    const auto pivot_index = pivot.index;
    closed_indices_.insert(pivot_index);

    this->env_->GetNeighborsAndCosts(pivot_index, neighbors_,
                                     pivot_to_neighbor_costs_);
    const auto max_num_neighbors = this->env_->GetMaxNumNeighbors();
    for (int el = 0; el < max_num_neighbors; ++el) {
      const auto neighbor = neighbors_[el];
      if (neighbor == kInvalidIndex) {
        continue;
      }

      if (!visited_indices_[neighbor]) {
        indices_to_g_values_[neighbor] = kInfCost;
        indices_to_parent_indices_[neighbor] = kInvalidIndex;
        visited_indices_[neighbor] = true;
      }

      const auto new_g_value_ =
          indices_to_g_values_[pivot_index] + pivot_to_neighbor_costs_[el];
      if (new_g_value_ < indices_to_g_values_[neighbor]) {
        if (closed_indices_.count(neighbor) == 0) {
          indices_to_g_values_[neighbor] = new_g_value_;
          indices_to_parent_indices_[neighbor] = pivot_index;
          open_queue_.InsertOrUpdate(neighbor, ComputeKey(neighbor));
        } else {
          inconsistent_indices_.insert(neighbor);
        }
      }
    }
  }
  if (IsInf(indices_to_g_values_[goal_])) {
    return PlannerStatus::kInfeasibleProblem;
  }
  return PlannerStatus::kSuccess;
}

}  // namespace optimus

#endif  // OPTIMUS_PATH_PLANNING_ARASTAR_PLANNER_H_
