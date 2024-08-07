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
#ifndef OPTIMUS_PATH_PLANNING_PLANNER_ALGORITHM_H_
#define OPTIMUS_PATH_PLANNING_PLANNER_ALGORITHM_H_

#include <functional>
#include <string>
#include <unordered_set>
#include <vector>

namespace optimus {

enum class [[nodiscard]] PlannerStatus {
  kUnknown,
  kSuccess,
  kInfeasibleProblem,
  kInternalError,
  kUserAbort,
  kNotImplemented,
};

enum class UserCallbackEvent {
  kSearch,
  kReconstruction,
  kSolutionFound,
};

using UserCallback = std::function<bool(UserCallbackEvent)>;

std::string ToString(PlannerStatus status);

template <class Derived, class Environment>
class PlannerAlgorithm {
 public:
  explicit PlannerAlgorithm(Environment* env) : env_(env) {}

  PlannerStatus PlanPath(int start, int goal, const UserCallback& user_callback,
                         std::vector<int>& path);

  PlannerStatus ReplanPath(int start,
                           const std::unordered_set<int>& states_to_update,
                           const UserCallback& user_callback,
                           std::vector<int>& path);

 protected:
  Environment* env_;

 private:
  PlannerStatus Validate(int start, int* goal) const;
};

template <class D, class E>
PlannerStatus PlannerAlgorithm<D, E>::PlanPath(
    int start, int goal, const UserCallback& user_callback,
    std::vector<int>& path) {
  if (auto status = Validate(start, &goal); status != PlannerStatus::kSuccess) {
    return status;
  }
  if (!env_->Initialize()) {
    return PlannerStatus::kInternalError;
  }
  if (start == goal) {
    path = {start, start};
    return PlannerStatus::kSuccess;
  }
  return static_cast<D*>(this)->PlanPathImpl(start, goal, user_callback, path);
}

template <class D, class E>
PlannerStatus PlannerAlgorithm<D, E>::ReplanPath(
    int start, const std::unordered_set<int>& states_to_update,
    const UserCallback& user_callback, std::vector<int>& path) {
  if (auto status = Validate(start, nullptr);
      status != PlannerStatus::kSuccess) {
    return status;
  }
  return static_cast<D*>(this)->ReplanPathImpl(start, states_to_update,
                                               user_callback, path);
}

template <class D, class E>
PlannerStatus PlannerAlgorithm<D, E>::Validate(int start, int* goal) const {
  // TODO(mvukov) Introduce more advanced status codes!
  if (env_ == nullptr || !env_->Validate()) {
    return PlannerStatus::kInternalError;
  }
  if (!env_->IsStateIndexValid(start) || !env_->IsStateValid(start)) {
    return PlannerStatus::kInternalError;
  }
  if (goal != nullptr &&
      (!env_->IsStateIndexValid(*goal) || !env_->IsStateValid(*goal))) {
    return PlannerStatus::kInternalError;
  }
  return PlannerStatus::kSuccess;
}

}  // namespace optimus

#endif  // OPTIMUS_PATH_PLANNING_PLANNER_ALGORITHM_H_
