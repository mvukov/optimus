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
#ifndef OPTIMUS_PATH_PLANNING_DSTAR_LITE_SE2_PLANNER_H_
#define OPTIMUS_PATH_PLANNING_DSTAR_LITE_SE2_PLANNER_H_

#include <vector>

#include "optimus/path_planning/dstar_lite_planner.h"
#include "optimus/path_planning/se2_environment.h"
#include "optimus/path_planning/se2_planner.h"

namespace optimus {

using DStarLiteSE2Planner = SE2Planner<DStarLitePlanner<SE2Environment>>;

template <>
std::optional<float> DStarLiteSE2Planner::GetPathCost() const {
  if (!start_index_) {
    return std::nullopt;
  }
  return algorithm_.g_values().at(*start_index_);
}

}  // namespace optimus

#endif  // OPTIMUS_PATH_PLANNING_DSTAR_LITE_SE2_PLANNER_H_
