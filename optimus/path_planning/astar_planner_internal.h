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
#ifndef OPTIMUS_PATH_PLANNING_ASTAR_PLANNER_INTERNAL_H_
#define OPTIMUS_PATH_PLANNING_ASTAR_PLANNER_INTERNAL_H_

#include <vector>

#include "boost/unordered/unordered_flat_map.hpp"

#include "optimus/path_planning/planner_algorithm.h"

namespace optimus::internal {

PlannerStatus ReconstructShortestPath(
    int goal, const UserCallback& user_callback,
    boost::unordered::unordered_flat_map<int, int> indices_to_parent_indices,
    std::vector<int>& path);

}  // namespace optimus::internal

#endif  // OPTIMUS_PATH_PLANNING_ASTAR_PLANNER_INTERNAL_H_
