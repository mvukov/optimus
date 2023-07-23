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
#ifndef OPTIMUS_PATH_PLANNING_ASTAR_SE2_PLANNER_H_
#define OPTIMUS_PATH_PLANNING_ASTAR_SE2_PLANNER_H_

#include <vector>

#include "optimus/path_planning/astar_planner.h"
#include "optimus/path_planning/se2_environment.h"
#include "optimus/path_planning/se2_planner.h"

namespace optimus {

using AStarSE2Planner = SE2Planner<AStarPlanner<SE2Environment>>;

}  // namespace optimus

#endif  // OPTIMUS_PATH_PLANNING_ASTAR_SE2_PLANNER_H_
