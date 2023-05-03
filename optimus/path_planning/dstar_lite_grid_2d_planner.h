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
#ifndef OPTIMUS_PATH_PLANNING_DSTAR_LITE_GRID_2D_PLANNER_H_
#define OPTIMUS_PATH_PLANNING_DSTAR_LITE_GRID_2D_PLANNER_H_

#include "optimus/path_planning/dstar_lite_planner.h"
#include "optimus/path_planning/grid_2d_environment.h"
#include "optimus/path_planning/grid_2d_planner.h"

namespace optimus {

using DStarLiteGrid2DPlanner =
    Grid2DPlanner<DStarLitePlanner<Grid2DEnvironment>>;

}  // namespace optimus

#endif  // OPTIMUS_PATH_PLANNING_DSTAR_LITE_GRID_2D_PLANNER_H_
