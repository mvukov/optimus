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
#include "optimus/path_planning/arastar_planner.h"

#include "optimus/debug.h"
#include "optimus/path_planning/common_utils.h"

namespace optimus {

bool AraStarPlannerConfig::Validate() const {
  if (IsLess(epsilon_start, 1.0)) {
    OPTIMUS_PRINT("epsilon_start must be >= 1!");
    return false;
  }
  if (!IsGreater(epsilon_decrease_rate, 0)) {
    OPTIMUS_PRINT("epsilon_decrease_rate must be > 0!");
    return false;
  }
  return true;
}

}  // namespace optimus
