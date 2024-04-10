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
#include "pybind11/pybind11.h"

#include "optimus/path_planning/planner_algorithm.h"

namespace py = pybind11;

namespace optimus {

void initialize_grid_2d_planners(py::module_& m);
void initialize_se2_planners(py::module_& m);

PYBIND11_MODULE(py_path_planning, m) {
  py::enum_<UserCallbackEvent>(m, "UserCallbackEvent")
      .value("SEARCH", UserCallbackEvent::kSearch)
      .value("RECONSTRUCTION", UserCallbackEvent::kReconstruction)
      .value("SOLUTION_FOUND", UserCallbackEvent::kSolutionFound)
      .export_values();

  initialize_grid_2d_planners(m);
  initialize_se2_planners(m);
}

}  // namespace optimus
