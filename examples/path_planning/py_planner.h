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
#ifndef EXAMPLES_PATH_PLANNING_PY_PLANNER_H_
#define EXAMPLES_PATH_PLANNING_PY_PLANNER_H_
#include <memory>

#include "pybind11/eigen.h"
#include "pybind11/pybind11.h"
#include "pybind11/stl.h"

#include "optimus/path_planning/grid_2d.h"

namespace py = pybind11;

namespace optimus {

using PyGrid2D =
    py::array_t<Grid2DScalar, py::array::c_style | py::array::forcecast>;

template <class Derived>
class PyPlanner {
 public:
  void SetGrid2D(PyGrid2D grid_2d) {
    if (grid_2d.ndim() != 2 || grid_2d.shape(0) == 0 || grid_2d.shape(1) == 0) {
      throw py::value_error(
          "The number of dimensions must be 2 and each dimension must be > 0!");
    }
    grid_2d_ = grid_2d;
    grid_2d_map_ = std::make_unique<Grid2DMap>(
        grid_2d_.data(0, 0), grid_2d_.shape(0), grid_2d_.shape(1));
    if (!static_cast<Derived*>(this)->mutable_planner()->SetGrid2D(
            grid_2d_map_.get())) {
      throw std::runtime_error("Failed to set grid!");
    }
  }

 protected:
  PyGrid2D grid_2d_;
  std::unique_ptr<Grid2DMap> grid_2d_map_;
};

}  // namespace optimus

#endif  // EXAMPLES_PATH_PLANNING_PY_PLANNER_H_
