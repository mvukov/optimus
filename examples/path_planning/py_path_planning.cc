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
#include <chrono>
#include <iostream>

#include "pybind11/eigen.h"
#include "pybind11/pybind11.h"
#include "pybind11/stl.h"

#include "optimus/path_planning/action_set_2d.h"
#include "optimus/path_planning/astar_grid_2d_planner.h"
#include "optimus/path_planning/astar_se2_planner.h"
#include "optimus/path_planning/dstar_lite_grid_2d_planner.h"
#include "optimus/path_planning/dstar_lite_se2_planner.h"
#include "optimus/path_planning/se2_environment.h"

namespace py = pybind11;

namespace optimus {

// This is an auto-generated function.
const ActionSet2D& get_example_primitives();

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
  py::array_t<Grid2DScalar> grid_2d_;
  std::unique_ptr<Grid2DMap> grid_2d_map_;
};

// Implements an example SE2 (lattice) planner.
// Look at the build file to learn how to set up code-generation of motion
// primitives.
template <class Planner>
class ExampleSE2Planner : public PyPlanner<ExampleSE2Planner<Planner>> {
 public:
  using Pose2D = SE2PlannerBase::Pose2D;

  // Note here how the code-generated primitives g_example_primitives are
  // interfaced to the planner implementation.
  explicit ExampleSE2Planner(const SE2Environment::Config& config)
      : planner_(config, &get_example_primitives()) {}

  // This implementation is Python specific where error handling is handled with
  // exceptions -- in C++ its handled with status codes.
  std::vector<Pose2D> PyPlanPath(const Pose2D& start, const Pose2D& goal) {
    std::vector<Pose2D> path;
    planning_time_ = 0;
    num_expansions_ = 0;
    auto callback = [this](UserCallbackEvent event) {
      if (PyErr_CheckSignals() != 0) {
        throw py::error_already_set();
      }
      if (event == UserCallbackEvent::kSearch) {
        ++num_expansions_;
      }
      return true;
    };
    const auto start_timestamp = std::chrono::steady_clock::now();
    if (auto status = planner_.PlanPath(start, goal, callback, path);
        status != PlannerStatus::kSuccess) {
      throw std::runtime_error("Failed to plan a path! Reason: " +
                               ToString(status));
    }
    planning_time_ = std::chrono::duration<double>(
                         std::chrono::steady_clock::now() - start_timestamp)
                         .count();
    return path;
  }

  std::vector<Pose2D> ReplanPath(
      const Pose2D& start, const std::vector<Position2D>& changed_positions) {
    std::vector<Pose2D> path;
    planning_time_ = 0;
    num_expansions_ = 0;
    auto callback = [this](UserCallbackEvent event) {
      if (event == UserCallbackEvent::kSearch) {
        ++num_expansions_;
      }
      return true;
    };
    const auto start_timestamp = std::chrono::steady_clock::now();
    if (auto status =
            planner_.ReplanPath(start, changed_positions, callback, path);
        status != PlannerStatus::kSuccess) {
      throw std::runtime_error("Failed to replan a path! Reason: " +
                               ToString(status));
    }
    planning_time_ = std::chrono::duration<double>(
                         std::chrono::steady_clock::now() - start_timestamp)
                         .count();
    return path;
  }

  auto planning_time() const { return planning_time_; }
  auto num_expansions() const { return num_expansions_; }
  auto path_cost() const { return planner_.GetPathCost(); }

  auto* mutable_planner() { return &planner_; }

 private:
  Planner planner_;
  double planning_time_ = 0;
  int num_expansions_ = 0;
};

using ExampleAStarSE2Planner = ExampleSE2Planner<AStarSE2Planner>;
using ExampleDStarLiteSE2Planner = ExampleSE2Planner<DStarLiteSE2Planner>;

template <class Planner>
class PyGrid2DPlanner : public PyPlanner<PyGrid2DPlanner<Planner>> {
 public:
  explicit PyGrid2DPlanner(const Grid2DEnvironment::Config& config)
      : planner_(config) {}

  std::vector<Position2D> PlanPath(const Position2D& start,
                                   const Position2D& goal) {
    std::vector<Position2D> path;
    planning_time_ = 0;
    num_expansions_ = 0;
    auto callback = [this](UserCallbackEvent event) {
      if (PyErr_CheckSignals() != 0) {
        throw py::error_already_set();
      }
      if (event == UserCallbackEvent::kSearch) {
        ++num_expansions_;
      }
      return true;
    };
    const auto start_timestamp = std::chrono::steady_clock::now();
    if (auto status = planner_.PlanPath(start, goal, callback, path);
        status != PlannerStatus::kSuccess) {
      throw std::runtime_error("Failed to plan a path! Reason: " +
                               ToString(status));
    }
    planning_time_ = std::chrono::duration<double>(
                         std::chrono::steady_clock::now() - start_timestamp)
                         .count();
    return path;
  }

  std::vector<Position2D> ReplanPath(
      const Position2D& start,
      const std::vector<Position2D>& changed_positions) {
    std::vector<Position2D> path;
    planning_time_ = 0;
    num_expansions_ = 0;
    auto callback = [this](UserCallbackEvent event) {
      if (event == UserCallbackEvent::kSearch) {
        ++num_expansions_;
      }
      return true;
    };
    const auto start_timestamp = std::chrono::steady_clock::now();
    if (auto status =
            planner_.ReplanPath(start, changed_positions, callback, path);
        status != PlannerStatus::kSuccess) {
      throw std::runtime_error("Failed to replan a path! Reason: " +
                               ToString(status));
    }
    planning_time_ = std::chrono::duration<double>(
                         std::chrono::steady_clock::now() - start_timestamp)
                         .count();
    return path;
  }

  auto planning_time() const { return planning_time_; }
  auto num_expansions() const { return num_expansions_; }
  auto path_cost() const { return planner_.GetPathCost(); }

  auto* mutable_planner() { return &planner_; }

 private:
  Planner planner_;
  double planning_time_ = 0;
  int num_expansions_ = 0;
};

using PyAStarGrid2DPlanner = PyGrid2DPlanner<AStarGrid2DPlanner>;
using PyDStarLiteGrid2DPlanner = PyGrid2DPlanner<DStarLiteGrid2DPlanner>;

PYBIND11_MODULE(py_path_planning, m) {
  auto se2_environment = py::class_<SE2Environment>(m, "SE2Environment");

  py::class_<SE2Environment::Config>(se2_environment, "Config")
      .def(py::init<>())
      .def_readwrite("valid_state_threshold",
                     &SE2Environment::Config::valid_state_threshold)
      .def_readwrite("swath_cost_multiplier",
                     &SE2Environment::Config::swath_cost_multiplier)
      .def_readwrite("length_cost_multiplier",
                     &SE2Environment::Config::length_cost_multiplier)
      .def_readwrite("abs_angle_diff_cost_multiplier",
                     &SE2Environment::Config::abs_angle_diff_cost_multiplier);

  py::class_<ExampleAStarSE2Planner>(m, "ExampleAStarSE2Planner")
      .def(py::init<const SE2Environment::Config&>(), py::arg("config"))
      .def("plan_path", &ExampleAStarSE2Planner::PyPlanPath)
      .def("replan_path", &ExampleAStarSE2Planner::ReplanPath)
      .def("set_grid_2d", &ExampleAStarSE2Planner::SetGrid2D)
      .def_property_readonly("planning_time",
                             &ExampleAStarSE2Planner::planning_time)
      .def_property_readonly("num_expansions",
                             &ExampleAStarSE2Planner::num_expansions)
      .def_property_readonly("path_cost", &ExampleAStarSE2Planner::path_cost);

  py::class_<ExampleDStarLiteSE2Planner>(m, "ExampleDStarLiteSE2Planner")
      .def(py::init<const SE2Environment::Config&>(), py::arg("config"))
      .def("plan_path", &ExampleDStarLiteSE2Planner::PyPlanPath)
      .def("replan_path", &ExampleDStarLiteSE2Planner::ReplanPath)
      .def("set_grid_2d", &ExampleDStarLiteSE2Planner::SetGrid2D)
      .def_property_readonly("planning_time",
                             &ExampleDStarLiteSE2Planner::planning_time)
      .def_property_readonly("num_expansions",
                             &ExampleDStarLiteSE2Planner::num_expansions)
      .def_property_readonly("path_cost",
                             &ExampleDStarLiteSE2Planner::path_cost);

  py::class_<SE2PlannerBase::Pose2D>(m, "Pose2D")
      .def(py::init<>())
      .def(py::init<float, float, float>())
      .def_readwrite("x", &AStarSE2Planner::Pose2D::x)
      .def_readwrite("y", &AStarSE2Planner::Pose2D::y)
      .def_readwrite("theta", &AStarSE2Planner::Pose2D::theta);

  auto grid_2d_environment =
      py::class_<Grid2DEnvironment>(m, "Grid2DEnvironment");

  py::class_<Grid2DEnvironment::Config>(grid_2d_environment, "Config")
      .def(py::init<>())
      .def_readwrite("valid_state_threshold",
                     &Grid2DEnvironment::Config::valid_state_threshold);

  py::class_<PyAStarGrid2DPlanner>(m, "AStarGrid2DPlanner")
      .def(py::init<const Grid2DEnvironment::Config&>(), py::arg("config"))
      .def("plan_path", &PyAStarGrid2DPlanner::PlanPath)
      .def("replan_path", &PyAStarGrid2DPlanner::ReplanPath)
      .def("set_grid_2d", &PyAStarGrid2DPlanner::SetGrid2D)
      .def_property_readonly("planning_time",
                             &PyAStarGrid2DPlanner::planning_time)
      .def_property_readonly("num_expansions",
                             &PyAStarGrid2DPlanner::num_expansions)
      .def_property_readonly("path_cost", &PyAStarGrid2DPlanner::path_cost);

  py::class_<PyDStarLiteGrid2DPlanner>(m, "DStarLiteGrid2DPlanner")
      .def(py::init<const Grid2DEnvironment::Config&>(), py::arg("config"))
      .def("plan_path", &PyDStarLiteGrid2DPlanner::PlanPath)
      .def("replan_path", &PyDStarLiteGrid2DPlanner::ReplanPath)
      .def("set_grid_2d", &PyDStarLiteGrid2DPlanner::SetGrid2D)
      .def_property_readonly("planning_time",
                             &PyDStarLiteGrid2DPlanner::planning_time)
      .def_property_readonly("num_expansions",
                             &PyDStarLiteGrid2DPlanner::num_expansions)
      .def_property_readonly("path_cost", &PyDStarLiteGrid2DPlanner::path_cost);
}

}  // namespace optimus
