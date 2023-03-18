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

#include "pybind11/eigen.h"
#include "pybind11/pybind11.h"
#include "pybind11/stl.h"

#include "optimus/path_planning/action_set_2d.h"
#include "optimus/path_planning/astar_grid_2d_planner.h"
#include "optimus/path_planning/astar_se2_planner.h"
#include "optimus/path_planning/se2_environment.h"

namespace py = pybind11;

namespace optimus {

// This is an auto-generated function.
const ActionSet2D& get_example_primitives();

// Implements an example SE2 (lattice) planner.
// Look at the build file to learn how to set up code-generation of motion
// primitives.
class ExampleAStarSE2Planner {
 public:
  using Pose2D = AStarSE2Planner::Pose2D;

  // Note here how the code-generated primitives g_example_primitives are
  // interfaced to the planner implementation.
  explicit ExampleAStarSE2Planner(const SE2Environment::Config& config)
      : planner_(config, &get_example_primitives()) {}

  // This implementation is Python specific where error handling is handled with
  // exceptions -- in C++ its handled with status codes.
  std::vector<Pose2D> PyPlanPath(
      const SE2Environment::ObstacleData& obstacle_data, const Pose2D& start,
      const Pose2D& goal) {
    if (!planner_.SetObstacleData(&obstacle_data)) {
      throw std::runtime_error("Failed to set obstacle data!");
    }
    std::vector<Pose2D> path;

    planning_time_ = 0;
    num_expansions_ = 0;
    auto callback = [this]() {
      ++num_expansions_;
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
    --num_expansions_;  // This is the exit in fact.
    return path;
  }

  auto planning_time() const { return planning_time_; }
  auto num_expansions() const { return num_expansions_; }

 private:
  AStarSE2Planner planner_;
  double planning_time_ = 0;
  int num_expansions_ = 0;
};

class PyAStarGrid2DPlanner {
 public:
  using Position = AStarGrid2DPlanner::Position;

  explicit PyAStarGrid2DPlanner(const Grid2DEnvironment::Config& config)
      : planner_(config) {}

  std::vector<Position> PyPlanPath(
      const Grid2DEnvironment::ObstacleData& obstacle_data,
      const Position& start, const Position& goal) {
    if (!planner_.SetObstacleData(&obstacle_data)) {
      throw std::runtime_error("Failed to set obstacle data!");
    }
    std::vector<Position> path;

    planning_time_ = 0;
    num_expansions_ = 0;
    auto callback = [this]() {
      ++num_expansions_;
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
    --num_expansions_;
    return path;
  }

  auto planning_time() const { return planning_time_; }
  auto num_expansions() const { return num_expansions_; }

 private:
  AStarGrid2DPlanner planner_;
  double planning_time_ = 0;
  int num_expansions_ = 0;
};

PYBIND11_MODULE(py_path_planning, m) {
  py::class_<MotionPrimitive2D>(m, "MotionPrimitive2D")
      .def(py::init<>())
      .def_readwrite("length", &MotionPrimitive2D::length)
      .def_readwrite("abs_angle_diff", &MotionPrimitive2D::abs_angle_diff)
      .def_readwrite("x", &MotionPrimitive2D::x)
      .def_readwrite("y", &MotionPrimitive2D::y)
      .def_readwrite("theta", &MotionPrimitive2D::theta)
      .def_readwrite("swath_x", &MotionPrimitive2D::swath_x)
      .def_readwrite("swath_y", &MotionPrimitive2D::swath_y)
      .def_readwrite("end_angle_idx", &MotionPrimitive2D::end_angle_idx)
      .def_readwrite("end_x_idx", &MotionPrimitive2D::end_x_idx)
      .def_readwrite("end_y_idx", &MotionPrimitive2D::end_y_idx);

  py::class_<ActionSet2D>(m, "ActionSet2D")
      .def(py::init<>())
      .def_readwrite("angles", &ActionSet2D::angles)
      .def_readwrite("motion_primitives", &ActionSet2D::motion_primitives);

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

  auto astar_se2_planner =
      py::class_<ExampleAStarSE2Planner>(m, "ExampleAStarSE2Planner")
          .def(py::init<const SE2Environment::Config&>(), py::arg("config"))
          .def("plan_path", &ExampleAStarSE2Planner::PyPlanPath)
          .def_property_readonly("planning_time",
                                 &ExampleAStarSE2Planner::planning_time)
          .def_property_readonly("num_expansions",
                                 &ExampleAStarSE2Planner::num_expansions);

  py::class_<AStarSE2Planner::Pose2D>(astar_se2_planner, "Pose2D")
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

  auto astar_grid_2d_planner =
      py::class_<PyAStarGrid2DPlanner>(m, "AStarGrid2DPlanner")
          .def(py::init<const Grid2DEnvironment::Config&>(), py::arg("config"))
          .def("plan_path", &PyAStarGrid2DPlanner::PyPlanPath)
          .def_property_readonly("planning_time",
                                 &PyAStarGrid2DPlanner::planning_time)
          .def_property_readonly("num_expansions",
                                 &PyAStarGrid2DPlanner::num_expansions);
}

}  // namespace optimus
