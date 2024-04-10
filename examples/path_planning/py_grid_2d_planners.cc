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

#include "pybind11/functional.h"

#include "examples/path_planning/py_planner.h"
#include "optimus/path_planning/arastar_grid_2d_planner.h"
#include "optimus/path_planning/astar_grid_2d_planner.h"
#include "optimus/path_planning/dstar_lite_grid_2d_planner.h"
#include "optimus/path_planning/grid_2d_environment.h"

namespace optimus {

template <class Planner>
class PyGrid2DPlanner : public PyPlanner<PyGrid2DPlanner<Planner>> {
 public:
  template <typename... Args>
  explicit PyGrid2DPlanner(const Grid2DEnvironment::Config& config,
                           Args&&... args)
      : planner_(config, std::forward<Args>(args)...) {}

  std::vector<Position2D> PlanPath(
      const Position2D& start, const Position2D& goal,
      const std::optional<UserCallback>& user_callback) {
    std::vector<Position2D> path;
    planning_time_ = 0;
    num_expansions_ = 0;

    auto callback = [this, &user_callback](UserCallbackEvent event) {
      if (PyErr_CheckSignals() != 0) {
        throw py::error_already_set();
      }
      if (event == UserCallbackEvent::kSearch) {
        ++num_expansions_;
      }
      if (user_callback != std::nullopt) {
        return user_callback.value()(event);
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
      const Position2D& start, const std::vector<Position2D>& changed_positions,
      const std::optional<UserCallback>& user_callback) {
    std::vector<Position2D> path;
    planning_time_ = 0;
    num_expansions_ = 0;
    auto callback = [this, &user_callback](UserCallbackEvent event) {
      if (event == UserCallbackEvent::kSearch) {
        ++num_expansions_;
      }
      if (user_callback != std::nullopt) {
        return user_callback.value()(event);
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
  auto epsilon() const { return planner_.epsilon(); }

  auto* mutable_planner() { return &planner_; }

 private:
  Planner planner_;
  double planning_time_ = 0;
  int num_expansions_ = 0;
};

using PyAStarGrid2DPlanner = PyGrid2DPlanner<AStarGrid2DPlanner>;
using PyDStarLiteGrid2DPlanner = PyGrid2DPlanner<DStarLiteGrid2DPlanner>;
using PyAraStarGrid2DPlanner = PyGrid2DPlanner<AraStarGrid2DPlanner>;

void initialize_grid_2d_planners(py::module_& m) {
  auto grid_2d_environment =
      py::class_<Grid2DEnvironment>(m, "Grid2DEnvironment");

  py::class_<Grid2DEnvironment::Config>(grid_2d_environment, "Config")
      .def(py::init<>())
      .def_readwrite("valid_state_threshold",
                     &Grid2DEnvironment::Config::valid_state_threshold);

  py::class_<PyAStarGrid2DPlanner>(m, "AStarGrid2DPlanner")
      .def(py::init<const Grid2DEnvironment::Config&>(), py::arg("config"))
      .def("plan_path", &PyAStarGrid2DPlanner::PlanPath, py::arg("start"),
           py::arg("goal"), py::arg("user_callback") = std::nullopt)
      .def("replan_path", &PyAStarGrid2DPlanner::ReplanPath, py::arg("start"),
           py::arg("changed_positions"),
           py::arg("user_callback") = std::nullopt)
      .def("set_grid_2d", &PyAStarGrid2DPlanner::SetGrid2D)
      .def_property_readonly("planning_time",
                             &PyAStarGrid2DPlanner::planning_time)
      .def_property_readonly("num_expansions",
                             &PyAStarGrid2DPlanner::num_expansions)
      .def_property_readonly("path_cost", &PyAStarGrid2DPlanner::path_cost);

  py::class_<PyDStarLiteGrid2DPlanner>(m, "DStarLiteGrid2DPlanner")
      .def(py::init<const Grid2DEnvironment::Config&>(), py::arg("config"))
      .def("plan_path", &PyDStarLiteGrid2DPlanner::PlanPath, py::arg("start"),
           py::arg("goal"), py::arg("user_callback") = std::nullopt)
      .def("replan_path", &PyDStarLiteGrid2DPlanner::ReplanPath,
           py::arg("start"), py::arg("changed_positions"),
           py::arg("user_callback") = std::nullopt)
      .def("set_grid_2d", &PyDStarLiteGrid2DPlanner::SetGrid2D)
      .def_property_readonly("planning_time",
                             &PyDStarLiteGrid2DPlanner::planning_time)
      .def_property_readonly("num_expansions",
                             &PyDStarLiteGrid2DPlanner::num_expansions)
      .def_property_readonly("path_cost", &PyDStarLiteGrid2DPlanner::path_cost);

  py::class_<AraStarPlannerConfig>(m, "AraStarPlannerConfig")
      .def(py::init<>())
      .def_readwrite("epsilon_start", &AraStarPlannerConfig::epsilon_start)
      .def_readwrite("epsilon_decrease_rate",
                     &AraStarPlannerConfig::epsilon_decrease_rate)
      .def("validate", &AraStarPlannerConfig::Validate);

  py::class_<PyAraStarGrid2DPlanner>(m, "AraStarGrid2DPlanner")
      .def(py::init<const Grid2DEnvironment::Config&,
                    const AraStarPlannerConfig&>(),
           py::arg("env_config"), py::arg("planner_config"))
      .def("plan_path", &PyAraStarGrid2DPlanner::PlanPath, py::arg("start"),
           py::arg("goal"), py::arg("user_callback") = std::nullopt)
      .def("replan_path", &PyAraStarGrid2DPlanner::ReplanPath, py::arg("start"),
           py::arg("changed_positions"),
           py::arg("user_callback") = std::nullopt)
      .def("set_grid_2d", &PyAraStarGrid2DPlanner::SetGrid2D)
      .def_property_readonly("planning_time",
                             &PyAraStarGrid2DPlanner::planning_time)
      .def_property_readonly("num_expansions",
                             &PyAraStarGrid2DPlanner::num_expansions)
      .def_property_readonly("path_cost", &PyAraStarGrid2DPlanner::path_cost)
      .def_property_readonly("epsilon", &PyAraStarGrid2DPlanner::epsilon);
}

}  // namespace optimus
