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

#include <memory>

#include "gmock/gmock.h"
#include "gtest/gtest.h"

#include "optimus/path_planning/astar_grid_2d_planner.h"
#include "optimus/path_planning/dstar_lite_grid_2d_planner.h"

namespace optimus {

using ::testing::Eq;
using ::testing::FloatNear;
using ::testing::Test;

class TestGrid2DPlanners : public Test {
 public:
  static constexpr float kExpectedStraightPathCost = 26;
  static constexpr float kExpectedPathAroundObstacleCost = 37.3553391;

  TestGrid2DPlanners() {
    grid_data_.resize(50 * 50, 0);
    grid_map_ = std::make_unique<Grid2DMap>(grid_data_.data(), 50, 50);
    start_ = {12, 25};
    goal_ = {38, 25};
  }

  void PlanPath() {
    EXPECT_TRUE(planner_->SetGrid2D(grid_map_.get()));

    num_iterations_ = 0;
    auto callback = [this](UserCallbackEvent event) {
      if (event == UserCallbackEvent::kSearch) {
        ++num_iterations_;
      }
      return true;
    };

    EXPECT_THAT(planner_->PlanPath(start_, goal_, callback, path_),
                Eq(PlannerStatus::kSuccess));
  }

  auto GetPathCost() const {
    if (const auto path_cost = planner_->GetPathCost(); path_cost) {
      return *path_cost;
    }
    return kInfCost;
  }

  void MakeObstacle() {
    for (int x = 25; x < 27; ++x) {
      for (int y = 0; y < 38; ++y) {
        grid_data_[y * 50 + x] = 1;
      }
    }
  }

  std::vector<Position2D> RaiseObstacle() {
    std::vector<Position2D> changed_positions;
    for (int x = 25; x < 27; ++x) {
      for (int y = 38; y < 40; ++y) {
        grid_data_[y * 50 + x] = 1;
        changed_positions.emplace_back(x, y);
      }
    }
    return changed_positions;
  }

  std::vector<Grid2DScalar> grid_data_;
  std::unique_ptr<Grid2DMap> grid_map_;
  Grid2DEnvironment::Config env_config_;
  std::unique_ptr<Grid2DPlannerBase> planner_;
  Position2D start_;
  Position2D goal_;
  std::vector<Position2D> path_;
  int num_iterations_ = 0;
};

TEST_F(TestGrid2DPlanners,
       GivenAStarGrid2DPlanner_WhenEmptySpace_EnsurePathIsFound) {
  planner_ = std::make_unique<AStarGrid2DPlanner>(env_config_);
  PlanPath();
  EXPECT_THAT(GetPathCost(), Eq(kExpectedStraightPathCost));
  EXPECT_EQ(num_iterations_, 26);
}

TEST_F(TestGrid2DPlanners,
       GivenDStarLiteGrid2DPlanner_WhenEmptySpace_EnsurePathIsFound) {
  planner_ = std::make_unique<DStarLiteGrid2DPlanner>(env_config_);
  PlanPath();
  EXPECT_THAT(GetPathCost(), Eq(kExpectedStraightPathCost));
  EXPECT_EQ(num_iterations_, 27);
}

TEST_F(TestGrid2DPlanners,
       GivenAStarGrid2DPlanner_WhenObstaclePresent_EnsurePathIsFound) {
  MakeObstacle();
  planner_ = std::make_unique<AStarGrid2DPlanner>(env_config_);
  PlanPath();
  EXPECT_THAT(GetPathCost(), Eq(kExpectedPathAroundObstacleCost));
  EXPECT_EQ(num_iterations_, 387);
}

TEST_F(TestGrid2DPlanners,
       GivenDStarLiteGrid2DPlanner_WhenObstaclePresent_EnsurePathIsFound) {
  MakeObstacle();
  planner_ = std::make_unique<DStarLiteGrid2DPlanner>(env_config_);
  PlanPath();
  EXPECT_THAT(GetPathCost(), Eq(kExpectedPathAroundObstacleCost));
  EXPECT_EQ(num_iterations_, 377);
}

class TestGrid2DPlannersNewStartAndRaisedObstacle : public TestGrid2DPlanners {
 public:
  static constexpr float kExpectedPathAroundRaisedObstacleCost = 35.1126938;

  TestGrid2DPlannersNewStartAndRaisedObstacle() {
    MakeObstacle();
    new_start_ = {15, 30};
  }

  void ReplanPath(const std::vector<Position2D>& changed_positions) {
    num_iterations_ = 0;
    auto callback = [this](UserCallbackEvent event) {
      if (event == UserCallbackEvent::kSearch) {
        ++num_iterations_;
      }
      return true;
    };

    EXPECT_THAT(
        planner_->ReplanPath(new_start_, changed_positions, callback, path_),
        Eq(PlannerStatus::kSuccess));
  }

  Position2D new_start_;
};

TEST_F(
    TestGrid2DPlannersNewStartAndRaisedObstacle,
    GivenAStarGrid2DPlanner_WhenObstacleRaisedAndNewStart_EnsurePathIsFound) {
  planner_ = std::make_unique<AStarGrid2DPlanner>(env_config_);
  start_ = new_start_;
  RaiseObstacle();
  PlanPath();
  EXPECT_THAT(GetPathCost(), Eq(kExpectedPathAroundRaisedObstacleCost));
  EXPECT_EQ(num_iterations_, 349);
}

TEST_F(
    TestGrid2DPlannersNewStartAndRaisedObstacle,
    GivenDStarLiteGrid2DPlanner_WhenObstacleRaisedAndNewStart_EnsurePathIsFound) {  // NOLINT
  planner_ = std::make_unique<DStarLiteGrid2DPlanner>(env_config_);
  PlanPath();
  const auto changed_positions = RaiseObstacle();
  ReplanPath(changed_positions);
  EXPECT_THAT(GetPathCost(),
              FloatNear(kExpectedPathAroundRaisedObstacleCost, 5e-6));
  EXPECT_EQ(num_iterations_, 145);
}

}  // namespace optimus
