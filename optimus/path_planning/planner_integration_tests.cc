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

#include "optimus/path_planning/action_set_2d.h"
#include "optimus/path_planning/arastar_grid_2d_planner.h"
#include "optimus/path_planning/astar_grid_2d_planner.h"
#include "optimus/path_planning/astar_se2_planner.h"
#include "optimus/path_planning/dstar_lite_grid_2d_planner.h"
#include "optimus/path_planning/dstar_lite_se2_planner.h"
#include "optimus/path_planning/grid_2d_environment.h"
#include "optimus/path_planning/se2_environment.h"
#include "optimus/path_planning/se2_planner.h"

namespace optimus {

using ::testing::Eq;
using ::testing::FloatNear;
using ::testing::Test;

class TestEnvSetup {
 public:
  static constexpr int kGridWidth = 50;

  TestEnvSetup() {
    grid_data_.resize(kGridWidth * kGridWidth, 0);
    grid_map_ =
        std::make_unique<Grid2DMap>(grid_data_.data(), kGridWidth, kGridWidth);
  }

  void MakeObstacle() {
    for (int x = 25; x < 27; ++x) {
      for (int y = 0; y < 38; ++y) {
        grid_data_[y * kGridWidth + x] = 1;
      }
    }
  }

  std::vector<Position2D> RaiseObstacle() {
    std::vector<Position2D> changed_positions;
    for (int x = 25; x < 27; ++x) {
      for (int y = 38; y < 40; ++y) {
        grid_data_[y * kGridWidth + x] = 1;
        changed_positions.emplace_back(x, y);
      }
    }
    return changed_positions;
  }

  std::vector<Grid2DScalar> grid_data_;
  std::unique_ptr<Grid2DMap> grid_map_;
};

class TestGrid2DPlanners : public Test, public TestEnvSetup {
 public:
  static constexpr float kExpectedStraightPathCost = 26;
  static constexpr float kExpectedPathAroundObstacleCost = 37.3553391;

  TestGrid2DPlanners() {
    start_ = {12, 25};
    goal_ = {38, 25};
  }

  auto GetPathCost() const {
    if (const auto path_cost = planner_->GetPathCost(); path_cost) {
      return *path_cost;
    }
    return kInfCost;
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
    EXPECT_TRUE(CheckPath());
  }

  bool CheckPath() {
    for (const auto& p : path_) {
      const auto x = std::floor(p.x());
      const auto y = std::floor(p.y());
      if (grid_data_[y * kGridWidth + x] != 0) {
        return false;
      }
    }
    return true;
  }

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
  EXPECT_THAT(num_iterations_, Eq(26));
}

TEST_F(TestGrid2DPlanners,
       GivenDStarLiteGrid2DPlanner_WhenEmptySpace_EnsurePathIsFound) {
  planner_ = std::make_unique<DStarLiteGrid2DPlanner>(env_config_);
  PlanPath();
  EXPECT_THAT(GetPathCost(), Eq(kExpectedStraightPathCost));
  EXPECT_THAT(num_iterations_, Eq(27));
}

TEST_F(TestGrid2DPlanners,
       GivenAraStarLiteGrid2DPlanner_WhenEmptySpace_EnsurePathIsFound) {
  AraStarPlannerConfig planner_config;
  planner_ =
      std::make_unique<AraStarGrid2DPlanner>(env_config_, planner_config);
  PlanPath();
  EXPECT_THAT(GetPathCost(), Eq(kExpectedStraightPathCost));
  EXPECT_THAT(num_iterations_, Eq(26));
}

TEST_F(TestGrid2DPlanners,
       GivenAStarGrid2DPlanner_WhenObstaclePresent_EnsurePathIsFound) {
  MakeObstacle();
  planner_ = std::make_unique<AStarGrid2DPlanner>(env_config_);
  PlanPath();
  EXPECT_THAT(GetPathCost(), Eq(kExpectedPathAroundObstacleCost));
  EXPECT_THAT(num_iterations_, Eq(387));
}

TEST_F(TestGrid2DPlanners,
       GivenDStarLiteGrid2DPlanner_WhenObstaclePresent_EnsurePathIsFound) {
  MakeObstacle();
  planner_ = std::make_unique<DStarLiteGrid2DPlanner>(env_config_);
  PlanPath();
  EXPECT_THAT(GetPathCost(), Eq(kExpectedPathAroundObstacleCost));
  EXPECT_THAT(num_iterations_, Eq(377));
}

TEST_F(TestGrid2DPlanners,
       GivenAraStarLiteGrid2DPlanner_WhenObstaclePresent_EnsurePathIsFound) {
  MakeObstacle();
  AraStarPlannerConfig planner_config;
  planner_config.epsilon_start = 1.;  // This case is equivalent to A*.
  planner_ =
      std::make_unique<AraStarGrid2DPlanner>(env_config_, planner_config);
  PlanPath();
  EXPECT_THAT(GetPathCost(), Eq(kExpectedPathAroundObstacleCost));
  EXPECT_THAT(num_iterations_, Eq(387));

  // In this case, the ARA* path is a bit longer than the one from A*.
  planner_config.epsilon_start = 3.;
  planner_ =
      std::make_unique<AraStarGrid2DPlanner>(env_config_, planner_config);
  PlanPath();
  EXPECT_THAT(GetPathCost(), FloatNear(kExpectedPathAroundObstacleCost, 0.6));
  EXPECT_THAT(num_iterations_, Eq(569));
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
    EXPECT_TRUE(CheckPath());
  }

  Position2D new_start_;
};

TEST_F(
    TestGrid2DPlannersNewStartAndRaisedObstacle,
    GivenAStarGrid2DPlanner_WhenObstacleRaisedAndNewStart_EnsurePathIsFound) {
  planner_ = std::make_unique<AStarGrid2DPlanner>(env_config_);
  // A* doesn't support replanning, so, here we just plan from scratch from the
  // new start and with new (raised) obstacle.
  start_ = new_start_;
  RaiseObstacle();
  PlanPath();
  EXPECT_THAT(GetPathCost(), Eq(kExpectedPathAroundRaisedObstacleCost));
  EXPECT_THAT(num_iterations_, Eq(349));
}

TEST_F(
    TestGrid2DPlannersNewStartAndRaisedObstacle,
    GivenDStarLiteGrid2DPlanner_WhenObstacleRaisedAndNewStart_EnsurePathIsFound) {  // NOLINT
  planner_ = std::make_unique<DStarLiteGrid2DPlanner>(env_config_);
  PlanPath();
  EXPECT_THAT(GetPathCost(), Eq(kExpectedPathAroundObstacleCost));
  const auto changed_positions = RaiseObstacle();
  ReplanPath(changed_positions);
  // The point here is that the path cost must be the same as with A* planning
  // from scratch in the same env.
  EXPECT_THAT(GetPathCost(),
              FloatNear(kExpectedPathAroundRaisedObstacleCost, 5e-6));
  EXPECT_THAT(num_iterations_, Eq(145));
}

const ActionSet2D& get_test_primitives();  // Auto-generated primitives.

class TestSE2Planners : public Test, public TestEnvSetup {
 public:
  static constexpr float kExpectedFreeSpacePathCost = 29.3049507;
  static constexpr float kExpectedPathAroundObstacleCost = 37.5892181;

  using Pose2D = SE2PlannerBase::Pose2D;

  TestSE2Planners() {
    start_ = {12, 25, M_PI_2};
    goal_ = {38, 25, -M_PI_2};
  }

  template <class P>
  auto MakePlanner() {
    return std::make_unique<P>(env_config_, &get_test_primitives());
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
    EXPECT_TRUE(CheckPath());
  }

  auto GetPathCost() const {
    if (const auto path_cost = planner_->GetPathCost(); path_cost) {
      return *path_cost;
    }
    return kInfCost;
  }

  bool CheckPath() {
    for (const auto& p : path_) {
      const auto x = std::floor(p.x);
      const auto y = std::floor(p.y);
      if (grid_data_[y * kGridWidth + x] != 0) {
        return false;
      }
    }
    return true;
  }

  SE2Environment::Config env_config_;
  std::unique_ptr<SE2PlannerBase> planner_;
  Pose2D start_;
  Pose2D goal_;
  std::vector<Pose2D> path_;
  int num_iterations_ = 0;
};

TEST_F(TestSE2Planners, GivenAStarSE2Planner_WhenEmptySpace_EnsurePathIsFound) {
  planner_ = MakePlanner<AStarSE2Planner>();
  PlanPath();
  EXPECT_THAT(GetPathCost(), Eq(kExpectedFreeSpacePathCost));
  EXPECT_THAT(num_iterations_, Eq(321));
}

TEST_F(TestSE2Planners,
       GivenDStarLiteSE2Planner_WhenEmptySpace_EnsurePathIsFound) {
  planner_ = MakePlanner<DStarLiteSE2Planner>();
  PlanPath();
  EXPECT_THAT(GetPathCost(), Eq(kExpectedFreeSpacePathCost));
  EXPECT_THAT(num_iterations_, Eq(322));
}

TEST_F(TestSE2Planners,
       GivenAStarSE2Planner_WhenObstaclePresent_EnsurePathIsFound) {
  MakeObstacle();
  planner_ = MakePlanner<AStarSE2Planner>();
  PlanPath();
  EXPECT_THAT(GetPathCost(), Eq(kExpectedPathAroundObstacleCost));
  EXPECT_THAT(num_iterations_, Eq(867));
}

TEST_F(TestSE2Planners,
       GivenDStarLiteSE2Planner_WhenObstaclePresent_EnsurePathIsFound) {
  MakeObstacle();
  planner_ = MakePlanner<DStarLiteSE2Planner>();
  PlanPath();
  EXPECT_THAT(GetPathCost(), FloatNear(kExpectedPathAroundObstacleCost, 5e-6));
  EXPECT_THAT(num_iterations_, Eq(742));
}

class TestSE2PlannersNewStartAndRaisedObstacle : public TestSE2Planners {
 public:
  static constexpr float kExpectedPathAroundRaisedObstacleCost = 35.2602234;

  TestSE2PlannersNewStartAndRaisedObstacle() {
    MakeObstacle();
    new_start_ = {15, 30, M_PI_2 / 2.0};
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
    EXPECT_TRUE(CheckPath());
  }

  Pose2D new_start_;
};

TEST_F(TestSE2PlannersNewStartAndRaisedObstacle,
       GivenAStarSE2Planner_WhenObstacleRaisedAndNewStart_EnsurePathIsFound) {
  planner_ = MakePlanner<AStarSE2Planner>();
  // A* doesn't support replanning, so, here we just plan from scratch from the
  // new start and with new (raised) obstacle.
  start_ = new_start_;
  RaiseObstacle();
  PlanPath();
  EXPECT_THAT(GetPathCost(), Eq(kExpectedPathAroundRaisedObstacleCost));
  EXPECT_THAT(num_iterations_, Eq(652));
}

TEST_F(
    TestSE2PlannersNewStartAndRaisedObstacle,
    GivenDStarLiteSE2Planner_WhenObstacleRaisedAndNewStart_EnsurePathIsFound) {  // NOLINT
  planner_ = MakePlanner<DStarLiteSE2Planner>();
  PlanPath();
  EXPECT_THAT(GetPathCost(), FloatNear(kExpectedPathAroundObstacleCost, 5e-6));
  const auto changed_positions = RaiseObstacle();
  ReplanPath(changed_positions);
  // The point here is that the path cost must be the same as with A* planning
  // from scratch in the same env.
  EXPECT_THAT(GetPathCost(),
              FloatNear(kExpectedPathAroundRaisedObstacleCost, 5e-6));
  EXPECT_THAT(num_iterations_, Eq(522));
}

}  // namespace optimus
