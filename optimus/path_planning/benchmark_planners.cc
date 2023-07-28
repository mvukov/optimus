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
#include <string>

#include "benchmark/benchmark.h"

#define STB_IMAGE_IMPLEMENTATION
#include "stb_image.h"  // NOLINT

#include "optimus/check.h"
#include "optimus/path_planning/astar_grid_2d_planner.h"
#include "optimus/path_planning/astar_se2_planner.h"
#include "optimus/path_planning/dstar_lite_grid_2d_planner.h"
#include "optimus/path_planning/dstar_lite_se2_planner.h"

namespace optimus {

class Image {
 public:
  Image(const std::string& file_name, int desired_channels) {
    data_.reset(::stbi_load(file_name.c_str(), &width_, &height_,
                            &num_channels_, desired_channels));
  }

  const auto* data() const { return data_.get(); }
  auto width() const { return width_; }
  auto height() const { return height_; }
  auto num_channels() const { return num_channels_; }

 private:
  struct StbImageDeleter {
    void operator()(unsigned char* img) { ::stbi_image_free(img); }
  };
  std::unique_ptr<unsigned char, StbImageDeleter> data_;

  int width_ = 0;
  int height_ = 0;
  int num_channels_ = 0;  // The number of 8-bit components per pixel.
};

class BenchmarkPlanner : public benchmark::Fixture {
 public:
  BenchmarkPlanner() {
    img_ = std::make_unique<Image>("optimus/path_planning/scsail.png", 0);
    OPTIMUS_CHECK(img_->data() != nullptr);
    OPTIMUS_CHECK(img_->num_channels() == 1);

    grid_2d_map_ = std::make_unique<Grid2DMap>(img_->data(), img_->height(),
                                               img_->width());
  }

  std::unique_ptr<Image> img_;
  std::unique_ptr<Grid2DMap> grid_2d_map_;
};

class BenchmarkGrid2dPlanner : public BenchmarkPlanner {
 public:
  using Position = Grid2DPlannerBase::Position;

  BenchmarkGrid2dPlanner() {
    env_config_.valid_state_threshold = 15;

    start_ = {127, 28};
    goal_ = {146, 436};
  }

  Grid2DEnvironment::Config env_config_;
  Position start_;
  Position goal_;
};

BENCHMARK_DEFINE_F(BenchmarkGrid2dPlanner, AStar)
(benchmark::State& state) {
  AStarGrid2DPlanner planner(env_config_);
  if (!planner.SetGrid2D(grid_2d_map_.get())) {
    state.SkipWithError("Failed to set obstacle data!");
    return;
  }

  std::vector<Position> path;
  for (auto _ : state) {
    if (planner.PlanPath(start_, goal_, {}, path) != PlannerStatus::kSuccess) {
      state.SkipWithError("Failed to generate path!");
    }
  }
}
BENCHMARK_REGISTER_F(BenchmarkGrid2dPlanner, AStar)
    ->Unit(::benchmark::kMillisecond)
    ->MinTime(1.0);

BENCHMARK_DEFINE_F(BenchmarkGrid2dPlanner, DStarLite)
(benchmark::State& state) {
  DStarLiteGrid2DPlanner planner(env_config_);
  if (!planner.SetGrid2D(grid_2d_map_.get())) {
    state.SkipWithError("Failed to set obstacle data!");
    return;
  }

  std::vector<Position> path;
  for (auto _ : state) {
    if (planner.PlanPath(start_, goal_, {}, path) != PlannerStatus::kSuccess) {
      state.SkipWithError("Failed to generate path!");
    }
  }
}
BENCHMARK_REGISTER_F(BenchmarkGrid2dPlanner, DStarLite)
    ->Unit(::benchmark::kMillisecond)
    ->MinTime(2.0);

class BenchmarkSE2Planner : public BenchmarkPlanner {
 public:
  using Pose2D = SE2PlannerBase::Pose2D;

  BenchmarkSE2Planner() {
    env_config_.valid_state_threshold = 15;

    start_ = {127, 28, M_PI_2};
    goal_ = {146, 436, M_PI_2};
  }

  SE2Environment::Config env_config_;
  Pose2D start_;
  Pose2D goal_;
};

const ActionSet2D& get_benchmark_primitives();

BENCHMARK_DEFINE_F(BenchmarkSE2Planner, AStar)
(benchmark::State& state) {
  AStarSE2Planner planner(env_config_, &get_benchmark_primitives());
  if (!planner.SetGrid2D(grid_2d_map_.get())) {
    state.SkipWithError("Failed to set obstacle data!");
    return;
  }

  std::vector<Pose2D> path;
  for (auto _ : state) {
    if (planner.PlanPath(start_, goal_, {}, path) != PlannerStatus::kSuccess) {
      state.SkipWithError("Failed to generate path!");
    }
  }
}
BENCHMARK_REGISTER_F(BenchmarkSE2Planner, AStar)
    ->Unit(::benchmark::kMillisecond)
    ->MinTime(4.0);

BENCHMARK_DEFINE_F(BenchmarkSE2Planner, DStarLite)
(benchmark::State& state) {
  DStarLiteSE2Planner planner(env_config_, &get_benchmark_primitives());
  if (!planner.SetGrid2D(grid_2d_map_.get())) {
    state.SkipWithError("Failed to set obstacle data!");
    return;
  }

  std::vector<Pose2D> path;
  for (auto _ : state) {
    if (planner.PlanPath(start_, goal_, {}, path) != PlannerStatus::kSuccess) {
      state.SkipWithError("Failed to generate path!");
    }
  }
}
BENCHMARK_REGISTER_F(BenchmarkSE2Planner, DStarLite)
    ->Unit(::benchmark::kMillisecond)
    ->MinTime(4.0);

}  // namespace optimus

BENCHMARK_MAIN();
