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
#include <cassert>
#include <memory>
#include <random>

#include "benchmark/benchmark.h"

#include "optimus/mhe/mhe_qp_solver.h"

namespace optimus {

template <typename T>
class NormalRandomNumberGenerator {
 public:
  static constexpr size_t kSeed = 42U;

  NormalRandomNumberGenerator(T mean, T stddev, size_t seed)
      : generator_(seed), distribution_(mean, stddev) {}

  T operator()() { return distribution_(generator_); }

 private:
  std::mt19937_64 generator_;
  std::normal_distribution<T> distribution_;
};

template <class Solver>
class BenchmarkMheQpSolver : public ::benchmark::Fixture {
 public:
  using NoiseGenerator = NormalRandomNumberGenerator<double>;

  void SetUp(const ::benchmark::State& state) {
    const size_t num_intervals = state.range(0);
    typename Solver::Config config;
    config.num_intervals = num_intervals;
    solver_.reset(new Solver(config));
    noise_generator_.reset(new NoiseGenerator(0.0, 0.2, NoiseGenerator::kSeed));
  }

 protected:
  void SetUpQp(size_t num_intervals) {
    auto* user_data = solver_->mutable_user_data();

    typename Solver::JacobianStateWrtState a;
    typename Solver::JacobianStateWrtControl b;
    typename Solver::VectorGx d;

    const size_t nx = Solver::kNx;
    const size_t nu = Solver::kNu;

    a.setZero();
    a.template triangularView<Eigen::Upper>().setConstant(1.0);

    b.setZero();
    b(nx - 1, nu - 1) = 0.9;

    for (size_t el = 0; el < nx; ++el) {
      d(el) = 0.005 + (*noise_generator_)();
    }

    typename Solver::MatrixQ q;
    typename Solver::VectorR r;

    q.setZero();
    q(0, 0) = 0.42;
    r.setOnes();
    r *= 0.36 + std::abs((*noise_generator_)());

    auto& a_stack = user_data->a_stack;
    auto& b_stack = user_data->b_stack;
    auto& d_stack = user_data->d_stack;
    auto& q_stack = user_data->q_stack;
    auto& r_stack = user_data->r_stack;
    for (size_t node = 0; node < num_intervals; ++node) {
      a_stack.at(node) = a;
      b_stack.at(node) = b;
      d_stack.at(node) = d;
      q_stack.at(node) = q;
      r_stack.at(node) = r;
    }
    q_stack.back() = q;
  }

  std::unique_ptr<Solver> solver_;
  std::unique_ptr<NoiseGenerator> noise_generator_;
};

#define BENCHMARK_MHE_QP_SOLVER(NX, NU)                            \
  BENCHMARK_TEMPLATE_DEFINE_F(BenchmarkMheQpSolver,                \
                              BenchmarkNmheQpSolver##_##NX##_##NU, \
                              MheQpSolver<NX, NU, 0>)              \
  (::benchmark::State & state) {                                   \
    const size_t num_intervals = state.range(0);                   \
    assert(solver_->config().num_intervals == num_intervals);      \
    for (auto _ : state) {                                         \
      state.PauseTiming();                                         \
      SetUpQp(num_intervals);                                      \
      state.ResumeTiming();                                        \
      if (!solver_->Solve()) {                                     \
        state.SkipWithError("Failed to solve the QP!");            \
        break;                                                     \
      }                                                            \
      ::benchmark::ClobberMemory();                                \
    }                                                              \
  }                                                                \
  BENCHMARK_REGISTER_F(BenchmarkMheQpSolver,                       \
                       BenchmarkNmheQpSolver##_##NX##_##NU)        \
      ->RangeMultiplier(2)                                         \
      ->Range(2, 2 << 6)                                           \
      ->Unit(::benchmark::kMillisecond)                            \
      ->MinTime(2.0)

BENCHMARK_MHE_QP_SOLVER(7, 6);

BENCHMARK_MHE_QP_SOLVER(16, 12);

}  // namespace optimus

BENCHMARK_MAIN();
