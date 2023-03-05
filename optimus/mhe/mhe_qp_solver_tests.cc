// Copyright 2021 Milan Vukov. All rights reserved.
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
#include <limits>
#include <memory>

#include "gmock/gmock.h"

#include "optimus/mhe/mhe_qp_solver.h"
#include "optimus/mhe/mhe_qp_test_data.h"
#include "optimus/testing/eigen_matchers.h"

namespace optimus {

using ::testing::Eq;

constexpr double kEqualTolerance =
    10.0 * std::numeric_limits<double>::epsilon();

class TestUnconstrainedMheQpSolver
    : public ::testing::TestWithParam<QpTestVector> {
 public:
  using Solver = MheQpSolver</*nx*/ kNx, /*nu*/ kNu, /*nc*/ 0>;
  using StateMap = Eigen::Map<const Solver::State>;
  using ControlMap = Eigen::Map<const Solver::Control>;
  using MatrixQMap = Eigen::Map<const Solver::MatrixQ>;
  using VectorRMap = Eigen::Map<const Solver::VectorR>;
  using VectorGxMap = Eigen::Map<const Solver::VectorGx>;
  using VectorGuMap = Eigen::Map<const Solver::VectorGu>;
  using MatrixAMap = Eigen::Map<const Solver::JacobianStateWrtState>;
  using MatrixBMap = Eigen::Map<const Solver::JacobianStateWrtControl>;
};

TEST_P(
    TestUnconstrainedMheQpSolver,
    GivenExternalTestData_WhenSolverCalled_EnsureResultsMatchExpectedResults) {
  const auto& test_vector = GetParam();
  Solver::Config config;
  config.num_intervals = test_vector.num_intervals;
  ASSERT_TRUE(config.Validate());
  Solver solver(config);

  StateMap arrival_cost_state(test_vector.arrival_cost_state.data());
  MatrixQMap arrival_cost_information_matrix(
      test_vector.arrival_cost_information_matrix.data());

  MatrixQMap q(test_vector.q.data());
  VectorRMap r(test_vector.r.data());
  VectorGxMap g_x(test_vector.g_x.data());
  VectorGuMap g_u(test_vector.g_u.data());
  MatrixAMap a(test_vector.a.data());
  MatrixBMap b(test_vector.b.data());
  VectorGxMap d(test_vector.d.data());

  ASSERT_TRUE(test_vector.e.empty());
  ASSERT_TRUE(test_vector.f.empty());

  const auto num_intervals = config.num_intervals;
  auto* user_data = solver.mutable_user_data();
  const auto& workspace = solver.workspace();

  user_data->arrival_cost_state = arrival_cost_state;
  user_data->arrival_cost_information_matrix = arrival_cost_information_matrix;

  auto& q_stack = user_data->q_stack;
  auto& r_stack = user_data->r_stack;

  auto& g_x_stack = user_data->g_x_stack;
  auto& g_u_stack = user_data->g_u_stack;

  auto& a_stack = user_data->a_stack;
  auto& b_stack = user_data->b_stack;
  auto& d_stack = user_data->d_stack;

  for (size_t node = 0; node < num_intervals; ++node) {
    q_stack.at(node) = q;
    r_stack.at(node) = r;

    g_x_stack.at(node) = g_x;
    g_u_stack.at(node) = g_u;

    a_stack.at(node) = a;
    b_stack.at(node) = b;
    d_stack.at(node) = d;
  }
  q_stack.at(num_intervals) = q;
  g_x_stack.at(num_intervals) = g_x;

  ASSERT_TRUE(solver.Solve());
  EXPECT_TRUE(solver.ComputeLastStateCovariance());

  const auto& qp_x = workspace.qp_primal_solution_x;
  const auto& qp_u = workspace.qp_primal_solution_u;
  const auto& qp_lambda_x = workspace.qp_dual_solution_x;

  for (size_t node = 0; node < num_intervals; ++node) {
    const Eigen::Matrix<double, kNx, 1> expected_x =
        a * qp_x.at(node) + b * qp_u.at(node) + d;
    EXPECT_THAT(qp_x.at(node + 1), EigenNear(expected_x, kEqualTolerance))
        << "Contraint violated at node " << node << " expected\n"
        << expected_x << "\ngot\n"
        << qp_x.at(node + 1) << "\n\n";
  }

  for (size_t node = 0; node < num_intervals; ++node) {
    StateMap current_expected_x(test_vector.expected_x.data() + node * kNx);
    ControlMap current_expected_u(test_vector.expected_u.data() + node * kNu);
    StateMap current_expected_lambda_x(test_vector.expected_lambda.data() +
                                       node * kNx);

    EXPECT_THAT(qp_x.at(node), EigenNear(current_expected_x, kEqualTolerance))
        << "x is different at node " << node;
    EXPECT_THAT(qp_u.at(node), EigenNear(current_expected_u, kEqualTolerance))
        << "u is different at node " << node;
    EXPECT_THAT(qp_lambda_x.at(node),
                EigenNear(current_expected_lambda_x, kEqualTolerance))
        << "lambda is different at node " << node;
  }
  StateMap last_expected_x(test_vector.expected_x.data() + num_intervals * kNx);
  EXPECT_THAT(qp_x.at(num_intervals),
              EigenNear(last_expected_x, kEqualTolerance))
      << "The last state is different!";
}

INSTANTIATE_TEST_SUITE_P(QpSolverTests, TestUnconstrainedMheQpSolver,
                         ::testing::ValuesIn(qp_test_vectors));

constexpr size_t kNc = 1;

class TestConstrainedMheQpSolver
    : public ::testing::TestWithParam<QpTestVector> {
 public:
  using ConstrainedSolver = MheQpSolver<kNx, kNu, kNc>;
  using StateMap = Eigen::Map<const ConstrainedSolver::State>;
  using ControlMap = Eigen::Map<const ConstrainedSolver::Control>;
  using MatrixQMap = Eigen::Map<const ConstrainedSolver::MatrixQ>;
  using VectorRMap = Eigen::Map<const ConstrainedSolver::VectorR>;
  using VectorGxMap = Eigen::Map<const ConstrainedSolver::VectorGx>;
  using VectorGuMap = Eigen::Map<const ConstrainedSolver::VectorGu>;
  using MatrixAMap = Eigen::Map<const ConstrainedSolver::JacobianStateWrtState>;
  using MatrixBMap =
      Eigen::Map<const ConstrainedSolver::JacobianStateWrtControl>;
  using MatrixEMap =
      Eigen::Map<const ConstrainedSolver::JacobianTerminalConstraintWrtState>;
  using VectorFMap = Eigen::Map<const ConstrainedSolver::TerminalConstraint>;
  using TerminalConstraint = ConstrainedSolver::TerminalConstraint;
};

TEST_P(
    TestConstrainedMheQpSolver,
    GivenExternalTestData_WhenSolverCalled_EnsureResultsMatchExpectedResults) {
  const auto& test_vector = GetParam();
  ConstrainedSolver::Config config;
  config.num_intervals = test_vector.num_intervals;
  ASSERT_TRUE(config.Validate());
  ConstrainedSolver solver(config);

  StateMap arrival_cost_state(test_vector.arrival_cost_state.data());
  MatrixQMap arrival_cost_information_matrix(
      test_vector.arrival_cost_information_matrix.data());

  MatrixQMap q(test_vector.q.data());
  VectorRMap r(test_vector.r.data());
  VectorGxMap g_x(test_vector.g_x.data());
  VectorGuMap g_u(test_vector.g_u.data());
  MatrixAMap a(test_vector.a.data());
  MatrixBMap b(test_vector.b.data());
  VectorGxMap d(test_vector.d.data());

  ASSERT_THAT(test_vector.e.size(), Eq(kNc * kNx));
  ASSERT_THAT(test_vector.f.size(), Eq(kNc));

  MatrixEMap e(test_vector.e.data());
  VectorFMap f(test_vector.f.data());

  const auto num_intervals = config.num_intervals;
  auto* user_data = solver.mutable_user_data();
  const auto& workspace = solver.workspace();

  user_data->arrival_cost_state = arrival_cost_state;
  user_data->arrival_cost_information_matrix = arrival_cost_information_matrix;

  auto& q_stack = user_data->q_stack;
  auto& r_stack = user_data->r_stack;

  auto& g_x_stack = user_data->g_x_stack;
  auto& g_u_stack = user_data->g_u_stack;

  auto& a_stack = user_data->a_stack;
  auto& b_stack = user_data->b_stack;
  auto& d_stack = user_data->d_stack;

  for (size_t node = 0; node < num_intervals; ++node) {
    q_stack.at(node) = q;
    r_stack.at(node) = r;

    g_x_stack.at(node) = g_x;
    g_u_stack.at(node) = g_u;

    a_stack.at(node) = a;
    b_stack.at(node) = b;
    d_stack.at(node) = d;
  }
  q_stack.at(num_intervals) = q;
  g_x_stack.at(num_intervals) = g_x;

  user_data->e = e;
  user_data->f = f;

  ASSERT_TRUE(solver.Solve());
  EXPECT_TRUE(solver.ComputeLastStateCovariance());

  const auto& qp_x = workspace.qp_primal_solution_x;
  const auto& qp_u = workspace.qp_primal_solution_u;
  const auto& qp_lambda_x = workspace.qp_dual_solution_x;
  const ConstrainedSolver::TerminalConstraint& qp_lambda_c =
      workspace.qp_dual_solution_c;

  for (size_t node = 0; node < num_intervals; ++node) {
    const Eigen::Matrix<double, kNx, 1> expected_x =
        a * qp_x.at(node) + b * qp_u.at(node) + d;
    EXPECT_THAT(qp_x.at(node + 1), EigenNear(expected_x, kEqualTolerance))
        << "Contraint violated at node: " << node << " expected\n"
        << expected_x << "\ngot\n"
        << qp_x.at(node + 1) << "\n\n";
  }

  for (size_t node = 0; node < num_intervals; ++node) {
    StateMap current_expected_x(test_vector.expected_x.data() + node * kNx);
    ControlMap current_expected_u(test_vector.expected_u.data() + node * kNu);
    StateMap current_expected_lambda_x(test_vector.expected_lambda.data() +
                                       node * kNx);

    EXPECT_THAT(qp_x.at(node), EigenNear(current_expected_x, kEqualTolerance))
        << "x is different at node " << node;
    EXPECT_THAT(qp_u.at(node), EigenNear(current_expected_u, kEqualTolerance))
        << "u is different at node " << node;
    EXPECT_THAT(qp_lambda_x.at(node),
                EigenNear(current_expected_lambda_x, kEqualTolerance))
        << "lambda_x is different at node " << node;
  }

  StateMap last_expected_x(test_vector.expected_x.data() + num_intervals * kNx);
  TerminalConstraint residual_ef = e * last_expected_x + f;
  TerminalConstraint zero_tc = ConstrainedSolver::TerminalConstraint::Zero();
  EXPECT_THAT(residual_ef, EigenNear(zero_tc, kEqualTolerance));

  EXPECT_THAT(qp_x.at(num_intervals),
              EigenNear(last_expected_x, kEqualTolerance))
      << "The last state is different!";

  VectorFMap expected_lambda_c(test_vector.expected_lambda.data() +
                               num_intervals * kNx);
  EXPECT_THAT(qp_lambda_c, EigenNear(expected_lambda_c, kEqualTolerance));

  TerminalConstraint residual_tc = e * qp_x.at(num_intervals) + f;
  EXPECT_THAT(residual_tc, EigenNear(zero_tc, kEqualTolerance))
      << "The terminal constraint is not satisfied!";
}

INSTANTIATE_TEST_SUITE_P(QpSolverTests, TestConstrainedMheQpSolver,
                         ::testing::ValuesIn(constrained_qp_test_vectors));

}  // namespace optimus
