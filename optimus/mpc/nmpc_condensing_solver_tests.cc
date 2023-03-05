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
#include "qpoases_embedded/MessageHandling.hpp"

#include "optimus/mpc/nmpc_condensing_solver.h"
#include "optimus/testing/eigen_matchers.h"

namespace optimus {

using ::testing::AllOf;
using ::testing::Eq;
using ::testing::Ge;
using ::testing::Gt;
using ::testing::Le;

constexpr double kDoubleMax = std::numeric_limits<double>::max();
constexpr double kQpEqualTolerance = 1e-12;
constexpr double kEqualTolerance = 1e-14;

constexpr size_t kNumIntervals = 3;
constexpr size_t kNx = 2;
constexpr size_t kNu = 1;
constexpr size_t kNy = 3;
constexpr size_t kNyn = 2;

using Solver = NmpcCondensingSolver</*nx*/ kNx, /*nu*/ kNu>;

class TestNmpcCondensingSolver : public ::testing::Test {
 public:
  void SetUpSolverConfig() {
    auto integrator_callback =
        [this](size_t /*node*/, const Solver::Control& control,
               Solver::State* state,
               Solver::JacobianStateWrtState* j_state_wrt_state,
               Solver::JacobianStateWrtControl* j_state_wrt_control) {
          ++num_integrator_callback_calls_;

          // Implement A and B for a double integrator model, discretized with
          // sampling period of 1 second.
          const Solver::JacobianStateWrtState a =
              (Solver::JacobianStateWrtState() << 1, 1, 0, 1).finished();
          const Solver::JacobianStateWrtControl b =
              (Solver::JacobianStateWrtControl() << 0, 1).finished();
          *state = a * *state + b * control;
          *j_state_wrt_state = a;
          *j_state_wrt_control = b;

          return integrator_callback_return_value_;
        };

    using StageReference = Eigen::Matrix<double, kNy, 1>;
    using TerminalReference = Eigen::Matrix<double, kNyn, 1>;

    const StageReference stage_weight_diag(1, 0.1, 0.1);
    const TerminalReference terminal_weight_diag(1, 1);

    auto stage_cost_evaluator_callback =
        [this, stage_weight_diag](
            size_t /*node*/, const Solver::Control& control,
            const Solver::State& state, Solver::MatrixQ* q, Solver::MatrixR* r,
            Solver::VectorGx* g_x, Solver::VectorGu* g_u) {
          ++num_stage_cost_evaluator_callback_calls_;

          StageReference residual;

          residual.head<kNx>() = state;
          residual.tail<kNu>() = control;

          Eigen::Matrix<double, kNy, kNx> h_x;
          h_x << 1, 0, 0, 1, 0, 0;
          Eigen::Matrix<double, kNy, kNu> h_u;
          h_u << 0, 0, 1;

          *q = h_x.transpose() * stage_weight_diag.asDiagonal() * h_x;
          *r = h_u.transpose() * stage_weight_diag.asDiagonal() * h_u;

          *g_x = h_x.transpose() * stage_weight_diag.cwiseProduct(residual);
          *g_u = h_u.transpose() * stage_weight_diag.cwiseProduct(residual);
        };

    auto stage_cost_value_callback = [stage_weight_diag](
                                         size_t /*node*/,
                                         const Solver::Control& control,
                                         const Solver::State& state) {
      StageReference residual;
      residual << state, control;

      return residual.transpose() * residual.cwiseProduct(stage_weight_diag);
    };

    auto terminal_cost_evaluator_callback = [this, terminal_weight_diag](
                                                const Solver::State& state,
                                                Solver::MatrixQ* q,
                                                Solver::VectorGx* g_x) {
      ++num_terminal_cost_evaluator_callback_;

      const TerminalReference residual = state;

      Eigen::Matrix<double, kNyn, kNx> h_x;
      h_x << 1, 0, 0, 1;
      *q = h_x.transpose() * terminal_weight_diag.asDiagonal() * h_x;
      *g_x = h_x.transpose() * terminal_weight_diag.cwiseProduct(residual);
    };

    auto terminal_cost_value_callback =
        [terminal_weight_diag](const Solver::State& state) {
          return state.transpose() * state.cwiseProduct(terminal_weight_diag);
        };

    config_.num_intervals = kNumIntervals;
    config_.num_working_set_recalculations = 10;
    config_.state_bound_indices = {0, 1};
    config_.integrator_callback = integrator_callback;
    config_.stage_cost_evaluator_callback = stage_cost_evaluator_callback;
    config_.stage_cost_value_callback = stage_cost_value_callback;
    config_.terminal_cost_evaluator_callback = terminal_cost_evaluator_callback;
    config_.terminal_cost_value_callback = terminal_cost_value_callback;
  }

  void SetUpSolverAndUserData() {
    solver_.reset(new Solver(config_));

    auto* user_data = solver_->mutable_user_data();
    for (size_t node = 0; node < kNumIntervals; ++node) {
      user_data->lower_control_bounds.at(node) = Solver::Control(-1);
      user_data->upper_control_bounds.at(node) = Solver::Control(1);

      user_data->lower_state_bounds.at(node) = Solver::State(-10, -2.5);
      user_data->upper_state_bounds.at(node) = Solver::State(10, 2.5);
    }
    user_data->state_feedback = Solver::State(1, 1);
  }

  Solver::Workspace* mutable_workspace() {
    return solver_->mutable_workspace();
  }

  int num_integrator_callback_calls_ = 0;
  bool integrator_callback_return_value_ = true;
  int num_stage_cost_evaluator_callback_calls_ = 0;
  int num_terminal_cost_evaluator_callback_ = 0;

  Solver::Config config_;
  std::unique_ptr<Solver> solver_;
};

TEST_F(TestNmpcCondensingSolver, WhenValidConfig_EnsureValidatePasses) {
  SetUpSolverConfig();

  EXPECT_TRUE(config_.Validate());
}

TEST_F(TestNmpcCondensingSolver,
       GivenValidConfig_WhenUserDataValid_EnsureValidatePasses) {
  SetUpSolverConfig();

  SetUpSolverAndUserData();

  EXPECT_TRUE(solver_->user_data().Validate());
}

TEST_F(TestNmpcCondensingSolver,
       GivenValidConfig_WhenUserDataValid_EnsureCorrectWorkspaceDimensions) {
  SetUpSolverConfig();

  SetUpSolverAndUserData();

  const auto& workspace = solver_->workspace();
  EXPECT_THAT(workspace.num_state_bounds, Eq(kNx * kNumIntervals));
  EXPECT_THAT(workspace.max_num_working_set_recalculations,
              Eq(3u * (kNu + kNx) * kNumIntervals));
}

class TestNmpcCondensingSolverInvalidConfig : public TestNmpcCondensingSolver {
 public:
  TestNmpcCondensingSolverInvalidConfig() { SetUpSolverConfig(); }

  void TearDown() override { EXPECT_FALSE(config_.Validate()); }
};

TEST_F(TestNmpcCondensingSolverInvalidConfig,
       WhenZeroNumIntervals_EnsureValidateFails) {
  config_.num_intervals = 0u;
}

TEST_F(TestNmpcCondensingSolverInvalidConfig,
       WhenZeroNumQpIterations_EnsureValidateFails) {
  config_.num_working_set_recalculations = 0u;
}

TEST_F(TestNmpcCondensingSolverInvalidConfig,
       WhenTooManyBoundIndices_EnsureValidateFails) {
  config_.state_bound_indices = {0, 1, 2};
}

TEST_F(TestNmpcCondensingSolverInvalidConfig,
       WhenInvalidBoundIndices_EnsureValidateFails) {
  config_.state_bound_indices = {0, 2};
}

TEST_F(TestNmpcCondensingSolverInvalidConfig,
       WhenBoundIndicesNotSorted_EnsureValidateFails) {
  config_.state_bound_indices = {1, 0};
}

TEST_F(TestNmpcCondensingSolverInvalidConfig,
       WhenBoundIndicesDuplicates_EnsureValidateFails) {
  config_.state_bound_indices = {0, 0};
}

TEST_F(TestNmpcCondensingSolverInvalidConfig,
       WhenNoIntegratorCallback_EnsureValidateFails) {
  config_.integrator_callback = Solver::IntegratorCallback();
}

TEST_F(TestNmpcCondensingSolverInvalidConfig,
       WhenNoCostEvaluatorCallback_EnsureValidateFails) {
  config_.stage_cost_evaluator_callback = Solver::StageCostEvaluatorCallback();
}

TEST_F(TestNmpcCondensingSolverInvalidConfig,
       WhenNoCostValueCallback_EnsureValidateFails) {
  config_.stage_cost_value_callback = Solver::StageCostValueCallback();
}

TEST_F(TestNmpcCondensingSolverInvalidConfig,
       WhenNoTerminalCostEvaluatorCallback_EnsureValidateFails) {
  config_.terminal_cost_evaluator_callback =
      Solver::TerminalCostEvaluatorCallback();
}

TEST_F(TestNmpcCondensingSolverInvalidConfig,
       WhenNoTerminalCostValueCallback_EnsureValidateFails) {
  config_.terminal_cost_value_callback = Solver::TerminalCostValueCallback();
}

class TestNmpcCondensingSolverInvalidUserData
    : public TestNmpcCondensingSolver {
 public:
  TestNmpcCondensingSolverInvalidUserData() {
    SetUpSolverConfig();
    SetUpSolverAndUserData();
    user_data_ = solver_->mutable_user_data();
  }

  void TearDown() override { EXPECT_FALSE(solver_->user_data().Validate()); }

  Solver::UserData* user_data_ = nullptr;
};

TEST_F(TestNmpcCondensingSolverInvalidUserData,
       WhenControlBoundsInvalidSize_EnsureValidateFails) {
  user_data_->lower_control_bounds.resize(1u);
}

TEST_F(TestNmpcCondensingSolverInvalidUserData,
       WhenStateBoundsInvalidSize_EnsureValidateFails) {
  user_data_->lower_state_bounds.resize(2u);
}

TEST_F(TestNmpcCondensingSolverInvalidUserData,
       WhenInvalidControlBounds_EnsureValidateFails) {
  user_data_->lower_control_bounds.at(1)(0) =
      user_data_->upper_control_bounds.at(1)(0) + 0.1;
}

TEST_F(TestNmpcCondensingSolverInvalidUserData,
       WhenInvalidStateBounds_EnsureValidateFails) {
  user_data_->lower_state_bounds.at(2)(0) =
      user_data_->upper_state_bounds.at(2)(0) + 2.1;
}

class TestCompute : public TestNmpcCondensingSolver {
 public:
  TestCompute() {
    SetUpSolverConfig();
    SetUpSolverAndUserData();
  }

  void DoPreparationAndFeedbackSteps() {
    EXPECT_TRUE(solver_->DoPreparationStep());
    EXPECT_TRUE(solver_->DoFeedbackStep())
        << "Failed to execute the feedback step! Reason: "
        << qpoases_embedded::getErrorString(
               solver_->workspace().qp_solver_status);
  }
};

TEST_F(TestCompute,
       WhenPreparationAndFeedbackStepsCalled_EnsureCorrectNumCallbacks) {
  DoPreparationAndFeedbackSteps();

  EXPECT_THAT(num_integrator_callback_calls_, Eq(kNumIntervals));
  EXPECT_THAT(num_stage_cost_evaluator_callback_calls_, Eq(kNumIntervals));
  EXPECT_THAT(num_terminal_cost_evaluator_callback_, Eq(1));
}

TEST_F(TestCompute,
       WhenPreparationAndFeedbackStepsCalled_EnsureKktTolAndObjValuesGe0) {
  DoPreparationAndFeedbackSteps();

  EXPECT_THAT(solver_->GetKktTolerance(), Ge(0));
  EXPECT_THAT(solver_->GetObjectiveValue(), Ge(0));
}

TEST_F(TestCompute, WhenIntegratorCallbackFails_EnsurePreparationStepFails) {
  integrator_callback_return_value_ = false;

  EXPECT_FALSE(solver_->DoPreparationStep());
}

TEST_F(
    TestCompute,
    GivenValidSetup_WhenSolverStepsCalled_EnsureCorrectInternalDataComputed) {
  const auto& workspace = solver_->workspace();
  const auto num_qp_variables = workspace.num_qp_variables;
  const auto num_state_bounds = workspace.num_state_bounds;
  const auto& qp_solver = workspace.qp_solver;

  ASSERT_TRUE(solver_->DoPreparationStep());

  const std::vector<double> expected_qp_lb(num_qp_variables, -1);
  const std::vector<double> expected_qp_ub(num_qp_variables, 1);

  EXPECT_THAT(qp_solver.getLb(), Eq(expected_qp_lb));
  EXPECT_THAT(qp_solver.getUb(), Eq(expected_qp_ub));

  ASSERT_TRUE(solver_->DoFeedbackStep());

  std::vector<double> expected_qp_lb_a = {-12, -3.5, -13, -3.5, -14, -3.5};
  std::vector<double> expected_qp_ub_a = {8, 1.5, 7, 1.5, 6, 1.5};

  EXPECT_THAT(qp_solver.getLbA(), Eq(expected_qp_lb_a));
  EXPECT_THAT(qp_solver.getUbA(), Eq(expected_qp_ub_a));

  const auto& user_data = solver_->user_data();
  for (size_t node = 0; node < kNumIntervals; ++node) {
    EXPECT_THAT(user_data.controls.at(node)(0),
                AllOf(Ge(-1 - kQpEqualTolerance), Le(1 + kQpEqualTolerance)));
  }
  for (size_t node = 0; node < kNumIntervals + 1; ++node) {
    EXPECT_THAT(user_data.states.at(node)(0), AllOf(Ge(-10), Le(10)));
    EXPECT_THAT(user_data.states.at(node)(1), AllOf(Ge(-2.5), Le(2.5)));
  }

  Solver::ControlArray expected_controls = {Solver::Control(-1),
                                            Solver::Control(-1),
                                            Solver::Control(0.90909090909099)};
  EXPECT_THAT(user_data.controls,
              EigenNear(expected_controls, kQpEqualTolerance));

  Solver::StateArray expected_states = {
      {1, 1}, {2, 1e-13}, {2, -1}, {1, -0.090909090909099}};
  EXPECT_THAT(user_data.states, EigenNear(expected_states, kQpEqualTolerance));

  Eigen::Map<const Solver::QpMatrixH> qp_h(qp_solver.getH().data(),
                                           num_qp_variables, num_qp_variables);
  Eigen::Map<const Solver::QpMatrixA> qp_a(qp_solver.getA().data(),
                                           num_state_bounds, num_qp_variables);

  ASSERT_TRUE(qp_h == qp_h.transpose())
      << "Condensed Hessian is not symmetric!";

  Solver::QpMatrixH expected_qp_h(workspace.num_qp_variables,
                                  workspace.num_qp_variables);
  Solver::QpMatrixA expected_qp_a(workspace.num_state_bounds,
                                  workspace.num_qp_variables);
  // clang-format off
  expected_qp_h << 6.3, 3.1, 1,
                   3.1, 2.2, 1,
                   1,   1,   1.1;
  expected_qp_a << 0, 0, 0,
                   1, 0, 0,
                   1, 0, 0,
                   1, 1, 0,
                   2, 1, 0,
                   1, 1, 1;
  // clang-format on
  EXPECT_THAT(qp_h, EigenNear(expected_qp_h, kQpEqualTolerance));
  EXPECT_THAT(qp_a, EigenNear(expected_qp_a, kQpEqualTolerance));
}

TEST_F(TestCompute, WhenStepsCalledInLoop_EnsureSolverConverges) {
  auto* user_data = solver_->mutable_user_data();
  const auto& workspace = solver_->workspace();

  double kkt_tolerance = kDoubleMax;
  double objective_value = kDoubleMax;
  for (size_t iteration = 0u; iteration < 10u; ++iteration) {
    ASSERT_TRUE(solver_->DoPreparationStep())
        << "Failed to execute the preparation step at iteration " << iteration
        << "!";
    ASSERT_TRUE(solver_->DoFeedbackStep())
        << "Failed to execute the feedback step at iteration " << iteration
        << "! Reason: "
        << qpoases_embedded::getErrorString(workspace.qp_solver_status);

    for (size_t node = 0; node < kNumIntervals - 1; ++node) {
      user_data->controls.at(node) = user_data->controls.at(node + 1);
    }
    for (size_t node = 0; node < kNumIntervals; ++node) {
      user_data->states.at(node) = user_data->states.at(node + 1);
    }
    user_data->state_feedback = user_data->states.at(0);

    auto new_kkt_tolerance = solver_->GetKktTolerance();
    EXPECT_THAT(new_kkt_tolerance, Le(kkt_tolerance));
    kkt_tolerance = new_kkt_tolerance;

    auto new_objective_value = solver_->GetObjectiveValue();
    EXPECT_THAT(new_objective_value, Le(objective_value));
    objective_value = new_objective_value;
  }

  constexpr double kConvergedTolerance = 2e-9;
  EXPECT_THAT(kkt_tolerance, AllOf(Gt(0), Le(kConvergedTolerance)));
  EXPECT_THAT(objective_value, AllOf(Gt(0), Le(kConvergedTolerance)));
}

}  // namespace optimus
