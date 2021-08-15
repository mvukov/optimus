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
#ifndef OPTIMUS_NMHE_SOLVER_H_
#define OPTIMUS_NMHE_SOLVER_H_

#include <algorithm>
#include <cassert>
#include <limits>
#include <vector>

#include "Eigen/Dense"
#include "third_party/blas_f77.h"
#include "third_party/lapack_f77.h"

#include "optimus/debug.h"
#include "optimus/mhe_qp_solver.h"

namespace optimus {

/**
 * Implements an NMHE solver based on real-time iteration (RTI) scheme with:
 * - multiple shooting discretization,
 * - a sparse QP solver for solving the underlying QP.
 *
 * For notation see Chapter 3 of the PhD thesis of Milan Vukov available at
 * https://lirias.kuleuven.be/retrieve/315720 . More about the algorithms can be
 * found in the thesis and references therein.
 *
 * NOTE: Must be compiled with at least C++17 support.
 *
 * @tparam nx The number of differential states.
 * @tparam nu The number of controls.
 * @tparam nc The number of terminal equality constraints.
 */
template <size_t nx, size_t nu, size_t nc>
class NmheSolver {
 public:
  static constexpr size_t kNx = nx;
  static constexpr size_t kNu = nu;
  static constexpr size_t kNc = nc;

  using QpSolver = MheQpSolver<nx, nu, nc>;

  using State = typename QpSolver::State;
  using StateArray = typename QpSolver::StateArray;

  using Control = typename QpSolver::Control;
  using ControlArray = typename QpSolver::ControlArray;

  using JacobianStateWrtState = typename QpSolver::JacobianStateWrtState;
  using JacobianStateWrtControl = typename QpSolver::JacobianStateWrtControl;

  using TerminalConstraint = typename QpSolver::TerminalConstraint;
  using JacobianTerminalConstraintWrtState =
      typename QpSolver::JacobianTerminalConstraintWrtState;

  using StateCovariance = typename QpSolver::StateCovariance;

  using MatrixQ = typename QpSolver::MatrixQ;
  using VectorGx = typename QpSolver::VectorGx;
  using VectorR = typename QpSolver::VectorR;
  using VectorGu = typename QpSolver::VectorGu;

  using IntegratorCallback = std::function<bool(
      size_t /*node*/, const Control& /*control*/, State* /*state*/,
      JacobianStateWrtState* /*j_state_wrt_state*/,
      JacobianStateWrtControl* /*j_state_wrt_control*/)>;

  using StageCostEvaluatorCallback = std::function<void(
      size_t /*node*/, const Control& /*control*/, const State& /*state*/,
      MatrixQ* /*q*/, VectorR* /*r*/, VectorGx* /*g_x*/, VectorGu* /*g_u*/)>;

  using StageCostValueCallback = std::function<double(
      size_t /*node*/, const Control& /*control*/, const State& /*state*/)>;

  using TerminalCostEvaluatorCallback = std::function<void(
      const State& /*state*/, MatrixQ* /*q*/, VectorGx* /*g_x*/)>;

  using TerminalCostValueCallback =
      std::function<double(const State& /*state*/)>;

  using TerminalConstraintEvaluatorCallback = std::function<void(
      const State& /*state*/, TerminalConstraint* /*terminal_constraint*/,
      JacobianTerminalConstraintWrtState* /*j_terminal_constraint_wrt_state*/)>;

  struct Config {
    IntegratorCallback integrator_callback;
    StageCostEvaluatorCallback stage_cost_evaluator_callback;
    StageCostValueCallback stage_cost_value_callback;
    TerminalCostEvaluatorCallback terminal_cost_evaluator_callback;
    TerminalCostValueCallback terminal_cost_value_callback;
    TerminalConstraintEvaluatorCallback terminal_constraint_evaluator_callback;

    typename QpSolver::Config qp_solver_config;

    bool Validate() const;
  };

  struct UserData {
    UserData() = delete;
    explicit UserData(const Config& config);

    State arrival_cost_state;
    // Only the lower-triangular part is maintained.
    MatrixQ arrival_cost_information_matrix;

    StateArray states;
    ControlArray controls;

    // Only the lower-triangular part is computed.
    StateCovariance last_state_covariance;

    bool Validate() const;
  };

  struct Workspace {
    Workspace() = delete;
    explicit Workspace(const Config& config);

    QpSolver qp_solver;
  };

  explicit NmheSolver(const Config& config)
      : config_(config), user_data_(config_), workspace_(config_) {}

  bool DoPreparationStep();
  bool DoFeedbackStep();

  /**
   * Implements a simple arrival cost update as a one-step predicition.
   *
   * @returns True on success, false otherwise.
   */
  bool UpdateArrivalCost();

  double GetObjectiveValue() const;
  double GetKktTolerance() const;

  const Config& config() const { return config_; }

  const UserData& user_data() const { return user_data_; }
  UserData* mutable_user_data() { return &user_data_; }

  const Workspace& workspace() const { return workspace_; }

 protected:
  bool EvaluateModel();
  void EvaluateCosts();

  Workspace* mutable_workspace() { return &workspace_; }

 private:
  const Config config_;

  UserData user_data_;
  Workspace workspace_;
};

template <size_t nx, size_t nu, size_t nc>
bool NmheSolver<nx, nu, nc>::Config::Validate() const {
  if (!integrator_callback || !stage_cost_evaluator_callback ||
      !stage_cost_value_callback || !terminal_cost_evaluator_callback ||
      !terminal_cost_value_callback) {
    return false;
  }
  if (nc > 0 && !terminal_constraint_evaluator_callback) {
    return false;
  }
  if (!qp_solver_config.Validate()) {
    return false;
  }
  return true;
}

template <size_t nx, size_t nu, size_t nc>
NmheSolver<nx, nu, nc>::UserData::UserData(const Config& config) {
  const auto num_intervals = config.qp_solver_config.num_intervals;
  assert(num_intervals > 0);

  arrival_cost_state.setZero();
  arrival_cost_information_matrix.setZero();
  states.resize(num_intervals + 1, State::Zero());
  controls.resize(num_intervals, Control::Zero());
  last_state_covariance.setZero();
}

template <size_t nx, size_t nu, size_t nc>
NmheSolver<nx, nu, nc>::Workspace::Workspace(const Config& config)
    : qp_solver(config.qp_solver_config) {}

template <size_t nx, size_t nu, size_t nc>
bool NmheSolver<nx, nu, nc>::DoPreparationStep() {
  return EvaluateModel();
}

template <size_t nx, size_t nu, size_t nc>
bool NmheSolver<nx, nu, nc>::DoFeedbackStep() {
  const size_t num_intervals = config_.qp_solver_config.num_intervals;

  EvaluateCosts();

  if constexpr (nc > 0) {
    auto* qp_user_data = workspace_.qp_solver.mutable_user_data();
    config_.terminal_constraint_evaluator_callback(
        user_data_.states.at(num_intervals), &qp_user_data->f,
        &qp_user_data->e);
  }

  {
    auto* qp_user_data = workspace_.qp_solver.mutable_user_data();
    qp_user_data->arrival_cost_state =
        user_data_.arrival_cost_state - user_data_.states.at(0);
    qp_user_data->arrival_cost_information_matrix =
        user_data_.arrival_cost_information_matrix;
  }

  if (!workspace_.qp_solver.Solve()) {
    return false;
  }

  const auto& qp_workspace = workspace_.qp_solver.workspace();
  for (size_t node = 0; node < num_intervals + 1; ++node) {
    user_data_.states.at(node) += qp_workspace.qp_primal_solution_x.at(node);
  }
  for (size_t node = 0; node < num_intervals; ++node) {
    user_data_.controls.at(node) += qp_workspace.qp_primal_solution_u.at(node);
  }

  return true;
}

template <size_t nx, size_t nu, size_t nc>
bool NmheSolver<nx, nu, nc>::UpdateArrivalCost() {
  auto* qp_user_data = workspace_.qp_solver.mutable_user_data();

  auto& a_stack = qp_user_data->a_stack;
  auto& b_stack = qp_user_data->b_stack;

  auto& q_stack = qp_user_data->q_stack;
  auto& r_stack = qp_user_data->r_stack;
  auto& g_x_stack = qp_user_data->g_x_stack;
  auto& g_u_stack = qp_user_data->g_u_stack;

  State integrated_state = user_data_.states.at(0);
  if (!config_.integrator_callback(0, user_data_.controls.at(0),
                                   &integrated_state, &a_stack.at(0),
                                   &b_stack.at(0))) {
    DEBUG_OPTIMUS("UpdateArrivalCost: failed to integrate model!");
    return false;
  }

  for (size_t node = 0; node < 2; ++node) {
    config_.stage_cost_evaluator_callback(
        node, user_data_.controls.at(node), user_data_.states.at(node),
        &q_stack.at(node), &r_stack.at(node), &g_x_stack.at(node),
        &g_u_stack.at(node));
  }

  Eigen::LLT<MatrixQ> llt_q;
  MatrixQ p_inv;
  MatrixQ big_sigma;
  big_sigma = q_stack.at(0);
  big_sigma += user_data_.arrival_cost_information_matrix;
  big_sigma += workspace_.qp_solver.workspace().kQRegularization;

  llt_q.compute(big_sigma);
  if (llt_q.info() != Eigen::Success) {
    DEBUG_OPTIMUS("Failed to factorize q at init!");
    return false;
  }

  const int if_nx = nx;
  const int if_nu = nu;
  const double if_one = 1.0;
  const double if_zero = 0.0;
  int lapack_info = 0;

  // ALq_0 = A_0 * (Lq_0)^{-T} <=> solve ALq_0 * Lq_0^T = A_0 for ALq_0.
  // A_0 gets overwritten by ALq_0.
  dtrsm_("R", "L", "T", "N", &if_nx, &if_nx, &if_one, llt_q.matrixLLT().data(),
         &if_nx, a_stack.at(0).data(), &if_nx);
  // BLr_0 = B_0 * (Lr_0)^{-T} <=> solve BLr_0 * Lr_0^T = B_0 for BLr_0.
  // B_0 gets overwritten by BLr_0.
  b_stack.at(0) *= r_stack.at(0).cwiseSqrt().cwiseInverse().asDiagonal();

  // Pinv = AL_0 * AL_0^T; the following computes the lower triangular part.
  dsyrk_("L", "N", &if_nx, &if_nx, &if_one, a_stack.at(0).data(), &if_nx,
         &if_zero, p_inv.data(), &if_nx);
  // Pinv += BL_0 * BL_0^T; the following computes the lower triangular part.
  dsyrk_("L", "N", &if_nx, &if_nu, &if_one, b_stack.at(0).data(), &if_nx,
         &if_one, p_inv.data(), &if_nx);

  llt_q.compute(p_inv);
  if (llt_q.info() != Eigen::Success) {
    DEBUG_OPTIMUS("Failed to factorize Pinv at init!");
    return false;
  }

  big_sigma = llt_q.matrixU();
  // U_1 = chol(Pinv)**-T
  dtrtri_("U", "N", &if_nx, big_sigma.data(), &if_nx, &lapack_info);
  if (lapack_info) {
    DEBUG_OPTIMUS("dtrtri_ failed!");
    return false;
  }

  // Sigma = U_N * U_N^T; only the upper triangular part is computed.
  dlauum_("U", &if_nx, big_sigma.data(), &if_nx, &lapack_info);
  if (lapack_info) {
    DEBUG_OPTIMUS("dlauum_ failed!");
    return false;
  }
  big_sigma.transposeInPlace();

  user_data_.arrival_cost_state = integrated_state;
  user_data_.arrival_cost_information_matrix = big_sigma;

  return true;
}

template <size_t nx, size_t nu, size_t nc>
double NmheSolver<nx, nu, nc>::GetObjectiveValue() const {
  double objective_value = 0;

  const State arrival_cost_residual =
      user_data_.states.at(0) - user_data_.arrival_cost_state;
  objective_value += arrival_cost_residual.dot(
      user_data_.arrival_cost_information_matrix * arrival_cost_residual);
  const auto num_intervals = config_.qp_solver_config.num_intervals;
  for (size_t node = 0; node < num_intervals; ++node) {
    objective_value += config_.stage_cost_value_callback(
        node, user_data_.controls.at(node), user_data_.states.at(node));
  }
  objective_value +=
      config_.terminal_cost_value_callback(user_data_.states.at(num_intervals));

  objective_value *= 0.5;
  return objective_value;
}

template <size_t nx, size_t nu, size_t nc>
double NmheSolver<nx, nu, nc>::GetKktTolerance() const {
  const size_t num_intervals = config_.qp_solver_config.num_intervals;
  const auto& qp_user_data = workspace_.qp_solver.user_data();
  const auto& qp_workspace = workspace_.qp_solver.workspace();

  double kkt_tolerance = 0;
  for (size_t node = 0; node < num_intervals + 1; ++node) {
    kkt_tolerance += std::fabs(qp_user_data.g_x_stack.at(node).dot(
        qp_workspace.qp_primal_solution_x.at(node)));
  }
  for (size_t node = 0; node < num_intervals; ++node) {
    kkt_tolerance += std::fabs(qp_user_data.g_u_stack.at(node).dot(
        qp_workspace.qp_primal_solution_u.at(node)));
  }
  for (size_t node = 0; node < num_intervals; ++node) {
    kkt_tolerance += std::fabs(qp_user_data.d_stack.at(node).dot(
        qp_workspace.qp_dual_solution_x.at(node)));
  }
  if constexpr (nc > 0) {
    kkt_tolerance +=
        std::fabs(qp_user_data.f.dot(qp_workspace.qp_dual_solution_c));
  }

  return kkt_tolerance;
}

template <size_t nx, size_t nu, size_t nc>
bool NmheSolver<nx, nu, nc>::EvaluateModel() {
  const size_t num_intervals = config_.qp_solver_config.num_intervals;
  auto* qp_user_data = workspace_.qp_solver.mutable_user_data();

  auto& a_stack = qp_user_data->a_stack;
  auto& b_stack = qp_user_data->b_stack;
  auto& d_stack = qp_user_data->d_stack;

  for (size_t node = 0; node < num_intervals; ++node) {
    const State& current_state = user_data_.states.at(node);
    const State& next_state = user_data_.states.at(node + 1);
    State integrated_state = current_state;
    if (!config_.integrator_callback(node, user_data_.controls.at(node),
                                     &integrated_state, &a_stack.at(node),
                                     &b_stack.at(node))) {
      return false;
    }
    d_stack.at(node) = integrated_state - next_state;
  }
  return true;
}

template <size_t nx, size_t nu, size_t nc>
void NmheSolver<nx, nu, nc>::EvaluateCosts() {
  const size_t num_intervals = config_.qp_solver_config.num_intervals;
  auto* qp_user_data = workspace_.qp_solver.mutable_user_data();

  auto& q_stack = qp_user_data->q_stack;
  auto& r_stack = qp_user_data->r_stack;
  auto& g_x_stack = qp_user_data->g_x_stack;
  auto& g_u_stack = qp_user_data->g_u_stack;

  for (size_t node = 0; node < num_intervals; ++node) {
    config_.stage_cost_evaluator_callback(
        node, user_data_.controls.at(node), user_data_.states.at(node),
        &q_stack.at(node), &r_stack.at(node), &g_x_stack.at(node),
        &g_u_stack.at(node));
  }
  config_.terminal_cost_evaluator_callback(user_data_.states.at(num_intervals),
                                           &q_stack.at(num_intervals),
                                           &g_x_stack.at(num_intervals));
}

}  // namespace optimus

#endif  // OPTIMUS_NMHE_SOLVER_H_
