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
#ifndef OPTIMUS_MPC_NMPC_CONDENSING_SOLVER_H_
#define OPTIMUS_MPC_NMPC_CONDENSING_SOLVER_H_

#include <algorithm>
#include <limits>
#include <memory>
#include <vector>

#include "Eigen/Core"
#include "qpoases_embedded/QProblem.hpp"

#ifdef OPTIMUS_DEBUG
#include "qpoases_embedded/Utils.hpp"
#endif  // OPTIMUS_DEBUG

#include "optimus/debug.h"

namespace optimus {

/**
 * Implements an NMPC solver based on real-time iteration (RTI) scheme with:
 * - multiple shooting discretization,
 * - N^2 condensing of the underlying QP,
 * - qpOASES QP solver for solving the condensed QP.
 *
 * For notation see Chapter 3 of the PhD thesis of Milan Vukov available at
 * https://lirias.kuleuven.be/retrieve/315720 . More about the algorithms can be
 * found in the thesis and references therein.
 *
 * NOTE: Must be compiled with at least C++17 support.
 *
 * @tparam nx The number of differential states.
 * @tparam nu The number of controls.
 */
template <size_t nx, size_t nu>
class NmpcCondensingSolver {
 public:
  static constexpr double kNoUpperBound = qpoases_embedded::INFTY;
  static constexpr double kNoLowerBound = -kNoUpperBound;

  using State = Eigen::Matrix<double, nx, 1>;
  using StateArray = std::vector<State>;

  using Control = Eigen::Matrix<double, nu, 1>;
  using ControlArray = std::vector<Control>;

  using JacobianStateWrtState = Eigen::Matrix<double, nx, nx>;
  using JacobianStateWrtControl = Eigen::Matrix<double, nx, nu>;

  using MatrixQ = Eigen::Matrix<double, nx, nx>;
  using VectorGx = Eigen::Matrix<double, nx, 1>;
  using MatrixR = Eigen::Matrix<double, nu, nu>;
  using VectorGu = Eigen::Matrix<double, nu, 1>;

  using QpMatrixH = Eigen::MatrixXd;
  using QpMatrixA =
      Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>;
  using QpVector = Eigen::VectorXd;

  using IntegratorCallback = std::function<bool(
      size_t /*node*/, const Control& /*control*/, State* /*state*/,
      JacobianStateWrtState* /*j_state_wrt_state*/,
      JacobianStateWrtControl* /*j_state_wrt_control*/)>;

  using StageCostEvaluatorCallback = std::function<void(
      size_t /*node*/, const Control& /*control*/, const State& /*state*/,
      MatrixQ* /*q*/, MatrixR* /*r*/, VectorGx* /*g_x*/, VectorGu* /*g_u*/)>;

  using StageCostValueCallback = std::function<double(
      size_t /*node*/, const Control& /*control*/, const State& /*state*/)>;

  using TerminalCostEvaluatorCallback = std::function<void(
      const State& /*state*/, MatrixQ* /*q*/, VectorGx* /*g_x*/)>;

  using TerminalCostValueCallback =
      std::function<double(const State& /*state*/)>;

  struct Config {
    size_t num_intervals;
    std::vector<size_t> state_bound_indices;

    size_t num_working_set_recalculations = 1u;
    bool warmstart_qp_solver = true;

    IntegratorCallback integrator_callback;
    StageCostEvaluatorCallback stage_cost_evaluator_callback;
    StageCostValueCallback stage_cost_value_callback;
    TerminalCostEvaluatorCallback terminal_cost_evaluator_callback;
    TerminalCostValueCallback terminal_cost_value_callback;

    bool Validate() const;
  };

  struct UserData {
    UserData() = delete;
    explicit UserData(const Config& config);

    State state_feedback;

    StateArray states;
    ControlArray controls;

    // Bounds for states 1..num_intervals.
    StateArray lower_state_bounds, upper_state_bounds;
    // Bounds for controls 0..(num_intervals - 1).
    ControlArray lower_control_bounds, upper_control_bounds;

    // Validates that bounds are set correctly.
    bool Validate() const;
  };

  struct Workspace {
    Workspace() = delete;
    explicit Workspace(const Config& config);

    const size_t num_qp_variables;
    const size_t num_state_bounds;
    const size_t max_num_working_set_recalculations;

    std::vector<JacobianStateWrtState> a_stack;
    std::vector<JacobianStateWrtControl> b_stack;

    std::vector<MatrixQ> q_stack;
    std::vector<MatrixR> r_stack;
    std::vector<VectorGx> g_x_stack;
    std::vector<VectorGu> g_u_stack;

    std::vector<State> d_stack;
    std::vector<State> s_bar_stack;
    std::vector<JacobianStateWrtState> c_stack;
    std::vector<JacobianStateWrtControl> e_stack;

    qpoases_embedded::QProblem qp_solver;
    qpoases_embedded::returnValue qp_solver_status =
        qpoases_embedded::SUCCESSFUL_RETURN;
    int num_working_set_recalculations = 1;
  };

  explicit NmpcCondensingSolver(const Config& config)
      : config_(config), user_data_(config_), workspace_(config_) {}

  bool DoPreparationStep();
  bool DoFeedbackStep();

  double GetObjectiveValue() const;
  double GetKktTolerance() const;

  const UserData& user_data() const { return user_data_; }
  UserData* mutable_user_data() { return &user_data_; }

  const Workspace& workspace() const { return workspace_; }

 protected:
  bool EvaluateModel();
  void EvaluateCosts();
  void ExpandQpSolution();

  Workspace* mutable_workspace() { return &workspace_; }

 private:
  friend class TestNmpcCondensingSolver;

  const Config config_;

  UserData user_data_;
  Workspace workspace_;
};

template <size_t nx, size_t nu>
bool NmpcCondensingSolver<nx, nu>::Config::Validate() const {
  if (num_intervals == 0U) {
    OPTIMUS_PRINT("The number of intervals must be >= 0!");
    return false;
  }
  if (num_working_set_recalculations == 0U) {
    OPTIMUS_PRINT("The number of working set recalculations must be >= 0!");
    return false;
  }

  if (state_bound_indices.size() > nx ||
      !std::is_sorted(state_bound_indices.begin(), state_bound_indices.end())) {
    OPTIMUS_PRINT("The state bound indices must be sorted!");
    return false;
  }
  if (std::adjacent_find(state_bound_indices.begin(),
                         state_bound_indices.end()) !=
      state_bound_indices.end()) {
    OPTIMUS_PRINT("The state bound indices must be unique!");
    return false;
  }
  for (const auto& index : state_bound_indices) {
    if (index >= nx) {
      OPTIMUS_PRINT("All state bound indices must be >= 0 and < " << nx << "!");
      return false;
    }
  }

  if (!integrator_callback || !stage_cost_evaluator_callback ||
      !stage_cost_value_callback || !terminal_cost_evaluator_callback ||
      !terminal_cost_value_callback) {
    OPTIMUS_PRINT("All NMPC solver callbacks must be defined!");
    return false;
  }

  return true;
}

template <size_t nx, size_t nu>
NmpcCondensingSolver<nx, nu>::UserData::UserData(const Config& config) {
  const auto num_intervals = config.num_intervals;
  state_feedback.setZero();
  states.resize(num_intervals + 1, State::Zero());
  controls.resize(num_intervals, Control::Zero());

  lower_state_bounds.resize(num_intervals);
  for (State& bound : lower_state_bounds) {
    bound.setConstant(kNoLowerBound);
  }

  upper_state_bounds.resize(num_intervals);
  for (State& bound : upper_state_bounds) {
    bound.setConstant(kNoUpperBound);
  }

  lower_control_bounds.resize(num_intervals);
  for (Control& bound : lower_control_bounds) {
    bound.setConstant(kNoLowerBound);
  }

  upper_control_bounds.resize(num_intervals);
  for (Control& bound : upper_control_bounds) {
    bound.setConstant(kNoUpperBound);
  }
}

template <size_t nx, size_t nu>
bool NmpcCondensingSolver<nx, nu>::UserData::Validate() const {
  if (lower_control_bounds.size() != upper_control_bounds.size()) {
    return false;
  }
  for (size_t node = 0; node < lower_control_bounds.size(); ++node) {
    for (size_t el = 0; el < nu; ++el) {
      if (lower_control_bounds.at(node)(el) >
          upper_control_bounds.at(node)(el)) {
        return false;
      }
    }
  }
  if (lower_state_bounds.size() != upper_state_bounds.size()) {
    return false;
  }
  for (size_t node = 0; node < lower_state_bounds.size(); ++node) {
    for (size_t el = 0; el < nx; ++el) {
      if (lower_state_bounds.at(node)(el) > upper_state_bounds.at(node)(el)) {
        return false;
      }
    }
  }

  return true;
}

template <size_t nx, size_t nu>
NmpcCondensingSolver<nx, nu>::Workspace::Workspace(const Config& config)
    : num_qp_variables(nu * config.num_intervals),
      num_state_bounds(config.state_bound_indices.size() *
                       config.num_intervals),
      max_num_working_set_recalculations(3 *
                                         (num_qp_variables + num_state_bounds)),
      qp_solver(num_qp_variables, num_state_bounds) {
  const auto num_intervals = config.num_intervals;
  a_stack.resize(num_intervals, JacobianStateWrtState::Zero());
  b_stack.resize(num_intervals, JacobianStateWrtControl::Zero());

  q_stack.resize(num_intervals + 1, MatrixQ::Zero());
  r_stack.resize(num_intervals, MatrixR::Zero());
  g_x_stack.resize(num_intervals + 1, State::Zero());
  g_u_stack.resize(num_intervals, Control::Zero());

  d_stack.resize(num_intervals, State::Zero());
  s_bar_stack.resize(num_intervals + 1, State::Zero());
  c_stack.resize(num_intervals, JacobianStateWrtState::Zero());
  e_stack.resize(num_intervals * (num_intervals + 1) / 2,
                 JacobianStateWrtControl::Zero());

  auto* qp_a = qp_solver.getMutableA();
  // NOTE: blocks above the block-diagonal of the A-matrix are always zero.
  std::fill(qp_a->begin(), qp_a->end(), 0);
}

template <size_t nx, size_t nu>
bool NmpcCondensingSolver<nx, nu>::DoPreparationStep() {
  const size_t num_intervals = config_.num_intervals;

  const auto& a_stack = workspace_.a_stack;
  const auto& b_stack = workspace_.b_stack;
  const auto& q_stack = workspace_.q_stack;
  const auto& r_stack = workspace_.r_stack;

  auto& c_stack = workspace_.c_stack;
  auto& e_stack = workspace_.e_stack;

  if (!EvaluateModel()) {
    return false;
  }
  EvaluateCosts();

  c_stack.at(0) = a_stack.at(0);
  for (size_t node = 1; node < num_intervals; ++node) {
    c_stack.at(node) = a_stack.at(node) * c_stack.at(node - 1);
  }

  // Implements indexing of the col-by-col stacked non-zero blocks of the
  // E-matrix.
  auto get_e_stack_block_index = [num_intervals](size_t row, size_t col) {
    assert(col <= row);
    const size_t index = row + col * (2 * num_intervals - col - 1) / 2;
    assert(index < num_intervals * (num_intervals + 1) / 2);
    return index;
  };

  const MatrixQ& q_n = q_stack.back();
  const auto num_qp_variables = workspace_.num_qp_variables;
  Eigen::Map<QpMatrixH> qp_h(workspace_.qp_solver.getMutableH()->data(),
                             num_qp_variables, num_qp_variables);
  for (size_t col = 0; col < num_intervals; ++col) {
    const size_t e_stack_col_offset = get_e_stack_block_index(col, col);

    // Compute blocks of the E-matrix.
    e_stack.at(e_stack_col_offset) = b_stack.at(col);
    for (size_t row = 1; row < num_intervals - col; ++row) {
      const size_t e_stack_block_index = e_stack_col_offset + row;
      e_stack.at(e_stack_block_index) =
          a_stack.at(col + row) * e_stack.at(e_stack_block_index - 1);
    }

    // Compute blocks of the (condensed Hessian) H-matrix.
    JacobianStateWrtControl w1 =
        q_n * e_stack.at(e_stack_col_offset + num_intervals - col - 1);
    JacobianStateWrtControl w2;
    for (int row = num_intervals - 1; row > static_cast<int>(col); --row) {
      qp_h.block<nu, nu>(row * nu, col * nu) = w1.transpose() * b_stack.at(row);

      w2 = a_stack.at(row).transpose() * w1;
      w1 =
          q_stack.at(row) * e_stack.at(e_stack_col_offset + row - col - 1) + w2;
    }
    qp_h.block<nu, nu>(col * nu, col * nu) =
        b_stack.at(col).transpose() * w1 + r_stack.at(col);
  }

  for (size_t col = 0; col < num_intervals; ++col) {
    for (size_t row = 0; row < col; ++row) {
      qp_h.block<nu, nu>(row * nu, col * nu) =
          qp_h.block<nu, nu>(col * nu, row * nu).transpose();
    }
  }

  Eigen::Map<QpVector> qp_lb(workspace_.qp_solver.getMutableLb()->data(),
                             num_qp_variables);
  for (size_t node = 0; node < num_intervals; ++node) {
    qp_lb.template segment<nu>(node * nu) =
        user_data_.lower_control_bounds.at(node) - user_data_.controls.at(node);
  }
  Eigen::Map<QpVector> qp_ub(workspace_.qp_solver.getMutableUb()->data(),
                             num_qp_variables);
  for (size_t node = 0; node < num_intervals; ++node) {
    qp_ub.template segment<nu>(node * nu) =
        user_data_.upper_control_bounds.at(node) - user_data_.controls.at(node);
  }

  size_t qp_a_row_offset = 0;
  Eigen::Map<QpMatrixA> qp_a(workspace_.qp_solver.getMutableA()->data(),
                             workspace_.num_state_bounds, num_qp_variables);
  for (size_t row = 0; row < num_intervals; ++row) {
    for (const auto bound_index : config_.state_bound_indices) {
      assert(bound_index < nx);
      for (size_t col = 0; col < row + 1; ++col) {
        const size_t e_stack_block_index = get_e_stack_block_index(row, col);
        qp_a.block<1, nu>(qp_a_row_offset, col * nu) =
            e_stack.at(e_stack_block_index).row(bound_index);
      }
      ++qp_a_row_offset;
    }
  }
  assert(qp_a_row_offset == workspace_.num_state_bounds);

  return true;
}

template <size_t nx, size_t nu>
bool NmpcCondensingSolver<nx, nu>::DoFeedbackStep() {
  const size_t num_intervals = config_.num_intervals;

  const auto& a_stack = workspace_.a_stack;
  const auto& b_stack = workspace_.b_stack;
  const auto& q_stack = workspace_.q_stack;
  const auto& g_x_stack = workspace_.g_x_stack;
  const auto& g_u_stack = workspace_.g_u_stack;
  const auto& d_stack = workspace_.d_stack;

  auto& s_bar_stack = workspace_.s_bar_stack;

  Eigen::Map<QpVector> qp_g(workspace_.qp_solver.getMutableG()->data(),
                            workspace_.num_qp_variables);
  for (size_t node = 0; node < num_intervals; ++node) {
    qp_g.segment<nu>(node * nu) = g_u_stack.at(node);
  }

  s_bar_stack.at(0) = user_data_.state_feedback - user_data_.states.at(0);
  for (size_t node = 0; node < num_intervals; ++node) {
    s_bar_stack.at(node + 1) = d_stack.at(node);
  }

  for (size_t node = 0; node < num_intervals; ++node) {
    s_bar_stack.at(node + 1) += a_stack.at(node) * s_bar_stack.at(node);
  }

  State w1 = q_stack.back() * s_bar_stack.back() + g_x_stack.back();
  State w2;
  for (int k = num_intervals - 1; k > 0; --k) {
    qp_g.segment<nu>(k * nu) += b_stack.at(k).transpose() * w1;
    w2 = a_stack.at(k).transpose() * w1 + g_x_stack.at(k);
    w1 = q_stack.at(k) * s_bar_stack.at(k) + w2;
  }
  qp_g.segment<nu>(0) += b_stack.at(0).transpose() * w1;

  {
    const auto num_state_bounds = workspace_.num_state_bounds;
    Eigen::Map<QpVector> qp_lb_a(workspace_.qp_solver.getMutableLbA()->data(),
                                 num_state_bounds);
    Eigen::Map<QpVector> qp_ub_a(workspace_.qp_solver.getMutableUbA()->data(),
                                 num_state_bounds);
    size_t offset = 0;
    for (size_t node = 0; node < num_intervals; ++node) {
      for (const auto bound_index : config_.state_bound_indices) {
        assert(bound_index < nx);
        const size_t state_index = node + 1;
        const double s_bar_plus_x =
            s_bar_stack.at(state_index)(bound_index) +
            user_data_.states.at(state_index)(bound_index);
        // Lower/upper bound group offset corresponds to state node + 1.
        qp_lb_a(offset) =
            user_data_.lower_state_bounds.at(node)(bound_index) - s_bar_plus_x;
        qp_ub_a(offset) =
            user_data_.upper_state_bounds.at(node)(bound_index) - s_bar_plus_x;

        ++offset;
      }
    }
    assert(offset == workspace_.num_state_bounds);
  }

  workspace_.num_working_set_recalculations =
      std::min(config_.num_working_set_recalculations,
               workspace_.max_num_working_set_recalculations);
  if (!config_.warmstart_qp_solver) {
    auto* qp_x = workspace_.qp_solver.getMutableX();
    auto* qp_y = workspace_.qp_solver.getMutableY();

    std::fill(qp_x->begin(), qp_x->end(), 0);
    std::fill(qp_y->begin(), qp_y->end(), 0);
  }
#ifdef OPTIMUS_DEBUG
  workspace_.qp_solver_status =
      workspace_.qp_solver.init(workspace_.num_working_set_recalculations,
                                qpoases_embedded::printIteration);
#else   // OPTIMUS_DEBUG
  workspace_.qp_solver_status =
      workspace_.qp_solver.init(workspace_.num_working_set_recalculations, {});
#endif  // OPTIMUS_DEBUG

  ExpandQpSolution();

  return workspace_.qp_solver_status == qpoases_embedded::SUCCESSFUL_RETURN;
}

template <size_t nx, size_t nu>
double NmpcCondensingSolver<nx, nu>::GetObjectiveValue() const {
  double result = 0;

  const auto num_intervals = config_.num_intervals;
  for (size_t node = 0; node < num_intervals; ++node) {
    result += config_.stage_cost_value_callback(
        node, user_data_.controls.at(node), user_data_.states.at(node));
  }
  result +=
      config_.terminal_cost_value_callback(user_data_.states.at(num_intervals));

  result *= 0.5;
  return result;
}

template <size_t nx, size_t nu>
double NmpcCondensingSolver<nx, nu>::GetKktTolerance() const {
  const auto& qp_solver = workspace_.qp_solver;
  const auto num_qp_variables = workspace_.num_qp_variables;
  const auto num_state_bounds = workspace_.num_state_bounds;
  Eigen::Map<const QpVector> qp_g(qp_solver.getG().data(), num_qp_variables);
  Eigen::Map<const QpVector> qp_lb(qp_solver.getLb().data(), num_qp_variables);
  Eigen::Map<const QpVector> qp_ub(qp_solver.getUb().data(), num_qp_variables);
  Eigen::Map<const QpVector> qp_lb_a(qp_solver.getLbA().data(),
                                     num_qp_variables);
  Eigen::Map<const QpVector> qp_ub_a(qp_solver.getUbA().data(),
                                     num_qp_variables);
  Eigen::Map<const QpVector> qp_x(qp_solver.getX().data(), num_qp_variables);
  Eigen::Map<const QpVector> qp_y(qp_solver.getY().data(),
                                  num_qp_variables + num_state_bounds);

  double kkt_tolerance = std::fabs(qp_g.dot(qp_x));

  static constexpr double kTolerance = 1.0 / qpoases_embedded::INFTY;
  for (size_t el = 0; el < num_qp_variables; ++el) {
    const auto y = qp_y(el);
    if (y > kTolerance) {
      kkt_tolerance += std::fabs(qp_lb(el) * y);
    } else if (y < -kTolerance) {
      kkt_tolerance += std::fabs(qp_ub(el) * y);
    }
  }

  for (size_t el = 0; el < num_state_bounds; ++el) {
    const auto y = qp_y(num_qp_variables + el);
    if (y > kTolerance) {
      kkt_tolerance += std::fabs(qp_lb_a(el) * y);
    } else if (y < -kTolerance) {
      kkt_tolerance += std::fabs(qp_ub_a(el) * y);
    }
  }

  return kkt_tolerance;
}

template <size_t nx, size_t nu>
bool NmpcCondensingSolver<nx, nu>::EvaluateModel() {
  const auto num_intervals = config_.num_intervals;

  auto& a_stack = workspace_.a_stack;
  auto& b_stack = workspace_.b_stack;
  auto& d_stack = workspace_.d_stack;

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

template <size_t nx, size_t nu>
void NmpcCondensingSolver<nx, nu>::EvaluateCosts() {
  const auto num_intervals = config_.num_intervals;

  auto& q_stack = workspace_.q_stack;
  auto& r_stack = workspace_.r_stack;
  auto& g_x_stack = workspace_.g_x_stack;
  auto& g_u_stack = workspace_.g_u_stack;

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

template <size_t nx, size_t nu>
void NmpcCondensingSolver<nx, nu>::ExpandQpSolution() {
  const size_t num_intervals = config_.num_intervals;

  const auto& a_stack = workspace_.a_stack;
  const auto& b_stack = workspace_.b_stack;
  const auto& d_stack = workspace_.d_stack;
  Eigen::Map<const QpVector> qp_x(workspace_.qp_solver.getX().data(),
                                  workspace_.num_qp_variables);

  auto& s_bar_stack = workspace_.s_bar_stack;
  ControlArray& controls = user_data_.controls;
  StateArray& states = user_data_.states;

  for (size_t node = 0; node < num_intervals; ++node) {
    controls.at(node) += qp_x.segment<nu>(node * nu);
  }

  for (size_t node = 0; node < num_intervals; ++node) {
    s_bar_stack.at(node + 1) = d_stack.at(node);
  }
  for (size_t node = 0; node < num_intervals; ++node) {
    s_bar_stack.at(node + 1) += a_stack.at(node) * s_bar_stack.at(node);
    s_bar_stack.at(node + 1) += b_stack.at(node) * qp_x.segment<nu>(node * nu);
  }
  for (size_t node = 0; node < num_intervals + 1; ++node) {
    states.at(node) += s_bar_stack.at(node);
  }
}

}  // namespace optimus

#endif  // OPTIMUS_MPC_NMPC_CONDENSING_SOLVER_H_
