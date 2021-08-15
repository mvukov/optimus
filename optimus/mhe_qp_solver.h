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
#ifndef OPTIMUS_MHE_QP_SOLVER_H_
#define OPTIMUS_MHE_QP_SOLVER_H_

#include <algorithm>
#include <cassert>
#include <limits>
#include <vector>

#include "Eigen/Dense"
#include "third_party/blas_f77.h"
#include "third_party/lapack_f77.h"

#include "optimus/debug.h"

namespace optimus {

/**
 * Implements a solver for equality-constrained MHE QPs.
 *
 * The algorithm is explained in details in:
 * - [1] "Algorithms and methods for high-performance model predictive control"
 *   by Gialuca Frison, available at
 *   https://orbit.dtu.dk/files/124371046/phd402_Frison_G.pdf
 * - [2] "High-Performance Small-Scale Solvers for Moving Horizon Estimation"
 *   by Gianluca Frison et al, available at
 *   http://citeseerx.ist.psu.edu/viewdoc/download?doi=10.1.1.728.3136&rep=rep1&type=pdf
 *
 * For notation consult the above references.
 *
 * NOTE: Must be compiled with at least C++17 support.
 *
 * @tparam nx The number of differential states.
 * @tparam nu The number of controls.
 * @tparam nc The number of terminal equality constraints.
 */
template <size_t nx, size_t nu, size_t nc>
class MheQpSolver {
 public:
  static constexpr size_t kNx = nx;
  static constexpr size_t kNu = nu;
  static constexpr size_t kNc = nc;

  using State = Eigen::Matrix<double, nx, 1>;
  using StateArray = std::vector<State>;

  using Control = Eigen::Matrix<double, nu, 1>;
  using ControlArray = std::vector<Control>;

  using JacobianStateWrtState = Eigen::Matrix<double, nx, nx>;
  using JacobianStateWrtControl = Eigen::Matrix<double, nx, nu>;

  using TerminalConstraint = Eigen::Matrix<double, nc, 1>;
  using JacobianTerminalConstraintWrtState = Eigen::Matrix<double, nc, nx>;

  using StateCovariance = Eigen::Matrix<double, nx - nc, nx - nc>;

  using MatrixQ = Eigen::Matrix<double, nx, nx>;
  using VectorGx = Eigen::Matrix<double, nx, 1>;
  using VectorR = Eigen::Matrix<double, nu, 1>;
  using VectorGu = Eigen::Matrix<double, nu, 1>;
  using MatrixLE = Eigen::Matrix<double, nc, nc>;

  struct Config {
    size_t num_intervals = 0;

    bool Validate() const;
  };

  struct UserData {
    UserData() = delete;
    explicit UserData(const Config& config);

    State arrival_cost_state;
    // Only the lower-triangular part is being maintained.
    MatrixQ arrival_cost_information_matrix;

    // Only the lower-triangular part is computed.
    StateCovariance last_state_covariance;

    // NOTE: the contents is overwritten while solving the QP!
    std::vector<JacobianStateWrtState> a_stack;
    // NOTE: the contents is overwritten while solving the QP!
    std::vector<JacobianStateWrtControl> b_stack;
    std::vector<MatrixQ> q_stack;
    std::vector<VectorR> r_stack;
    std::vector<VectorGx> g_x_stack;
    std::vector<VectorGu> g_u_stack;
    std::vector<State> d_stack;

    JacobianTerminalConstraintWrtState e;
    TerminalConstraint f;
  };

  struct Workspace {
    Workspace() = delete;
    explicit Workspace(const Config& config);

    const double kRegularizationFactor;
    const MatrixQ kQRegularization;

    std::vector<MatrixQ> l_q_stack;
    std::vector<VectorR> inverse_l_r_stack;
    std::vector<MatrixQ> big_u_stack;

    JacobianTerminalConstraintWrtState e_l_q;
    MatrixLE l_e;

    std::vector<State> q_hat_stack;
    std::vector<Control> r_hat_stack;
    std::vector<State> b_hat_stack;
    State sigma_n;

    std::vector<State> qp_primal_solution_x;
    std::vector<Control> qp_primal_solution_u;
    std::vector<State> qp_dual_solution_x;
    TerminalConstraint qp_dual_solution_c;
  };

  explicit MheQpSolver(const Config& config)
      : config_(config), user_data_(config_), workspace_(config_) {}

  bool Solve();
  bool ComputeLastStateCovariance();

  const Config& config() const { return config_; }
  const UserData& user_data() const { return user_data_; }
  UserData* mutable_user_data() { return &user_data_; }
  const Workspace& workspace() const { return workspace_; }

 private:
  const Config config_;

  UserData user_data_;
  Workspace workspace_;
};

template <size_t nx, size_t nu, size_t nc>
bool MheQpSolver<nx, nu, nc>::Config::Validate() const {
  if (num_intervals == 0u) {
    return false;
  }
  return true;
}

template <size_t nx, size_t nu, size_t nc>
MheQpSolver<nx, nu, nc>::UserData::UserData(const Config& config) {
  const auto num_intervals = config.num_intervals;
  assert(num_intervals > 0);

  arrival_cost_state.setZero();
  arrival_cost_information_matrix.setZero();
  last_state_covariance.setZero();

  a_stack.resize(num_intervals, JacobianStateWrtState::Zero());
  b_stack.resize(num_intervals, JacobianStateWrtControl::Zero());
  q_stack.resize(num_intervals + 1, MatrixQ::Zero());
  r_stack.resize(num_intervals, VectorR::Zero());
  g_x_stack.resize(num_intervals + 1, State::Zero());
  g_u_stack.resize(num_intervals, Control::Zero());
  d_stack.resize(num_intervals, State::Zero());
  e.setZero();
  f.setZero();
}

template <size_t nx, size_t nu, size_t nc>
MheQpSolver<nx, nu, nc>::Workspace::Workspace(const Config& config)
    : kRegularizationFactor(
          std::sqrt(std::numeric_limits<double>::epsilon())),  // = 1.49012e-08
      kQRegularization(MatrixQ::Identity() * kRegularizationFactor) {
  const auto num_intervals = config.num_intervals;
  assert(num_intervals > 0);

  l_q_stack.resize(num_intervals + 1, MatrixQ::Zero());
  inverse_l_r_stack.resize(num_intervals, VectorR::Zero());
  big_u_stack.resize(num_intervals, MatrixQ::Zero());

  e_l_q.setZero();
  l_e.setZero();

  q_hat_stack.resize(num_intervals, State::Zero());
  r_hat_stack.resize(num_intervals, Control::Zero());
  b_hat_stack.resize(num_intervals, State::Zero());
  sigma_n.setZero();

  qp_primal_solution_x.resize(num_intervals + 1, State::Zero());
  qp_primal_solution_u.resize(num_intervals, Control::Zero());
  qp_dual_solution_x.resize(num_intervals, State::Zero());
  qp_dual_solution_c.setZero();
}

template <size_t nx, size_t nu, size_t nc>
bool MheQpSolver<nx, nu, nc>::Solve() {
  const size_t num_intervals = config_.num_intervals;

  //
  // Factorize QP KKT matrix.
  //

  const auto& q_stack = user_data_.q_stack;
  const auto& r_stack = user_data_.r_stack;
  auto& a_stack = user_data_.a_stack;
  auto& b_stack = user_data_.b_stack;
  auto& l_q_stack = workspace_.l_q_stack;
  auto& inverse_l_r_stack = workspace_.inverse_l_r_stack;
  auto& big_u_stack = workspace_.big_u_stack;

  Eigen::LLT<MatrixQ> llt_q;
  MatrixQ p_inv;
  MatrixQ big_sigma;
  big_sigma = q_stack.at(0);
  big_sigma += user_data_.arrival_cost_information_matrix;
  big_sigma += workspace_.kQRegularization;

  llt_q.compute(big_sigma);
  if (llt_q.info() != Eigen::Success) {
    DEBUG_OPTIMUS("Failed to factorize q at init!");
    return false;
  }
  l_q_stack.at(0) = llt_q.matrixL();

  inverse_l_r_stack.at(0) = r_stack.at(0).cwiseSqrt().cwiseInverse();

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
  b_stack.at(0) *= inverse_l_r_stack.at(0).asDiagonal();

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

  big_u_stack.at(0) = llt_q.matrixU();
  // U_1 = chol(Pinv)**-T
  dtrtri_("U", "N", &if_nx, big_u_stack.at(0).data(), &if_nx, &lapack_info);
  if (lapack_info) {
    DEBUG_OPTIMUS("dtrtri_ failed!");
    return false;
  }

  for (size_t node = 1; node < num_intervals; ++node) {
    big_sigma = big_u_stack.at(node - 1);
    // Sigma = U_k * U_k^T; only the upper triangular part is computed.
    dlauum_("U", &if_nx, big_sigma.data(), &if_nx, &lapack_info);
    if (lapack_info) {
      DEBUG_OPTIMUS("dlauum_ failed!");
      return false;
    }

    big_sigma.transposeInPlace();

    big_sigma += q_stack.at(node);
    big_sigma += workspace_.kQRegularization;

    llt_q.compute(big_sigma);
    if (llt_q.info() != Eigen::Success) {
      DEBUG_OPTIMUS("Failed to factorize q at node: " << node);
      return false;
    }
    l_q_stack.at(node) = llt_q.matrixL();

    inverse_l_r_stack.at(node) = r_stack.at(node).cwiseSqrt().cwiseInverse();

    // ALq_k = A_k * (Lq_k)^{-T} <=> solve ALq_k * Lq_k^T = A_k for ALq_k.
    // A_k gets overwritten by ALq_k.
    dtrsm_("R", "L", "T", "N", &if_nx, &if_nx, &if_one,
           llt_q.matrixLLT().data(), &if_nx, a_stack.at(node).data(), &if_nx);
    // BLr_k = B_k * (Lr_k)^{-T} <=> solve BLr_k * Lr_k^T = B_k for BLr_k.
    // B_k gets overwritten by BLr_k.
    b_stack.at(node) *= inverse_l_r_stack.at(node).asDiagonal();

    // Pinv = AL_k * AL_k^T; the following computes the lower triangular part.
    dsyrk_("L", "N", &if_nx, &if_nx, &if_one, a_stack.at(node).data(), &if_nx,
           &if_zero, p_inv.data(), &if_nx);
    // Pinv += BL_0 * BL_0^T; the following computes the lower triangular part.
    dsyrk_("L", "N", &if_nx, &if_nu, &if_one, b_stack.at(node).data(), &if_nx,
           &if_one, p_inv.data(), &if_nx);

    llt_q.compute(p_inv);
    if (llt_q.info() != Eigen::Success) {
      DEBUG_OPTIMUS("Failed to factorize Pinv at node: " << node);
      return false;
    }

    big_u_stack.at(node) = llt_q.matrixU();
    // U_k = chol(Pinv)**-T
    dtrtri_("U", "N", &if_nx, big_u_stack.at(node).data(), &if_nx,
            &lapack_info);
    if (lapack_info) {
      DEBUG_OPTIMUS("dtrtri_ failed!");
      return false;
    }
  }

  big_sigma = big_u_stack.at(num_intervals - 1);
  // Sigma = U_N * U_N^T; only the upper triangular part is computed.
  dlauum_("U", &if_nx, big_sigma.data(), &if_nx, &lapack_info);
  if (lapack_info) {
    DEBUG_OPTIMUS("dlauum_ failed!");
    return false;
  }

  big_sigma.transposeInPlace();
  big_sigma += q_stack.at(num_intervals);
  big_sigma += workspace_.kQRegularization;

  llt_q.compute(big_sigma);
  l_q_stack.at(num_intervals) = llt_q.matrixL();

  if constexpr (nc > 0) {
    const int if_nc = nc;
    JacobianTerminalConstraintWrtState& e_l_q = workspace_.e_l_q;
    MatrixLE& l_e = workspace_.l_e;

    // ELq_N = E * (Lq_N)^{-T} <=> solve ELq_N * Lq_0^T = E for ELq_N.
    e_l_q = user_data_.e;
    dtrsm_("R", "L", "T", "N", &if_nc, &if_nx, &if_one,
           l_q_stack.at(num_intervals).data(), &if_nx, e_l_q.data(), &if_nc);
    // Pinv = ELq_N * ELq_N^T; the following computes the lower triangular part.
    dsyrk_("L", "N", &if_nc, &if_nx, &if_one, e_l_q.data(), &if_nc, &if_zero,
           l_e.data(), &if_nc);
    // Do the in-place Cholesky decomposition.
    dpotrf_("L", &if_nc, l_e.data(), &if_nc, &lapack_info);
    if (lapack_info) {
      DEBUG_OPTIMUS("Failed to factorize Pinv for the terminal constraint!");
      return false;
    }
  }

  //
  // Solve QP KKT system.
  //

  const auto& g_x_stack = user_data_.g_x_stack;
  const auto& g_u_stack = user_data_.g_u_stack;
  const auto& d_stack = user_data_.d_stack;
  auto& q_hat_stack = workspace_.q_hat_stack;
  auto& r_hat_stack = workspace_.r_hat_stack;
  auto& b_hat_stack = workspace_.b_hat_stack;

  q_hat_stack.at(0) =
      g_x_stack.at(0) - user_data_.arrival_cost_information_matrix *
                            user_data_.arrival_cost_state;

  l_q_stack.at(0).template triangularView<Eigen::Lower>().solveInPlace(
      q_hat_stack.at(0));
  r_hat_stack.at(0) = inverse_l_r_stack.at(0).asDiagonal() * g_u_stack.at(0);

  b_hat_stack.at(0) = d_stack.at(0) - a_stack.at(0) * q_hat_stack.at(0) -
                      b_stack.at(0) * r_hat_stack.at(0);

  for (size_t node = 1; node < num_intervals; ++node) {
    const MatrixQ& big_u = big_u_stack.at(node - 1);

    q_hat_stack.at(node) =
        g_x_stack.at(node) -
        big_u.template triangularView<Eigen::Upper>() *
            (big_u.template triangularView<Eigen::Upper>().transpose() *
             b_hat_stack.at(node - 1));

    l_q_stack.at(node).template triangularView<Eigen::Lower>().solveInPlace(
        q_hat_stack.at(node));
    r_hat_stack.at(node) =
        inverse_l_r_stack.at(node).asDiagonal() * g_u_stack.at(node);

    b_hat_stack.at(node) = d_stack.at(node) -
                           a_stack.at(node) * q_hat_stack.at(node) -
                           b_stack.at(node) * r_hat_stack.at(node);
  }

  const MatrixQ& big_u_n = big_u_stack.at(num_intervals - 1);
  workspace_.sigma_n =
      g_x_stack.at(num_intervals) -
      big_u_n.template triangularView<Eigen::Upper>() *
          (big_u_n.template triangularView<Eigen::Upper>().transpose() *
           b_hat_stack.at(num_intervals - 1));

  auto& qp_x = workspace_.qp_primal_solution_x;
  auto& qp_u = workspace_.qp_primal_solution_u;
  auto& qp_lambda_x = workspace_.qp_dual_solution_x;

  if constexpr (nc == 0) {
    qp_x.at(num_intervals) =
        -l_q_stack.at(num_intervals)
             .template triangularView<Eigen::Lower>()
             .transpose()
             .solve(l_q_stack.at(num_intervals)
                        .template triangularView<Eigen::Lower>()
                        .solve(workspace_.sigma_n));
  } else {
    MatrixLE& l_e = workspace_.l_e;
    State& sigma_n = workspace_.sigma_n;
    auto& qp_dual_solution_c = workspace_.qp_dual_solution_c;

    TerminalConstraint f_hat =
        user_data_.f -
        workspace_.e_l_q * (l_q_stack.at(num_intervals)
                                .template triangularView<Eigen::Lower>()
                                .solve(sigma_n));

    qp_dual_solution_c =
        l_e.template triangularView<Eigen::Lower>().transpose().solve(
            l_e.template triangularView<Eigen::Lower>().solve(f_hat));

    qp_x.at(num_intervals) =
        -l_q_stack.at(num_intervals)
             .template triangularView<Eigen::Lower>()
             .transpose()
             .solve(l_q_stack.at(num_intervals)
                        .template triangularView<Eigen::Lower>()
                        .solve(sigma_n +
                               user_data_.e.transpose() * qp_dual_solution_c));
  }

  for (int node = num_intervals - 1; node >= 0; --node) {
    const MatrixQ& big_u = big_u_stack.at(node);

    State& current_lambda_x = qp_lambda_x.at(node);
    current_lambda_x =
        big_u.template triangularView<Eigen::Upper>() *
        (big_u.template triangularView<Eigen::Upper>().transpose() *
         (b_hat_stack.at(node) - qp_x.at(node + 1)));

    State& current_x = qp_x.at(node);
    Control& current_u = qp_u.at(node);

    current_x =
        -q_hat_stack.at(node) - a_stack.at(node).transpose() * current_lambda_x;
    current_u =
        -r_hat_stack.at(node) - b_stack.at(node).transpose() * current_lambda_x;

    l_q_stack.at(node)
        .template triangularView<Eigen::Lower>()
        .transpose()
        .solveInPlace(current_x);
    current_u = inverse_l_r_stack.at(node).asDiagonal() * current_u;
  }

  return true;
}

template <size_t nx, size_t nu, size_t nc>
bool MheQpSolver<nx, nu, nc>::ComputeLastStateCovariance() {
  const int if_nx = nx;
  const double if_one = 1.0;
  const double if_zero = 0.0;
  int lapack_info = 0;

  if constexpr (nc == 0) {
    const int if_nx = nx;
    int lapack_info;

    // Compute L_q^{-1}.
    user_data_.last_state_covariance = workspace_.l_q_stack.back();
    dtrtri_("L", "N", &if_nx, user_data_.last_state_covariance.data(), &if_nx,
            &lapack_info);
    if (lapack_info) {
      DEBUG_OPTIMUS("dtrtri_ failed!");
      return false;
    }

    // Compute the lower-triangular part of the covariance matrix.
    dlauum_("L", &if_nx, user_data_.last_state_covariance.data(), &if_nx,
            &lapack_info);
    if (lapack_info) {
      DEBUG_OPTIMUS("dlauum_ failed!");
      return false;
    }
  } else {
    static_assert(nc < nx, "This implementation is for the case nc < nx!");

    const MatrixQ z_t_q =
        user_data_.e.transpose().householderQr().householderQ();
    const Eigen::Matrix<double, nx - nc, nx> z_t_times_l_q =
        z_t_q.template block<nx, nx - nc>(0, nc).transpose() *
        workspace_.l_q_stack.back();

    const int if_nx_min_nc = nx - nc;

    // Computes the information matrix.
    dsyrk_("L", "N", &if_nx_min_nc, &if_nx, &if_one, z_t_times_l_q.data(),
           &if_nx_min_nc, &if_zero, user_data_.last_state_covariance.data(),
           &if_nx_min_nc);

    // Computes the covariance matrix.
    dpotrf_("L", &if_nx_min_nc, user_data_.last_state_covariance.data(),
            &if_nx_min_nc, &lapack_info);
    if (lapack_info) {
      DEBUG_OPTIMUS("dpotrf_ failed!");
      return false;
    }
    dpotri_("L", &if_nx_min_nc, user_data_.last_state_covariance.data(),
            &if_nx_min_nc, &lapack_info);
    if (lapack_info) {
      DEBUG_OPTIMUS("dpotri_ failed!");
      return false;
    }
  }
  return true;
}

}  // namespace optimus

#endif  // OPTIMUS_MHE_QP_SOLVER_H_
