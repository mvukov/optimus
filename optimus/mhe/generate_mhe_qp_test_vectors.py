# Copyright 2021 Milan Vukov. All rights reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
"""Generates test vectors for NMHE QP solver."""
import argparse

import casadi
import numpy

NX = 2  # #states
NU = 1  # #controls.


def generate_test_vector(num_intervals, use_arrival_cost,
                         use_terminal_constraint):
  """Generates and solves an MHE QP."""
  x = casadi.SX.sym('x', NX, num_intervals + 1)
  u = casadi.SX.sym('u', NU, num_intervals)

  q_regularization = numpy.eye(NX) * numpy.sqrt(numpy.finfo(float).eps)

  if use_arrival_cost:
    arrival_cost_state = numpy.array([[0.1], [0.2]])
    arrival_cost_information_matrix = numpy.array([[0.3, 0], [0, 0.4]])
  else:
    arrival_cost_state = numpy.array([[0.0], [0.0]])
    arrival_cost_information_matrix = numpy.array([[0.0, 0], [0, 0.0]])

  qp_q = numpy.array([[1, 0], [0, 2]])
  qp_r = numpy.array([3])

  qp_g_x = numpy.array([[0.1], [0.2]])
  qp_g_u = numpy.array([0.03])

  qp_a = numpy.array([[1, 0.25], [0, 1]])
  qp_b = numpy.array([[0], [1]])
  qp_d = numpy.array([[0.1], [0.2]])

  if use_terminal_constraint:
    qp_e = numpy.array([0.21, 0.32]).reshape((1, 2))
    qp_f = numpy.array([0.065])
  else:
    qp_e = numpy.array([])
    qp_f = numpy.array([])

  qp_cost = 0

  if use_arrival_cost:
    arrival_cost_residual = x[:, 0] - arrival_cost_state
    qp_cost += 0.5 * casadi.mtimes([
        arrival_cost_residual.T, arrival_cost_information_matrix,
        arrival_cost_residual
    ])

  for node in range(num_intervals + 1):
    qp_cost += 0.5 * casadi.mtimes(
        [x[:, node].T, qp_q + q_regularization, x[:, node]])
    qp_cost += casadi.mtimes(x[:, node].T, qp_g_x)
  for node in range(num_intervals):
    qp_cost += 0.5 * casadi.mtimes([u[:, node].T, qp_r, u[:, node]])
    qp_cost += casadi.mtimes(u[:, node].T, qp_g_u)

  qp_g = []
  for node in range(num_intervals):
    constraint = (casadi.mtimes(qp_a, x[:, node]) +
                  casadi.mtimes(qp_b, u[:, node]) + qp_d - x[:, node + 1])
    qp_g.append(constraint)

  if use_terminal_constraint:
    qp_g.append(casadi.mtimes(qp_e, x[:, -1]) + qp_f)

  qp_g = casadi.veccat(*qp_g)

  optimization_variables = casadi.veccat(x, u)

  # Turns out that solving a QP with IPOPT is more accurate than using qpOASES.
  verbose = False
  solver = casadi.nlpsol(
      'solver',
      'ipopt',
      {
          'x': optimization_variables,
          'f': qp_cost,
          'g': qp_g
      },
      {
          'ipopt': {
              'print_level': 5 if verbose else 0,  # 5 is the default value.
          },
          'error_on_fail': True,
          'print_time': verbose,
      })

  solution = solver(lbg=0, ubg=0)

  numpy.set_printoptions(precision=16)

  qp_primal_solution = numpy.asarray(solution['x'])
  qp_dual_solution = numpy.asarray(solution['lam_g'])

  num_states = NX * (num_intervals + 1)
  qp_x = qp_primal_solution[0:num_states]
  qp_u = qp_primal_solution[num_states:]
  qp_lambda = qp_dual_solution

  test_vector = (num_intervals,
                 arrival_cost_state, arrival_cost_information_matrix, qp_q,
                 numpy.diag(qp_r), qp_g_x, qp_g_u, qp_a, qp_b, qp_d, qp_e, qp_f,
                 qp_x, qp_u, qp_lambda)
  return test_vector


def main():
  parser = argparse.ArgumentParser()
  parser.add_argument('--output', type=str, required=True)
  args = parser.parse_args()

  test_range_start = 3
  test_range_end = 10

  test_vectors = []
  for num_intervals in range(test_range_start, test_range_end):
    test_vectors.append(generate_test_vector(num_intervals, False, False))
    test_vectors.append(generate_test_vector(num_intervals, True, False))

  constrained_test_vectors = []
  for num_intervals in range(test_range_start, test_range_end):
    constrained_test_vectors.append(
        generate_test_vector(num_intervals, False, True))
    constrained_test_vectors.append(
        generate_test_vector(num_intervals, True, True))

  def get_test_vector_string(test_vector):
    """Returns the test vector as a string."""
    test_vector_str = []
    for data in test_vector:
      if isinstance(data, numpy.ndarray):
        flat_data = data.reshape((-1,), order='F').ravel()
        test_vector_str.append('{{{}}}'.format(', '.join(
            [str(value) for value in flat_data])))
      else:
        test_vector_str.append(str(data))
    return '{{{}}}'.format(', '.join(test_vector_str))

  test_vector_strings = ',\n'.join(
      [get_test_vector_string(test_vector) for test_vector in test_vectors])

  constrained_test_vector_strings = ',\n'.join([
      get_test_vector_string(test_vector)
      for test_vector in constrained_test_vectors
  ])

  header = f"""
static constexpr size_t kNx = {NX};
static constexpr size_t kNu = {NU};

struct QpTestVector {{
  size_t num_intervals;
  std::vector<double> arrival_cost_state;
  std::vector<double> arrival_cost_information_matrix;
  std::vector<double> q;
  std::vector<double> r;
  std::vector<double> g_x;
  std::vector<double> g_u;

  std::vector<double> a;
  std::vector<double> b;
  std::vector<double> d;

  std::vector<double> e;
  std::vector<double> f;

  std::vector<double> expected_x;
  std::vector<double> expected_u;
  std::vector<double> expected_lambda;
}};

const std::vector<QpTestVector> qp_test_vectors = {{
{test_vector_strings}
}};

const std::vector<QpTestVector> constrained_qp_test_vectors = {{
{constrained_test_vector_strings}
}};
"""

  with open(args.output, 'w') as stream:
    stream.write(header)


if __name__ == '__main__':
  main()
