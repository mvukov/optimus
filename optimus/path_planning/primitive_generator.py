# Copyright 2022 Milan Vukov. All rights reserved.
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
""" Implements a set of utilities for 2D motion primitive generation.

Important literature:
[1] "Differentially Constrained Mobile Robot Motion Planning in State Lattices"
  by Mihail Pivtoraiko, Ross A. Knepper, and Alonzo Kelly
  https://rpal.cs.cornell.edu/docs/PivEtal_JFR_2009.pdf
[2] "Adaptive Model-Predictive Motion Planning for Navigation in Complex
  Environments" by Thomas M. Howard
  https://www.ri.cmu.edu/pub_files/2009/8/howard_final_dissertation.pdf
"""
import dataclasses
import logging
from typing import List
from typing import Optional
from typing import Tuple

import casadi
import numpy.polynomial

from optimus.path_planning import geometry

SxType = casadi.casadi.SX
FunctionType = casadi.casadi.Function

EPS = numpy.finfo(numpy.float64).eps
NUM_POLY_COEFFS = 4
CELL_OFFSET = 0.5

logger = logging.getLogger(__name__)


def get_cubic_poly(s_init, s_final, cubic_spline_coeffs):
  """ See [2], Eq. 4.15 - 4.19.
  Pay attention to the fact that this method is much more numerically stable.
  """
  k0, k1, k2, k3 = (cubic_spline_coeffs[0], cubic_spline_coeffs[1],
                    cubic_spline_coeffs[2], cubic_spline_coeffs[3])
  a = k0
  s_diff = s_final - s_init
  b = -0.5 * (-2 * k3 + 11 * k0 - 18 * k1 + 9 * k2) / s_diff
  c = 4.5 * (-k3 + 2 * k0 - 5 * k1 + 4 * k2) / s_diff**2
  d = -4.5 * (-k3 + k0 - 3 * k1 + 3 * k2) / s_diff**3
  return a, b, c, d


def get_curvature(s, s_init, s_final, cubic_spline_coeffs):
  a, b, c, d = get_cubic_poly(s_init, s_final, cubic_spline_coeffs)
  k = a + b * s + c * s**2 + d * s**3
  return k


@dataclasses.dataclass(frozen=True)
class MotionModel:
  s: SxType
  s_init: SxType
  s_final: SxType
  p: SxType
  x: SxType
  dot_x: SxType
  q: SxType


def create_motion_model(velocity: float) -> MotionModel:
  """ [1] and [2] combined. This simple equation of motion can be found in e.g.
  [1], Eq, 2 and 3. Defines a polynomial spiral.
  """
  if numpy.isclose(velocity, 0):
    raise ValueError('velocity must be != 0!')

  p_x = casadi.SX.sym('p_x')
  p_y = casadi.SX.sym('p_y')
  theta = casadi.SX.sym('theta')
  x = casadi.vertcat(p_x, p_y, theta)

  s = casadi.SX.sym('s')
  s_init = casadi.SX.sym('s_init')
  s_final = casadi.SX.sym('s_final')
  p = casadi.SX.sym('p', NUM_POLY_COEFFS)
  k = get_curvature(s, s_init, s_final, p)

  dot_x = casadi.vertcat(velocity * casadi.cos(theta),
                         velocity * casadi.sin(theta), velocity * k)
  q = casadi.vertcat(k**2)

  return MotionModel(s, s_init, s_final, p, x, dot_x, q)


def create_integrator(model: MotionModel) -> FunctionType:
  """ Implements an explicit RK4 integrator.
  """
  ode = casadi.Function(
      'ode', [model.s, model.s_init, model.s_final, model.p, model.x],
      [model.dot_x, model.q])
  s = casadi.SX.sym('s_int')
  s_init = casadi.SX.sym('s_init_int')
  s_final = casadi.SX.sym('s_final_int')
  p = casadi.SX.sym('p_int', model.p.size())
  x = casadi.SX.sym('x_int', model.x.size())
  step = casadi.SX.sym('step_int')
  q = 0

  k1, k1_q = ode(s, s_init, s_final, p, x)
  k2, k2_q = ode(s + step / 2, s_init, s_final, p, x + step / 2 * k1)
  k3, k3_q = ode(s + step / 2, s_init, s_final, p, x + step / 2 * k2)
  k4, k4_q = ode(s + step, s_init, s_final, p, x + step * k3)
  x_int = x + step / 6 * (k1 + 2 * k2 + 2 * k3 + k4)
  q_int = q + step / 6 * (k1_q + 2 * k2_q + 2 * k3_q + k4_q)

  return casadi.Function('integrator', [s, s_init, s_final, p, x, step],
                         [x_int, q_int],
                         ['s', 's_init', 's_final', 'p', 'x', 'step'],
                         ['x_int', 'q_int'])


@dataclasses.dataclass(frozen=True)
class MotionPrimitive:
  """ The motion primitive.

  x and y arrays are grid-lower-left-edge centered.
  x, y, theta arrays are of the same length.
  """

  length: float  # The motion primitive length.
  x: numpy.ndarray  # x-axis primitive samples.
  y: numpy.ndarray  # y-axis primitive samples.
  theta: numpy.ndarray  # Angle samples of the primitive.
  poly_coeffs: numpy.ndarray  # The polynomial coeffs.


class MotionPrimitiveGenerator:
  """ Implements a motion primitive generator/solver.

  In essence, here we form a parametric NLP, in the constructor, and then
  solve the NLP for different start and end conditions (parameters).

  The NLP utilizes multiple-shooting tech for discretization. Once again, using
  the tech in get_cubic_poly allows us to use crap initial guess and avoid
  numerical issue.
  """

  @dataclasses.dataclass(frozen=True)
  class Settings:
    num_discretization_intervals: int = 25
    forward: bool = True
    max_num_iterations: int = 15
    verbose: bool = False

  def __init__(self, settings: Settings):
    self._settings = settings

    velocity = 1 if self._settings.forward else -1
    self._model = create_motion_model(velocity)
    self._integrator = create_integrator(self._model)

    w = []  # Optimization variables.
    objective = 0
    g = []  # Constraints.

    s_final = casadi.SX.sym('s_final')
    w += [s_final]

    p = casadi.SX.sym('p', self._model.p.size())
    w += [p]

    x_size = self._model.x.size()
    x_k = casadi.SX.sym('x_0', x_size)
    w += [x_k]

    num_discretization_intervals = self._settings.num_discretization_intervals
    step = s_final / num_discretization_intervals
    s_init = 0

    for kk in range(num_discretization_intervals):
      s_k = kk * step
      f_k = self._integrator(s=s_k,
                             s_init=s_init,
                             s_final=s_final,
                             p=p,
                             x=x_k,
                             step=step)
      x_k_int = f_k['x_int']
      q_k_int = f_k['q_int']

      if kk < num_discretization_intervals - 1:
        objective += q_k_int
      else:
        objective += 100 * q_k_int

      x_k = casadi.SX.sym('x_{}'.format(kk + 1), x_size)
      w += [x_k]
      g += [x_k_int - x_k]

    # Create an NLP solver.
    nlp = {'f': objective, 'x': casadi.vertcat(*w), 'g': casadi.vertcat(*g)}
    verbose = self._settings.verbose
    self._solver = casadi.nlpsol(
        'solver',
        'ipopt',
        nlp,
        {
            'ipopt': {
                'max_iter': self._settings.max_num_iterations,
                'print_level': 5 if verbose else 0,  # 5 is the default value.
            },
            'error_on_fail': True,
            'print_time': verbose,
        })

  def solve(self, p_x_start: float, p_y_start: float, theta_start: float,
            p_x_end: float, p_y_end: float,
            theta_end: float) -> MotionPrimitive:
    """ Solves the parametric NLP with the given start/end conditions.
    """
    lbw = []
    ubw = []
    w_init = []
    lbg = []
    ubg = []

    num_discretization_intervals = self._settings.num_discretization_intervals

    p_x_init = numpy.linspace(p_x_start, p_x_end,
                              num_discretization_intervals + 1)
    p_y_init = numpy.linspace(p_y_start, p_y_end,
                              num_discretization_intervals + 1)
    theta_init = numpy.linspace(theta_start, theta_end,
                                num_discretization_intervals + 1)

    xy_distance = numpy.sqrt((p_x_end - p_x_start)**2 +
                             (p_y_end - p_y_start)**2)
    lbw += [xy_distance]
    ubw += [xy_distance * 2]
    w_init += [xy_distance]

    inf = casadi.inf
    lbw += [0, -inf, -inf, -inf]
    ubw += [0, inf, inf, inf]
    w_init += [0, 0.1, 0.1, 0.1]

    lbw += [p_x_start, p_y_start, theta_start]
    ubw += [p_x_start, p_y_start, theta_start]
    w_init += [p_x_init[0], p_y_init[0], theta_init[0]]

    x_size = self._model.x.size()
    for kk in range(num_discretization_intervals):
      if kk < num_discretization_intervals - 1:
        lbw += [-inf, -inf, -inf]
        ubw += [inf, inf, inf]
      else:
        lbw += [p_x_end, p_y_end, theta_end]
        ubw += [p_x_end, p_y_end, theta_end]

      w_init += [p_x_init[kk + 1], p_y_init[kk + 1], theta_init[kk + 1]]

      lbg += numpy.zeros(x_size)
      ubg += numpy.zeros(x_size)

    solution = self._solver(x0=w_init, lbx=lbw, ubx=ubw, lbg=lbg, ubg=ubg)
    w_opt = solution['x'].full().flatten()

    s_final = w_opt[0]
    p = w_opt[1:5]
    p_x = w_opt[5::3]
    p_y = w_opt[6::3]
    theta = w_opt[7::3]

    return MotionPrimitive(s_final, p_x, p_y, theta, p)


@dataclasses.dataclass(frozen=True)
class AngleAndCoord:
  angle: float
  x: float
  y: float


def get_angles(grid_connectivity: int) -> List[AngleAndCoord]:
  """ Given the grid connectivity value, returns a list of angles on the grid
      together with their xy-coords.

  Args:
    grid_connectivity: The grid connectivity, must be >= 8 and power of 2.
  Returns:
    A sorted list of angles and their associated xy-coords.
    The angles are in the range (-pi, pi].
  """
  log_connectivity = numpy.log2(grid_connectivity)
  if grid_connectivity < 8 or 2**log_connectivity != grid_connectivity:
    raise ValueError('grid_connectivity must be a power of 2 and >= 8!')

  visited_angles = []

  def is_visited(angle: float) -> bool:
    for visited_angle in visited_angles:
      if numpy.isclose(visited_angle, angle):
        return True
    return False

  max_coord = int(log_connectivity) - 2
  angles_and_coords = []
  for m in range(1, max_coord + 1):
    for x in range(-m, m + 1):
      for y in range(-m, m + 1):
        if x == 0 and y == 0:
          continue
        angle = numpy.arctan2(y, x)
        if numpy.isclose(angle, -numpy.pi):
          angle = numpy.pi

        if not is_visited(angle):
          visited_angles.append(angle)
          angles_and_coords.append(AngleAndCoord(angle, x, y))

  return sorted(angles_and_coords, key=lambda x: x.angle)


def get_cubic_poly_extrema(primitive: MotionPrimitive) -> numpy.ndarray:
  length = primitive.length
  poly = numpy.polynomial.Polynomial(
      get_cubic_poly(0, length, primitive.poly_coeffs))
  poly_deriv = poly.deriv()
  deriv_roots = poly_deriv.roots()
  valid_deriv_roots = []
  for root in deriv_roots:
    if numpy.iscomplex(root) or root <= 0 or root > length:
      continue
    valid_deriv_roots.append(root)
  return poly(numpy.asarray(valid_deriv_roots))


def generate_primitive(
    angle_and_min_coord_start: AngleAndCoord,
    angle_and_min_coord_end: AngleAndCoord, min_radius_grid: float,
    generator: MotionPrimitiveGenerator) -> Optional[MotionPrimitive]:
  """ Generates a motion primitive.

  Args:
    angle_and_min_coord_start: The start angle and min corresponding coords.
    angle_and_min_coord_end: The end angle and min corresponding coords.
    min_radius_grid: The min radius of the primitive in the grid frame.
    generator: The motion primitive generator.
  Returns:
    The motion primitive on success, None otherwise.
  """
  if min_radius_grid <= 0:
    raise ValueError('min_radius_grid must be >= 0!')

  # For now assume the velocity is positive.
  max_k = 1. / min_radius_grid
  max_coord_inf_norm = numpy.ceil(min_radius_grid).astype(int) * 3

  theta_start = angle_and_min_coord_start.angle
  theta_end = angle_and_min_coord_end.angle

  theta_diff = theta_end - theta_start
  if abs(theta_diff) >= numpy.pi and theta_start * theta_end < 0:
    if theta_start > 0:
      theta_end += 2 * numpy.pi
    else:
      theta_end -= 2 * numpy.pi

  def generate_primitive_local(theta_start, end_x, end_y, theta_end):
    return generator.solve(0, 0, theta_start, end_x, end_y, theta_end)

  if numpy.isclose(theta_start, theta_end):
    _x = angle_and_min_coord_start.x
    _y = angle_and_min_coord_start.y
    return MotionPrimitive(length=numpy.hypot(_x, _y),
                           x=numpy.asarray([0, _x]),
                           y=numpy.asarray([0, _y]),
                           theta=numpy.asarray([theta_start, theta_start]),
                           poly_coeffs=numpy.zeros(NUM_POLY_COEFFS,))

  start_dir = [numpy.cos(theta_start), numpy.sin(theta_start)]
  end_dir = [numpy.cos(theta_end), numpy.sin(theta_end)]
  start_dir_cross_end_dir = numpy.cross(start_dir, end_dir)
  # TODO(mvukov) min_coord_inf_norm produces a bit different results.
  # To be further evaluated.
  min_coord_inf_norm = max(min_radius_grid - 1, 1)

  xy_pairs = set()
  for x in range(-max_coord_inf_norm, max_coord_inf_norm + 1):
    for y in range(-max_coord_inf_norm, max_coord_inf_norm + 1):
      # Down here is a number of heuristics used to speed up the search.

      if x == 0 and y == 0:
        continue
      if numpy.linalg.norm([x, y], ord=numpy.inf) < min_coord_inf_norm:
        continue
      xy_dir = [x, y]
      if numpy.isclose(numpy.cross(start_dir, xy_dir), 0):
        continue
      if numpy.cross(start_dir, xy_dir) * start_dir_cross_end_dir < 0:
        continue
      if numpy.dot(end_dir, xy_dir) < -EPS:
        continue
      xy_pairs.add((x, y))

  xy_pairs = sorted(
      xy_pairs,
      key=lambda xy: (xy[0]**2 + xy[1]**2, numpy.dot(start_dir, xy)),
  )

  for x, y in xy_pairs:
    try:
      primitive = generate_primitive_local(theta_start, x, y, theta_end)
      extrema = get_cubic_poly_extrema(primitive)
      candidate_max_k = numpy.max(numpy.abs(extrema))
      if candidate_max_k <= max_k:
        return primitive

    except RuntimeError:
      pass
  logging.warning(
      f'Failed to generate primitive from start angle {theta_start} '
      f'to end angle {theta_end}')
  return None


@dataclasses.dataclass(frozen=True)
class MotionPrimitiveExt:
  """ Implements an extended motion primitive structure.

  The structure is suitable for export to a search algorithm interface. See also
  MotionPrimitive class.

  start_angle_idx and end_angle_idx are the ones obtained from get_angles
  function.
  """

  length: float
  x: numpy.ndarray
  y: numpy.ndarray
  theta: numpy.ndarray
  start_angle_idx: int
  end_angle_idx: int


def create_extended_straight_primitives(
    primitives: List[MotionPrimitiveExt]) -> List[MotionPrimitiveExt]:
  """ See http://sbpl.net/node/53 "Forward Arc Motion Primitives".

  Straight extensions help getting more straight paths with search algorithms.
  """
  start_angle_indices_to_primitive_indices = {}
  for idx, p in enumerate(primitives):
    start_angle_idx = p.start_angle_idx
    if p.start_angle_idx not in start_angle_indices_to_primitive_indices:
      start_angle_indices_to_primitive_indices[start_angle_idx] = []
    start_angle_indices_to_primitive_indices[start_angle_idx].append(idx)

  origin_xy = numpy.zeros((2,))
  extended_primitives = []
  for (start_angle_idx,
       indices) in start_angle_indices_to_primitive_indices.items():
    start_angle = primitives[indices[0]].theta[0]
    normal_dir_angle = start_angle + numpy.pi / 2
    normal_dir = numpy.asarray(
        [numpy.cos(normal_dir_angle),
         numpy.sin(normal_dir_angle)])

    max_distance_to_normal = 0
    straight_primitive_idx = -1
    for idx in indices:
      primitive = primitives[idx]
      if primitive.start_angle_idx == primitive.end_angle_idx:
        straight_primitive_idx = idx
      num_samples = primitive.x.size
      for el in range(1, num_samples):
        xy = numpy.asarray([primitive.x[el], primitive.y[el]])
        distance_to_normal = geometry.get_distance_to_line(
            origin_xy, normal_dir, xy)
        max_distance_to_normal = numpy.max(
            [max_distance_to_normal, distance_to_normal])

    straight_primitive = primitives[straight_primitive_idx]
    x_end, y_end = straight_primitive.x[-1], straight_primitive.y[-1]
    extended_x_end, extended_y_end = 0, 0
    while True:
      new_x_end = extended_x_end + x_end
      new_y_end = extended_y_end + y_end
      if numpy.hypot(new_x_end, new_y_end) <= max_distance_to_normal + 1e-3:
        extended_x_end = new_x_end
        extended_y_end = new_y_end
      else:
        break
    # TODO(mvukov) Good results can be obtained if we allow extensions just one
    # x_end, y_end longer than max_distance_to_normal.

    theta = straight_primitive.theta[0]
    extended_primitives.append(
        MotionPrimitiveExt(length=numpy.hypot(extended_x_end, extended_y_end),
                           x=numpy.asarray([0, extended_x_end]),
                           y=numpy.asarray([0, extended_y_end]),
                           theta=numpy.asarray([theta, theta]),
                           start_angle_idx=start_angle_idx,
                           end_angle_idx=start_angle_idx))
  return extended_primitives


def reflect_primitive(theta: float, primitive: MotionPrimitiveExt,
                      angles: List[float]) -> MotionPrimitiveExt:
  reflection = geometry.to_reflection_tf(theta)

  num_samples = len(primitive.x)
  tfs = numpy.zeros((num_samples, 3, 3))
  for el in range(num_samples):
    tfs[el, :, :] = geometry.to_tf(primitive.x[el], primitive.y[el],
                                   primitive.theta[el])

  reflected_tfs = numpy.zeros((num_samples, 3, 3))
  for el in range(num_samples):
    reflected_tfs[el, :, :] = reflection @ tfs[el, :, :]

  reflected_x = numpy.zeros(num_samples)
  reflected_y = numpy.zeros(num_samples)
  reflected_theta = numpy.zeros(num_samples)

  for el in range(num_samples):
    x, y, theta = geometry.from_tf(reflected_tfs[el, :, :])
    reflected_x[el] = x
    reflected_y[el] = y
    reflected_theta[el] = theta

  def find_angle_idx(angle: float):
    if numpy.isclose(angle, -numpy.pi):
      angle = numpy.pi
    for el, candidate_angle in enumerate(angles):
      if numpy.isclose(angle, candidate_angle):
        return el
    raise RuntimeError(f'Failed to find index for angle {angle}!')

  return MotionPrimitiveExt(primitive.length, reflected_x, reflected_y,
                            reflected_theta, find_angle_idx(reflected_theta[0]),
                            find_angle_idx(reflected_theta[-1]))


def remove_duplicates(
    primitives: List[MotionPrimitiveExt]) -> List[MotionPrimitiveExt]:
  primitives_dict = {
      (p.start_angle_idx, p.end_angle_idx, p.length): p for p in primitives
  }
  return primitives_dict.values()


def sort_primitives(
    angles: List[AngleAndCoord],
    primitives: List[MotionPrimitiveExt]) -> List[MotionPrimitiveExt]:
  num_angles = len(angles)
  sorted_primitives = []
  for start_angle_idx in range(num_angles):
    current_primitives = []
    current_end_angle_indices = []

    for p in primitives:
      if p.start_angle_idx != start_angle_idx:
        continue
      current_primitives.append(p)
      current_end_angle_indices.append(p.end_angle_idx)
    current_end_angles = numpy.unwrap(
        [angles[el] for el in current_end_angle_indices])
    sorted_current_primitives = sorted(
        current_primitives,
        key=lambda p: current_end_angles[current_end_angle_indices.index(
            p.end_angle_idx)])
    sorted_primitives.extend(sorted_current_primitives)
  return sorted_primitives


def shift_primitives(
    primitives: List[MotionPrimitiveExt]) -> List[MotionPrimitiveExt]:
  shifted_primitives = []
  for p in primitives:
    shifted_primitives.append(
        MotionPrimitiveExt(p.length, p.x + CELL_OFFSET, p.y + CELL_OFFSET,
                           p.theta, p.start_angle_idx, p.end_angle_idx))
  return shifted_primitives


# V1: forward only motions. Implemented.
# V2: add bool flag for reverse motions.
# V3: add bool flag for in-place rotations.
def generate_all_primitives(
    angles_and_min_coords: List[AngleAndCoord], min_radius_grid: float,
    max_angle_idx_diff: int) -> List[MotionPrimitiveExt]:
  """ Generates motion primitives given the list of angles.

  Args:
    angles_and_min_coords: the list of angles. At the moment assumes this is an
    ouput of get_angles function.
    min_radius_grid: The minimum primitive radius in the grid frame.
    max_angle_idx_diff: Defines the maximum start-to-end angle deviation for a
    primitive via the difference of indices on angles_and_min_coords array.
    TODO(mvukov) Defining this via max angle diff directly could be more
    intuitive.
  Returns:
    The list of primitives. The origin of xy-coords is at
    (CELL_OFFSET, CELL_OFFSET), i.e. at the center of the grid cell (0, 0).
  """
  if min_radius_grid < 0:
    raise ValueError('min_radius_grid must be >= 0!')
  if max_angle_idx_diff < 0:
    raise ValueError('max_angle_idx_diff must be >= 0!')

  settings = MotionPrimitiveGenerator.Settings()
  generator = MotionPrimitiveGenerator(settings)

  num_angles = len(angles_and_min_coords)
  primitives = []

  def append_primitive(primitive, start_angle_idx, end_angle_idx):
    if primitive is not None:
      primitives.append(
          MotionPrimitiveExt(primitive.length, primitive.x, primitive.y,
                             primitive.theta, start_angle_idx, end_angle_idx))

  def generate_primitive_local(start_angle_idx, end_angle_idx):
    return generate_primitive(angles_and_min_coords[start_angle_idx],
                              angles_and_min_coords[end_angle_idx],
                              min_radius_grid, generator)

  # TODO(mvukov) Based on angle discretization, the limit angle in the first
  # quadrant can be reduced even more -> faster processing.
  pi_quarter = numpy.pi / 4
  for start_angle_idx in range(num_angles):
    start_angle = angles_and_min_coords[start_angle_idx].angle
    if start_angle < 0 or start_angle > pi_quarter:
      continue

    if numpy.isclose(start_angle, 0):
      for el_diff in range(0, max_angle_idx_diff + 1):
        end_angle_idx = (start_angle_idx + el_diff) % num_angles
        primitive = generate_primitive_local(start_angle_idx, end_angle_idx)
        append_primitive(primitive, start_angle_idx, end_angle_idx)

    elif numpy.isclose(start_angle, pi_quarter):
      for el_diff in range(-max_angle_idx_diff, 1):
        end_angle_idx = (start_angle_idx + el_diff) % num_angles
        primitive = generate_primitive_local(start_angle_idx, end_angle_idx)
        append_primitive(primitive, start_angle_idx, end_angle_idx)

    else:
      for el_diff in range(-max_angle_idx_diff, max_angle_idx_diff + 1):
        end_angle_idx = (start_angle_idx + el_diff) % num_angles
        primitive = generate_primitive_local(start_angle_idx, end_angle_idx)
        append_primitive(primitive, start_angle_idx, end_angle_idx)

  primitives.extend(create_extended_straight_primitives(primitives))

  angles = [el.angle for el in angles_and_min_coords]
  primitives.extend(
      [reflect_primitive(pi_quarter, p, angles) for p in primitives])
  primitives.extend([reflect_primitive(0, p, angles) for p in primitives])
  primitives.extend(
      [reflect_primitive(numpy.pi / 2, p, angles) for p in primitives])

  primitives = remove_duplicates(primitives)
  primitives = sort_primitives(angles, primitives)
  primitives = shift_primitives(primitives)
  return primitives


def to_grid_coord(coord: float) -> int:
  return int(numpy.floor(coord))


def split_primitive_xy(
    primitive: MotionPrimitiveExt) -> Tuple[numpy.ndarray, numpy.ndarray]:
  x = numpy.asarray(primitive.x)
  y = numpy.asarray(primitive.y)
  if x.shape != y.shape:
    raise ValueError('x and y arrays must be of the same shape!')
  if x.ndim != 1 or x.size == 1:
    raise ValueError('x must be flat and must have at least 2 elements!')
  length = primitive.length
  if numpy.isclose(length, 0):
    raise ValueError('length must be larger than zero!')
  if x.size > 2 or numpy.isclose(length, 1):
    return x, y
  num_points = numpy.ceil(length).astype(int) + 1
  split_x = numpy.linspace(x[0], x[1], num_points)
  split_y = numpy.linspace(y[0], y[1], num_points)
  return split_x, split_y


ListPairOfInts = List[Tuple[int, int]]


def compute_swath(primitive: MotionPrimitiveExt) -> ListPairOfInts:
  """ At the moment, this just discretizes the path.

  TODO(mvukov) Include vehicle geometry in this computation.
  """
  x, y = split_primitive_xy(primitive)
  cells_x = numpy.floor(x).astype(int)
  cells_y = numpy.floor(y).astype(int)
  cells_xy = list(set(zip(cells_x, cells_y)))  # Compute unique cells.
  return cells_xy


def sort_row_wise(cells_xy: ListPairOfInts) -> ListPairOfInts:
  if not cells_xy:
    return []
  cells_x, _ = zip(*cells_xy)
  min_x = min(cells_x)
  max_x = max(cells_x)
  width = max_x - min_x
  return sorted(cells_xy, key=lambda el: el[1] * width + (el[0] - min_x))
