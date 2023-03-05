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
from typing import Tuple

import numpy


def to_rotation_matrix(angle: float) -> numpy.ndarray:
  sin_angle = numpy.sin(angle)
  cos_angle = numpy.cos(angle)
  return numpy.asarray([[cos_angle, -sin_angle], [sin_angle, cos_angle]])


def get_angle_diff(angle1: float, angle2: float) -> float:
  ref_r_angle1 = to_rotation_matrix(angle1)
  ref_r_angle2 = to_rotation_matrix(angle2)
  angle1_r_angle2 = ref_r_angle1.T @ ref_r_angle2
  return numpy.arctan2(angle1_r_angle2[1, 0], angle1_r_angle2[0, 0])


def to_reflection_tf(angle: float) -> numpy.ndarray:
  cos_2angle = numpy.cos(2.0 * angle)
  sin_2angle = numpy.sin(2.0 * angle)
  return numpy.asarray([[cos_2angle, sin_2angle, 0],
                        [sin_2angle, -cos_2angle, 0], [0, 0, 1]])


def to_tf(x: float, y: float, theta: float) -> numpy.ndarray:
  cos_theta = numpy.cos(theta)
  sin_theta = numpy.sin(theta)
  return numpy.asarray([[cos_theta, -sin_theta, x], [sin_theta, cos_theta, y],
                        [0, 0, 1]])


def from_tf(tf: numpy.ndarray) -> Tuple[float, float, float]:
  return tf[0, 2], tf[1, 2], numpy.arctan2(tf[1, 0], tf[0, 0])


def get_distance_to_line(a: numpy.ndarray, b: numpy.ndarray,
                         c: numpy.ndarray) -> float:
  v = b - a
  t = numpy.dot(c - a, v) / numpy.dot(v, v)
  d = a + t * v
  return numpy.linalg.norm(d - c)
