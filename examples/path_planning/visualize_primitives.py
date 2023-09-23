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
import matplotlib
import numpy
from matplotlib import pyplot

from optimus.path_planning import primitive_generator

matplotlib.use('QtAgg')


def plot_primitives(primitives, title=None):
  fig, ax = pyplot.subplots()
  if title is not None:
    ax.set_title(title)
  arrow_scale = 0.5

  def plot_arrow(x, y, theta):
    pyplot.arrow(x,
                 y,
                 arrow_scale * numpy.cos(theta),
                 arrow_scale * numpy.sin(theta),
                 width=0.01)

  for primitive in primitives:
    ax.plot(primitive.x, primitive.y)

  for primitive in primitives:
    plot_arrow(primitive.x[-1], primitive.y[-1], primitive.theta[-1])

  ax.grid()
  ax.xaxis.set_major_locator(pyplot.MultipleLocator(1))
  ax.yaxis.set_major_locator(pyplot.MultipleLocator(1))
  ax.set_xlabel('x')
  ax.set_ylabel('y')
  ax.axis('equal')


def main():
  angles = primitive_generator.get_angles(grid_connectivity=32)
  print(f'Num angles: {len(angles)}')

  primitives = primitive_generator.generate_all_primitives(
      angles_and_min_coords=angles, min_radius_grid=4.95, max_angle_idx_diff=2)
  print(f'Num motion primitives: {len(primitives)}')

  plot_primitives(primitives)

  def angle_to_index(angle):
    for p in primitives:
      if numpy.isclose(p.theta[0], angle):
        return p.start_angle_idx
    raise ValueError(f'Primitives don\'t have start angle {angle}!')

  zero_angle_index = angle_to_index(0)

  def primitives_with_start_angle_index(index):
    return [p for p in primitives if p.start_angle_idx == index]

  for offset in range(0, 3):
    angle_index = zero_angle_index + offset
    plot_primitives(
        primitives_with_start_angle_index(angle_index),
        f'start angle {numpy.rad2deg(angles[angle_index].angle):.2f} deg')

  pyplot.show()


if __name__ == '__main__':
  main()
