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
import skfmm
from matplotlib import pyplot

from examples.path_planning import py_path_planning
from examples.path_planning import utils

matplotlib.use('QtAgg')

Pose2D = py_path_planning.ExampleAStarSE2Planner.Pose2D


def create_distance_field(img: numpy.ndarray):

  def threshold_img(value):
    if value == 0:
      return 1
    return 0

  return skfmm.distance(numpy.vectorize(threshold_img)(img), dx=1)


def plot_arrow(ax: matplotlib.axes.Axes, t: Pose2D, arrow_scale: float = 0.5):
  ax.arrow(t.x,
           t.y,
           arrow_scale * numpy.cos(t.theta),
           arrow_scale * numpy.sin(t.theta),
           width=0.05)


def plot_results(path: numpy.ndarray, ax: matplotlib.axes.Axes,
                 img: numpy.ndarray, color_map: str, title: str):
  num_rows, num_cols = img.shape
  ax.imshow(img,
            cmap=pyplot.get_cmap(color_map),
            origin='lower',
            extent=[0, num_cols, 0, num_rows])

  if path:
    path_x = [p.x for p in path]
    path_y = [p.y for p in path]
    ax.plot(path_x, path_y)

    plot_arrow(ax, path[0])
    plot_arrow(ax, path[-1])

  ax.grid()
  ax.set_xlabel('x')
  ax.set_ylabel('y')
  ax.set_title(title)


def main():
  obstacle_data = utils.load_obstacle_data()

  distance_field = create_distance_field(obstacle_data)
  distance_field = distance_field.astype(numpy.uint8)

  valid_state_threshold = 15

  def threshold_distance_field(squared_value):
    if squared_value > valid_state_threshold:
      return 0
    return valid_state_threshold - squared_value

  distance_field = numpy.vectorize(threshold_distance_field)(distance_field)

  env_config = py_path_planning.SE2Environment.Config()
  env_config.valid_state_threshold = valid_state_threshold
  env_config.swath_cost_multiplier = 1.0
  env_config.length_cost_multiplier = 1.0
  env_config.abs_angle_diff_cost_multiplier = 1.0

  planner = py_path_planning.ExampleAStarSE2Planner(env_config)
  start = Pose2D(127, 28, numpy.pi / 2)
  goal = Pose2D(146, 436, numpy.pi / 2)

  try:
    print('First run, takes longer because of planner memory allocation.')
    path = planner.plan_path(distance_field, start, goal)
    print(f'Planning time: {planner.planning_time:.3f} seconds.')
    print(f'Number of expansions: {planner.num_expansions}.')

    print('Second run, takes much less time as planner memory is allocated.')
    path = planner.plan_path(distance_field, start, goal)
    print(f'Planning time: {planner.planning_time:.3f} seconds.')
    print(f'Number of expansions: {planner.num_expansions}.')

  except RuntimeError as ex:
    print(ex)
    path = []

  fig, (ax1, ax2) = pyplot.subplots(1, 2, sharex=True, sharey=True)
  plot_results(path,
               ax1,
               distance_field,
               color_map='spring',
               title='truncated distance field')
  plot_results(path, ax2, obstacle_data, color_map='Greys', title='obstacles')

  pyplot.show()


if __name__ == '__main__':
  main()
