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

from examples.path_planning import py_path_planning
from examples.path_planning import utils

matplotlib.use('QtAgg')

Pose2D = py_path_planning.Pose2D


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


def run_planner(planner, obstacle_data, start, goal):
  path = []
  try:
    planner.set_grid_2d(obstacle_data)
    path = planner.plan_path(start, goal)
    print(f'Planning time: {planner.planning_time:.3f} seconds')
    print(f'Number of expansions: {planner.num_expansions}')
    print(f'Path cost: {planner.path_cost:.3f}')

  except RuntimeError as ex:
    print(ex)
  return path


def main():
  obstacle_data = utils.load_obstacle_data()

  env_config = py_path_planning.SE2Environment.Config()
  env_config.valid_state_threshold = 15

  start = Pose2D(127, 28, numpy.pi / 2)
  goal = Pose2D(146, 436, numpy.pi / 2)

  astar_planner = py_path_planning.ExampleAStarSE2Planner(env_config)
  print('Running A*')
  astar_path = run_planner(astar_planner, obstacle_data, start, goal)

  dstar_lite_planner = py_path_planning.ExampleDStarLiteSE2Planner(env_config)
  print('Running D*Lite')
  dstar_lite_path = run_planner(dstar_lite_planner, obstacle_data, start, goal)

  _, (ax1, ax2) = pyplot.subplots(1, 2, sharex=True, sharey=True)
  plot_results(astar_path, ax1, obstacle_data, color_map='Greys', title='A*')
  plot_results(dstar_lite_path,
               ax2,
               obstacle_data,
               color_map='Greys',
               title='D*Lite')

  pyplot.show()


if __name__ == '__main__':
  main()
