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


def plot_results(path: numpy.ndarray, ax: matplotlib.axes.Axes,
                 img: numpy.ndarray, color_map: str):
  num_rows, num_cols = img.shape
  ax.imshow(img,
            cmap=pyplot.get_cmap(color_map),
            origin='lower',
            extent=[0, num_cols, 0, num_rows])

  if path:
    path_x = [p[0] for p in path]
    path_y = [p[1] for p in path]
    ax.plot(path_x, path_y)

  ax.grid()
  ax.set_xlabel('x')
  ax.set_ylabel('y')


def main():
  obstacle_data = utils.load_obstacle_data()

  env_config = py_path_planning.Grid2DEnvironment.Config()
  env_config.valid_state_threshold = 15

  planner = py_path_planning.DStarLiteGrid2DPlanner(env_config)
  start = [127, 28]
  goal = [146, 436]

  try:
    path = planner.plan_path(obstacle_data.astype(numpy.uint8), start, goal)
    print(f'Planning time: {planner.planning_time:.3f} seconds.')
    print(f'Number of expansions: {planner.num_expansions}.')
    print(f'Path cost: {planner.path_cost}.')

  except RuntimeError as ex:
    print(ex)
    path = []

  fig, ax = pyplot.subplots(1, 1)
  plot_results(path, ax, obstacle_data, color_map='Greys')
  pyplot.show()


if __name__ == '__main__':
  main()
