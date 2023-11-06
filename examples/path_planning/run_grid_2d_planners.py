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
                 img: numpy.ndarray, color_map: str, title: str):
  num_rows, num_cols = img.shape
  ax.imshow(img,
            cmap=pyplot.get_cmap(color_map),
            origin='lower',
            extent=[0, num_cols, 0, num_rows])

  if path:
    path_x = [p[0] for p in path]
    path_y = [p[1] for p in path]
    ax.plot(path_x, path_y)

    def draw_circle(p, color):
      ax.add_patch(pyplot.Circle((p[0], p[1]), 2, color=color))

    draw_circle(path[0], color='tomato')
    draw_circle(path[-1], color='lime')

  ax.grid()
  ax.set_xlabel('x')
  ax.set_ylabel('y')
  ax.set_title(title)


def plan_path(planner, obstacle_data, start, goal):
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


def replan_path(planner, start, changed_positions):
  path = []
  try:
    path = planner.replan_path(start, changed_positions)
    print(f'Replanning time: {planner.planning_time:.3f} seconds')
    print(f'Number of expansions: {planner.num_expansions}')
    print(f'Path cost: {planner.path_cost:.3f}')

  except RuntimeError as ex:
    print(ex)
  return path


def main():
  obstacle_data = utils.load_obstacle_data()
  original_obstacle_data = numpy.copy(obstacle_data)

  start = [127, 28]
  goal = [146, 436]

  env_config = py_path_planning.Grid2DEnvironment.Config()
  env_config.valid_state_threshold = 15

  print('Planning')
  astar_planner = py_path_planning.AStarGrid2DPlanner(env_config)
  print('Running A*')
  astar_path = plan_path(astar_planner, obstacle_data, start, goal)

  dstar_lite_planner = py_path_planning.DStarLiteGrid2DPlanner(env_config)
  print('Running D*Lite')
  dstar_lite_path = plan_path(dstar_lite_planner, obstacle_data, start, goal)

  print('\n\nReplanning')
  # Simulate a moving robot: move a robot to a point approx. on the previously
  # found path.
  new_start = [185, 192]

  # Make the previous path infeasible.
  changed_positions = []
  for x in range(247, 249):
    for y in range(339, 353):
      obstacle_data[y, x] = 255
      changed_positions.append([x, y])

  print('Running A*')
  # A* has no replanning capability, therefore we plan from scratch.
  new_astar_path = plan_path(astar_planner, obstacle_data, new_start, goal)
  print('Running D*Lite')
  new_dstar_lite_path = replan_path(dstar_lite_planner, new_start,
                                    changed_positions)

  _, ((ax1, ax2), (ax3, ax4)) = pyplot.subplots(2, 2, sharex=True, sharey=True)
  plot_results(astar_path,
               ax1,
               original_obstacle_data,
               color_map='Greys',
               title='A*')
  plot_results(dstar_lite_path,
               ax2,
               original_obstacle_data,
               color_map='Greys',
               title='D*Lite')
  plot_results(new_astar_path,
               ax3,
               obstacle_data,
               color_map='Greys',
               title='Plan A*')
  plot_results(new_dstar_lite_path,
               ax4,
               obstacle_data,
               color_map='Greys',
               title='Replan D*Lite')
  pyplot.show()


if __name__ == '__main__':
  main()
