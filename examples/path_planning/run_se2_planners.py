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
import numpy
import plotly.graph_objects as go

from examples.path_planning import py_path_planning
from examples.path_planning import utils

Pose2D = py_path_planning.Pose2D


def plot_results(names_to_paths: dict[str, numpy.ndarray], img: numpy.ndarray,
                 title: str):
  fig = go.Figure()
  fig.add_trace(
      go.Heatmap(z=img, colorscale='gray', reversescale=True, showscale=False))

  for path_name, path in names_to_paths.items():
    if path:

      def plot_arrow(t: Pose2D, color: str, name: str):

        fig.add_scatter(x=[t.x],
                        y=[t.y],
                        marker=dict(symbol='circle', size=10, color=color),
                        showlegend=False,
                        name=name)

        arrow_scale = 2
        fig.add_scatter(x=[t.x, t.x + arrow_scale * numpy.cos(t.theta)],
                        y=[t.y, t.y + arrow_scale * numpy.sin(t.theta)],
                        marker=dict(symbol='arrow',
                                    size=10,
                                    angleref='previous',
                                    color='black'),
                        showlegend=False)

      plot_arrow(path[0], 'tomato', 'start')
      plot_arrow(path[-1], 'lime', 'goal')

      fig.add_scatter(x=[p.x for p in path],
                      y=[p.y for p in path],
                      mode='lines',
                      name=f'{path_name} path')

  fig.update_xaxes(autorange=True, title_text='x')
  fig.update_yaxes(autorange=True, title_text='y', scaleanchor='x')
  fig.layout.title = title
  fig.show()


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

  env_config = py_path_planning.SE2Environment.Config()
  env_config.valid_state_threshold = 15

  start = Pose2D(127, 28, numpy.pi / 2)
  goal = Pose2D(146, 436, numpy.pi / 2)

  print('Planning')
  astar_planner = py_path_planning.ExampleAStarSE2Planner(env_config)
  print('Running A*')
  astar_path = plan_path(astar_planner, obstacle_data, start, goal)

  dstar_lite_planner = py_path_planning.ExampleDStarLiteSE2Planner(env_config)
  print('Running D*Lite')
  dstar_lite_path = plan_path(dstar_lite_planner, obstacle_data, start, goal)

  print('\n\nReplanning')
  # Simulate a moving robot: move a robot to a point approx. on the previously
  # found path.
  new_start = Pose2D(185, 192, numpy.pi / 4)

  # Add an additional obstacle.
  changed_positions = []
  for x in range(235, 245):
    for y in range(405, 415):
      obstacle_data[y, x] = 255
      changed_positions.append([x, y])

  print('Running A*')
  # A* has no replanning capability, therefore we plan from scratch.
  new_astar_path = plan_path(astar_planner, obstacle_data, new_start, goal)
  print('Running D*Lite')
  new_dstar_lite_path = replan_path(dstar_lite_planner, new_start,
                                    changed_positions)

  plot_results({
      'A*': astar_path,
      'D*Lite': dstar_lite_path
  }, original_obstacle_data, 'Planning')
  plot_results({
      'A*': new_astar_path,
      'D*Lite': new_dstar_lite_path
  }, obstacle_data, 'Replanning')


if __name__ == '__main__':
  main()
