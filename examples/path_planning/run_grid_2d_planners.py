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


def plot_results(names_to_paths: dict[str, numpy.ndarray], img: numpy.ndarray,
                 title: str):
  fig = go.Figure()
  fig.add_trace(
      go.Heatmap(z=img, colorscale='gray', reversescale=True, showscale=False))

  for path_name, path in names_to_paths.items():
    if path:

      def draw_circle(p: numpy.ndarray, color: str, name: str):
        fig.add_scatter(x=[p[0]],
                        y=[p[1]],
                        marker=dict(symbol='circle', size=10, color=color),
                        showlegend=False,
                        name=name)

      draw_circle(path[0], 'tomato', 'start')
      draw_circle(path[-1], 'lime', 'goal')

      fig.add_scatter(x=[p[0] for p in path],
                      y=[p[1] for p in path],
                      mode='lines',
                      name=f'{path_name} path')

  fig.update_xaxes(autorange=True, title_text='x')
  fig.update_yaxes(autorange=True, title_text='y', scaleanchor='x')
  fig.layout.title = title
  fig.show()


def plan_path(planner, obstacle_data, start, goal, callback=None):
  path = []
  try:
    planner.set_grid_2d(obstacle_data)
    path = planner.plan_path(start, goal, callback)
    print(f'Planning time: {planner.planning_time:.3f} seconds')
    print(f'Number of expansions: {planner.num_expansions}')
    print(f'Path cost: {planner.path_cost:.3f}')

  except RuntimeError as ex:
    print(ex)
  return path


def replan_path(planner, start, changed_positions, callback=None):
  path = []
  try:
    path = planner.replan_path(start, changed_positions, callback)
    print(f'Replanning time: {planner.planning_time:.3f} seconds')
    print(f'Number of expansions: {planner.num_expansions}')
    print(f'Path cost: {planner.path_cost:.3f}')

  except RuntimeError as ex:
    print(ex)
  return path


def compute_all_arastar_planner_improvements(planner, obstacle_data, start,
                                             goal):

  def callback(event):
    return event != py_path_planning.UserCallbackEvent.SOLUTION_FOUND

  names_to_paths = dict()

  def append_path(path):
    names_to_paths[f'epsilon={planner.epsilon:.2f}'] = path

  # Timings are likely to be quite off due to signifant callback overheads.
  # The main point here is to illustrate path improvements.
  path = plan_path(planner, obstacle_data, start, goal, callback)
  append_path(path)
  while True:
    path = replan_path(planner, start, [], callback)
    append_path(path)
    if numpy.isclose(planner.epsilon, 1.):
      break

  return names_to_paths


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

  arastar_config = py_path_planning.AraStarPlannerConfig()
  arastar_config.epsilon_decrease_rate = 0.05
  assert arastar_config.validate()
  arastar_planner = py_path_planning.AraStarGrid2DPlanner(
      env_config, arastar_config)
  print('Running ARA*')
  arastar_path = plan_path(arastar_planner, obstacle_data, start, goal)

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

  plot_results(
      {
          'A*': astar_path,
          'D*Lite': dstar_lite_path,
          'ARA*': arastar_path
      }, original_obstacle_data, 'Planning')
  plot_results({
      'A*': new_astar_path,
      'D*Lite': new_dstar_lite_path
  }, obstacle_data, 'Replanning')

  print('\n\nARA* improvements')
  arastar_improvement_paths = compute_all_arastar_planner_improvements(
      arastar_planner, original_obstacle_data, start, goal)
  plot_results(arastar_improvement_paths, original_obstacle_data,
               'ARA* improvements')


if __name__ == '__main__':
  main()
