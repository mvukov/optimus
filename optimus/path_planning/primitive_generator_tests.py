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
from optimus.path_planning import primitive_generator


class TestComputeSwath:

  def create_primitive(self, x, y):
    return primitive_generator.MotionPrimitive(length=1,
                                               x=x,
                                               y=y,
                                               theta=None,
                                               poly_coeffs=None)

  def test_when_primitive_has_duplicates_then_duplicates_are_removed(self):
    primitive = self.create_primitive(
        x=[1.2, 1.2, 2.1],
        y=[1.21, 1.21, 2.13],
    )
    swath_xy = primitive_generator.compute_swath(primitive)
    assert swath_xy == [(1, 1), (2, 2)]

  def test_when_negative_coords_then_correct_cells_callculated(self):
    primitive = self.create_primitive(
        x=[-0.1, -1.5],
        y=[-3.9, 1.5],
    )
    swath_xy = primitive_generator.compute_swath(primitive)
    assert swath_xy == [(-2, 1), (-1, -4)]


class TestSortRowWise:

  def test_when_empty_input_then_output_is_empty_array(self):
    assert primitive_generator.sort_row_wise([]) == []

  def test_when_multi_row_input_then_output_is_sorted(self):
    cells_xy = [(1, 2), (-1, 2), (0, 1), (1, -2)]
    cells_xy_sorted = primitive_generator.sort_row_wise(cells_xy)
    assert cells_xy_sorted == [(1, -2), (0, 1), (-1, 2), (1, 2)]
