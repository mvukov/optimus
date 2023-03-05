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
import PIL


def load_obstacle_data(resize_factor: int = 2):
  img = PIL.Image.open('external/test_data_csail/file/scsail_corrected.png')
  img = img.convert(mode='L')
  img = PIL.ImageChops.invert(img)
  img = img.resize((img.width * resize_factor, img.height * resize_factor))
  return numpy.asarray(img)
