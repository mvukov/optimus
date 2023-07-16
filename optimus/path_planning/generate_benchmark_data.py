# Copyright 2023 Milan Vukov. All rights reserved.
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
import argparse
import os

import PIL.Image
import python.runfiles.runfiles


def main():
  parser = argparse.ArgumentParser()
  parser.add_argument('--output_dir', type=str, required=True)
  args = parser.parse_args()
  output_dir = args.output_dir

  runfiles = python.runfiles.runfiles.Create()
  img = PIL.Image.open(
      runfiles.Rlocation('test_data_csail/file/scsail_corrected.png'))
  img = img.convert(mode='L')
  img = PIL.ImageChops.invert(img)
  resize_factor = 2
  img = img.resize((img.width * resize_factor, img.height * resize_factor))
  img.save(f'{os.path.join(output_dir, "scsail.png")}')


if __name__ == '__main__':
  main()
