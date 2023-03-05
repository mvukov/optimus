// Copyright 2022 Milan Vukov. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
#ifndef OPTIMUS_CHECK_H_
#define OPTIMUS_CHECK_H_

namespace optimus::internal {

void PrintAndDie(const char* file, int line, const char* function,
                 const char* expression);

}  // namespace optimus::internal

#define OPTIMUS_CHECK(expression)                                      \
  do {                                                                 \
    if (!(expression)) {                                               \
      optimus::internal::PrintAndDie(__FILE__, __LINE__, __FUNCTION__, \
                                     #expression);                     \
    }                                                                  \
  } while (false)

#endif  // OPTIMUS_CHECK_H_
