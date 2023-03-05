// Copyright 2021 Milan Vukov. All rights reserved.
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
#ifndef OPTIMUS_DEBUG_H_
#define OPTIMUS_DEBUG_H_

#ifdef OPTIMUS_DEBUG
#include <iostream>

#define OPTIMUS_PRINT(stuff)                                      \
  std::cerr << "[OPTIMUS] " << __FILE__ << ":" << __LINE__ << ":" \
            << __FUNCTION__ << ": " << stuff << std::endl;

#else  // OPTIMUS_DEBUG
#define OPTIMUS_PRINT(stuff)
#endif  // OPTIMUS_DEBUG

#endif  // OPTIMUS_DEBUG_H_
