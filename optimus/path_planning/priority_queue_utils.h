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
#ifndef OPTIMUS_PATH_PLANNING_PRIORITY_QUEUE_UTILS_H_
#define OPTIMUS_PATH_PLANNING_PRIORITY_QUEUE_UTILS_H_

#include <cmath>
#include <functional>
#include <utility>

#include "boost/heap/priority_queue.hpp"

#include "optimus/path_planning/common_utils.h"

namespace optimus {

struct Key {
  bool operator<(const Key& other) const {
    if (AreEqual(k1, other.k1)) {
      return IsLess(k2, other.k2);
    }
    return IsLess(k1, other.k1);
  }

  bool operator>(const Key& other) const {
    if (AreEqual(k1, other.k1)) {
      return IsGreater(k2, other.k2);
    }
    return IsGreater(k1, other.k1);
  }

  float k1{0};
  float k2{0};
};

struct IndexAndKey {
  IndexAndKey(int index, Key key) : index(index), key(key) {}

  int index{kInvalidIndex};
  Key key{kInfCost, kInfCost};

  bool operator>(const IndexAndKey& other) const { return key > other.key; }
};

using PriorityQueue =
    boost::heap::priority_queue<IndexAndKey,
                                boost::heap::compare<std::greater<>>>;

}  // namespace optimus

#endif  // OPTIMUS_PATH_PLANNING_PRIORITY_QUEUE_UTILS_H_
