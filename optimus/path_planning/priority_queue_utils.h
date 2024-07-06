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
#include <vector>

#include "boost/heap/fibonacci_heap.hpp"
#include "boost/unordered/unordered_flat_map.hpp"

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

class PriorityQueue {
 public:
  using Heap =
      boost::heap::fibonacci_heap<IndexAndKey,
                                  boost::heap::compare<std::greater<>>>;
  // Assumption is that there can only be a single element with a unique index.
  using HashMap = boost::unordered::unordered_flat_map<int, Heap::handle_type>;

  auto empty() const { return heap_.empty(); }
  void clear() {
    heap_.clear();
    hash_map_.clear();
  }

  const auto& top() const { return heap_.top(); }
  void pop() {
    auto top_index = top().index;
    heap_.pop();
    hash_map_.erase(top_index);  // TODO(mvukov) The return value must be 1.
  }

  void insert(int index, Key key) {
    hash_map_[index] = heap_.emplace(index, key);
  }

  void Remove(int index) {
    auto it = hash_map_.find(index);
    if (it != hash_map_.end()) {
      heap_.erase(it->second);
      hash_map_.erase(it);
    }
  }

  void InsertOrUpdate(int index, Key key) {
    auto it = hash_map_.find(index);
    if (it != hash_map_.end()) {
      heap_.update(it->second, IndexAndKey(index, key));
    } else {
      insert(index, key);
    }
  }

  std::vector<int> GetIndices() const {
    std::vector<int> indices;
    indices.reserve(hash_map_.size());
    for (const auto& [index, _] : hash_map_) {
      indices.push_back(index);
    }
    return indices;
  }

 private:
  Heap heap_;
  HashMap hash_map_;
};

}  // namespace optimus

#endif  // OPTIMUS_PATH_PLANNING_PRIORITY_QUEUE_UTILS_H_
