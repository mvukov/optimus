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
#ifndef OPTIMUS_TESTING_EIGEN_MATCHERS_H_
#define OPTIMUS_TESTING_EIGEN_MATCHERS_H_

#include <type_traits>
#include <vector>

#include "Eigen/Core"
#include "gmock/gmock.h"
#include "gtest/gtest.h"

namespace optimus {

template <typename T>
class EigenNearMatcher {
 public:
  using is_gtest_matcher = void;

  using Scalar = typename T::Scalar;
  using Index = typename T::Index;

  EigenNearMatcher(T expected, Scalar tolerance)
      : expected_(expected), tolerance_(tolerance) {}

  template <typename U>
  bool MatchAndExplain(const Eigen::MatrixBase<U>& arg,
                       std::ostream* listener) const {
    using Arg = Eigen::MatrixBase<U>;
    static_assert(std::is_same<typename T::Index, typename Arg::Index>::value,
                  "Matrices have different Index types.");
    static_assert(std::is_same<typename T::Scalar, typename Arg::Scalar>::value,
                  "Matrices have different Scalar types.");

    bool dims_ok = true;
    if (expected_.rows() != arg.rows()) {
      dims_ok = false;
      if (listener != nullptr) {
        *listener << " the number of rows is not the same";
      }
    }
    if (expected_.cols() != arg.cols()) {
      dims_ok = false;
      if (listener != nullptr) {
        *listener << " the number of cols is not the same";
      }
    }
    if (!dims_ok) {
      return false;
    }

    bool equal = true;
    std::stringstream errors;
    for (Index row = 0; row < arg.rows(); ++row) {
      for (Index col = 0; col < arg.cols(); ++col) {
        const Scalar expected_value = expected_(row, col);
        const Scalar actual_value = arg(row, col);
        const Scalar error = std::abs(actual_value - expected_value);

        if (error > tolerance_) {
          equal = false;
          if (listener != nullptr) {
            *listener << std::endl
                      << "matrices differ at (" << row << ",  " << col << "): "
                      << " expected " << expected_value << ", got "
                      << actual_value << ", abs(diff)="
                      << std::abs(expected_value - actual_value);
          }
        }
      }
    }
    return equal;
  }

  void DescribeTo(std::ostream* os) const {
    *os << "is equal to:\n" << expected_;
  }

  void DescribeNegationTo(std::ostream* os) const {
    *os << "is not equal to:\n" << expected_;
  }

 private:
  T expected_;
  Scalar tolerance_;
};

template <typename T>
inline ::testing::Matcher<T> EigenEq(const T& expected) {
  return EigenNearMatcher<T>(expected, typename T::Scalar(0));
}

template <typename T>
inline auto EigenEq(const std::vector<T>& expected) {
  std::vector<::testing::Matcher<T>> matchers;
  for (const auto& e : expected) {
    matchers.emplace_back(EigenNearMatcher<T>(e, typename T::Scalar(0)));
  }
  return ::testing::ElementsAreArray(matchers);
}

template <typename T>
inline ::testing::Matcher<T> EigenNear(T expected,
                                       typename T::Scalar tolerance) {
  return EigenNearMatcher<T>(expected, tolerance);
}

template <typename PlainObjectType, int MapOptions, typename StrideType>
inline ::testing::Matcher<std::remove_cv_t<PlainObjectType>> EigenNear(
    const Eigen::Map<PlainObjectType, MapOptions, StrideType> expected,
    typename PlainObjectType::Scalar tolerance) {
  return EigenNearMatcher<std::remove_cv_t<PlainObjectType>>(expected,
                                                             tolerance);
}

template <typename T>
inline auto EigenNear(const std::vector<T>& expected,
                      typename T::Scalar tolerance) {
  std::vector<::testing::Matcher<T>> matchers;
  for (const auto& e : expected) {
    matchers.emplace_back(EigenNearMatcher<T>(e, tolerance));
  }
  return ::testing::ElementsAreArray(matchers);
}

}  // namespace optimus

#endif  // OPTIMUS_TESTING_EIGEN_MATCHERS_H_
