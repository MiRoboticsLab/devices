// Copyright (c) 2023 Beijing Xiaomi Mobile Software Co., Ltd. All rights reserved.
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

#ifndef CYBERDOG_UWB__FLOAT_COMPARISONS_HPP_
#define CYBERDOG_UWB__FLOAT_COMPARISONS_HPP_

#include <algorithm>
#include <cmath>
#include <limits>

namespace cyberdog
{
namespace device
{
namespace helper_functions
{

/**
 * @brief Check for approximate equality in absolute terms.
 * @pre eps >= 0
 * @return True iff 'a' and 'b' are within 'eps' of each other.
 */
template<typename T>
bool abs_eq(const T & a, const T & b, const T & eps)
{
  static_assert(
    std::is_floating_point<T>::value,
    "Float comparisons only support floating point types.");

  return std::abs(a - b) <= eps;
}

/**
 * @brief Check for approximate less than in absolute terms.
 * @pre eps >= 0
 * @return True iff 'a' is less than 'b' minus 'eps'.
 */
template<typename T>
bool abs_lt(const T & a, const T & b, const T & eps)
{
  return !abs_eq(a, b, eps) && (a < b);
}

/**
 * @brief Check for approximate less than or equal in absolute terms.
 * @pre eps >= 0
 * @return True iff 'a' is less than or equal to 'b' plus 'eps'.
 */
template<typename T>
bool abs_lte(const T & a, const T & b, const T & eps)
{
  return abs_eq(a, b, eps) || (a < b);
}

/**
 * @brief Check for approximate greater than or equal in absolute terms.
 * @pre eps >= 0
 * @return True iff 'a' is greater than or equal to 'b' minus 'eps'.
 */
template<typename T>
bool abs_gte(const T & a, const T & b, const T & eps)
{
  return !abs_lt(a, b, eps);
}

/**
 * @brief Check for approximate greater than in absolute terms.
 * @pre eps >= 0
 * @return True iff 'a' is greater than 'b' minus 'eps'.
 */
template<typename T>
bool abs_gt(const T & a, const T & b, const T & eps)
{
  return !abs_lte(a, b, eps);
}

/**
 * @brief Check whether a value is within epsilon of zero.
 * @pre eps >= 0
 * @return True iff 'a' is within 'eps' of zero.
 */
template<typename T>
bool abs_eq_zero(const T & a, const T & eps)
{
  return abs_eq(a, static_cast<T>(0), eps);
}

/**
 * @brief
 * https://randomascii.wordpress.com/2012/02/25/comparing-floating-point-numbers-2012-edition/
 * @pre rel_eps >= 0
 * @return True iff 'a' and 'b' are within relative 'rel_eps' of each other.
 */
template<typename T>
bool rel_eq(const T & a, const T & b, const T & rel_eps)
{
  static_assert(
    std::is_floating_point<T>::value,
    "Float comparisons only support floating point types.");

  const auto delta = std::abs(a - b);
  const auto larger = std::max(std::abs(a), std::abs(b));
  const auto max_rel_delta = (larger * rel_eps);
  return delta <= max_rel_delta;
}

/**
 * @brief Check for approximate equality in absolute and relative terms.
 *
 * @note This method should be used only if an explicit relative or absolute
 * comparison is not appropriate for the particular use case.
 *
 * @pre abs_eps >= 0
 * @pre rel_eps >= 0
 * @return True iff 'a' and 'b' are within 'eps' or 'rel_eps' of each other
 */
template<typename T>
bool approx_eq(const T & a, const T & b, const T & abs_eps, const T & rel_eps)
{
  const auto are_absolute_eq = abs_eq(a, b, abs_eps);
  const auto are_relative_eq = rel_eq(a, b, rel_eps);
  return are_absolute_eq || are_relative_eq;
}

}  // namespace helper_functions
}  // namespace device
}  // namespace cyberdog

#endif  // CYBERDOG_UWB__FLOAT_COMPARISONS_HPP_
