#pragma once

/**
  * @file misc.h
  * @author julian 
  * @date 2/13/21
 */

#include <ostream>

#define SKIP_NULLPTR(X, Y) if (X == nullptr or Y == nullptr) continue;

namespace registration::misc
{

/**
 * Prints point/vector3 or other ros types of form x/y/z
 *
 * @tparam T point type
 * @param os ostream
 * @param point point to print
 * @return ostream, updated
 */
template<typename T>
inline std::ostream & operator<<(std::ostream &os, const T &point)
{
    os << point.x << " / " << point.y << " / " << point.z;
    return os;
}

} // namespace registration