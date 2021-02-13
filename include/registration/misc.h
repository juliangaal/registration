#pragma once

/**
  * @file misc.h
  * @author julian 
  * @date 2/13/21
 */

#include <ostream>

namespace registration::misc
{

template<typename T>
inline std::ostream & operator<<(std::ostream &os, const T &point)
{
    os << point.x << " / " << point.y << " / " << point.z;
    return os;
}

} // registration::misc