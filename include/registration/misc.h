#pragma once

/**
  * @file misc.h
  * @author julian 
  * @date 2/13/21
 */

#include <ostream>
#include <registration/types.h>

#define SKIP_NULLPTR(X, Y) if (X == nullptr or Y == nullptr) continue
#define THROW_IF_COND(X, OP, Y, DESC) if (! (X OP Y)) throw std::runtime_error(DESC)
#define THROW_IF(X, DESC) if (X) throw std::runtime_error(DESC)

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

/**
 * Update centers of correlating point clouds with new match by acculating
 *
 * @param m_center model center
 * @param s_center scan center
 * @param pair new best match between model and scan point
 */
inline void updateCenters(geometry_msgs::Point32& m_center,
						  geometry_msgs::Point32& s_center,
						  const registration::types::CorrPair& pair)
{
	const auto& [mp, sp] = pair;
	m_center.x += mp->x;
	m_center.y += mp->y;
	m_center.z += mp->z;
	s_center.x += sp->x;
	s_center.y += sp->y;
	s_center.z += sp->z;
}

/**
 * Average center accumulations
 *
 * @param m_center model center
 * @param s_center scan center
 * @param size number to average with
 */
inline void avgCenters(geometry_msgs::Point32& m_center,
					   geometry_msgs::Point32& s_center,
					   float size)
{
	m_center.x /= size;
	m_center.y /= size;
	m_center.z /= size;
	s_center.x /= size;
	s_center.y /= size;
	s_center.z /= size;
}

} // namespace registration