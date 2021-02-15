#pragma once

/**
  * @file types.h
  * @author julian gaal
  * @date 2/14/21
 */

#include <geometry_msgs/Point32.h>
#include <vector>

namespace registration::types
{
/// defines correlations between two points in point cloud
using CorrPair = std::pair<const geometry_msgs::Point32 *, const geometry_msgs::Point32 *>;

/// saves pairs of correlations
using CorrVec = std::vector<CorrPair>;

/**
* ICP parameters
*/
struct ICPParams
{
	/// max distance after which points won't be correlated
	float max_distance;
	/// if calculated rotation falls below min_dtheta, Registration is considered done
	float min_dtheta;
	/// if max_it is reached, Registration is considered done
	float max_it;
};
}

inline std::ostream &operator<<(std::ostream &os, const registration::types::ICPParams &icp_params)
{
	os << "ICP Parameters: \n";
	os << "  max_distance: " << icp_params.max_distance << "\n";
	os << "  min_dtheta: " << icp_params.min_dtheta << "\n";
	os << "  max_it: " << icp_params.max_it << "\n";
	return os;
}