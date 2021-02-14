#pragma once

/**
  * @file types.h
  * @author julian 
  * @date 2/14/21
 */
 
#include <geometry_msgs/Point32.h>
#include <vector>

namespace registration::types
{
	using CorrPair = std::pair<const geometry_msgs::Point32*, const geometry_msgs::Point32*>;
	using CorrVec =  std::vector<CorrPair>;
}