
/**
  * @file registration_node.cpp
  * @author julian gaal
  * @date 2/9/21
 */

#include <ros/ros.h>
#include <angles/angles.h>
#include <registration/registration.h>

using namespace registration;

template <typename T>
T getParam(const ros::NodeHandle& nh, std::string name, T default_val)
{
	T var;
	nh.param<T>(name, var, default_val);
	return var;
}

/**
 * ROS Node that performs registration on PointCloud
 *
 * @param argc n_parameters
 * @param argv parameters
 *
 * @return 0 if successful
 */
int main(int argc, char **argv)
{
    ros::init(argc, argv, "registration");
    ros::NodeHandle nh;
	
	size_t queue_size = getParam(nh, "queue_size", 100);
	
	types::ICPParams icp_params
	{
			static_cast<float>(getParam(nh, "registration_node/max_distance", 1.0)),
			static_cast<float>(angles::from_degrees(getParam(nh, "registration_node/min_dtheta", 1.0))),
			static_cast<float>(getParam(nh, "registration_node/max_it", 25.0))
	};
	
	ROS_INFO_STREAM(icp_params);
	
	Registration registration(nh, "pcl", queue_size, icp_params);
    ros::spin();

    return 0;
}