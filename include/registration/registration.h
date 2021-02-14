#pragma once

/**
  * @file registration.h
  * @author julian 
  * @date 2/13/21
 */

#include <queue>
#include <string>
#include <cmath>

#include <ros/node_handle.h>
#include <registration/types.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf2_ros/transform_broadcaster.h>

#include <Eigen/Dense>

namespace registration
{

/**
 * Performs registration based on ICP. Calculates Transform based on 'Mobile Roboter' (Hertzberg, Laengemann, Nuechter)
 */
class Registration
{
public:
	/**
	 * Registration constructor
	 *
	 * @param nh NodeHandle, for pubs and subs
	 * @param topic topic to subscribe to
	 * @param queue_size size of queue
	 * @param max_distance max distance after which points won't be correlated
	 */
	Registration(ros::NodeHandle& nh, const std::string& topic, size_t queue_size, float max_distance);
	
	/// Default destructor. Queue cleanup is handles by ROS
	~Registration() = default;
	
	/// Delete move constructor
	Registration(Registration&& other) = delete;
	
	/// Delete Copy Constructor
	Registration(const Registration& other) = delete;
	
	/// Delete Assignment operator
	Registration& operator=(const Registration& other) = delete;
	
	/**
	 * static for testing
	 *
	 * Performs registration:
	 * 1. calculate correlating points
	 * 2. computer centers of each model and scan cloud based on these correlations
	 * 3. calculate transformation from this
	 *
	 * @param scan_cloud scan cloud
	 * @param model_cloud model cloud
	 * @param correlations correlations between model m and scan s points
	 * @param max_distance max distance after which points won't be correlated
	 * @return Transformation (tx, ty, theta) between model and scan cloud
	 */
	static Eigen::Vector3f
	performRegistration(const sensor_msgs::PointCloud2& scan_cloud,
					 	const sensor_msgs::PointCloud2& model_cloud,
					 	types::CorrVec& correlations,
					 	float max_distance);
	
private:
	/**
	 * PointCloud callback: build queue with 2 pointclouds and performs registration between scan n-1 and scan n
	 * @param pcl PointCloud
	 */
	void pclCallback(const sensor_msgs::PointCloud2::ConstPtr& pcl);
	
	/**
	 * Publishes transform after converting Transform (tx, ty, theta) to geometry_msgs::TransformStamped
	 *
	 * @param tf calculated transform
	 * @param header Header of model
	 */
	void publishTransform(const Eigen::Vector3f& tf, const std_msgs::Header& header);
	
	/**
	 * static for testing
	 *
	 * Calculates the Transformation (tx, ty, theta) between model and scan
	 *
	 * @param correlations correlations between model m and scan s points
	 * @param m_center model center
	 * @param s_center scan center
	 * @return Transformation (tx, ty, theta)
	 */
	static Eigen::Vector3f calculateTransform(const types::CorrVec& correlations,
									   const geometry_msgs::Point32& m_center,
									   const geometry_msgs::Point32& s_center);
	
	/// max distance after which points won't be correlated
	float max_distance;
	
	/// subscribes to pointcloud
	ros::Subscriber subscriber;
	
	/// publishes transformed cloud, for debugging
	ros::Publisher publisher;
	
	/// builds queue of 2 scans
	std::queue<sensor_msgs::PointCloud2> queue;
	
	/// publishes transform
	tf2_ros::TransformBroadcaster br;
};

} // namespace registration