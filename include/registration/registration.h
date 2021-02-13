#pragma once

/**
  * @file registration.h
  * @author julian 
  * @date 2/13/21
 */

#include <queue>
#include <string>
#include <vector>
#include <cmath>

#include <ros/node_handle.h>
#include <geometry_msgs/Point32.h>
#include <sensor_msgs/PointCloud2.h>

#include <Eigen/Dense>

using CorrPair = std::pair<const geometry_msgs::Point32*, const geometry_msgs::Point32*>;
using CorrVec =  std::vector<CorrPair>;

namespace registration
{

class Registration
{
public:
	Registration(ros::NodeHandle& nh, const std::string& topic, size_t queue_size, float max_distance);
	~Registration() = default;
	Registration(Registration&& other) = delete;
	Registration(const Registration& other) = delete;
	Registration& operator=(const Registration& other) = delete;
	
	/**
	 * Performs registration:
	 * 1. calculate correlating points
	 * 2. computer centers of each model and scan cloud based on these correlations
	 * 3. calculate transformation from this
	 *
	 * @param scan_cloud scan cloud
	 * @param model_cloud model cloud
	 * @param correlations correlations between model m and scan s points
	 * @return Transformation (tx, ty, theta) between model and scan cloud
	 */
	Eigen::Vector3f
	performRegistration(const sensor_msgs::PointCloud2& scan_cloud,
					 	const sensor_msgs::PointCloud2& model_cloud,
					 	CorrVec& correlations);
	
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
	 * Calculates the Transformation (tx, ty, theta) between model and scan
	 *
	 * @param correlations correlations between model m and scan s points
	 * @param m_center model center
	 * @param s_center scan center
	 * @return Transformation (tx, ty, theta)
	 */
	Eigen::Vector3f calculateTransform(const CorrVec& correlations,
									   const geometry_msgs::Point32& m_center,
									   const geometry_msgs::Point32& s_center);
	
	/**
	 * Update centers of correlating point clouds with new match by acculating
	 *
	 * @param m_center model center
	 * @param s_center scan center
	 * @param pair new best match between model and scan point
	 */
	void updateCenters(geometry_msgs::Point32& m_center, geometry_msgs::Point32& s_center, const CorrPair& pair);
	
	/**
	 * Average center accumulations
	 *
	 * @param m_center model center
	 * @param s_center scan center
	 * @param correlations correlations between model m and scan s points
	 */
	void avgCenters(geometry_msgs::Point32& m_center, geometry_msgs::Point32& s_center, const CorrVec& correlations);
	
	float max_distance;
	ros::Subscriber subscriber;
	ros::Publisher publisher;
	std::queue<sensor_msgs::PointCloud2> queue;
};

} // namespace registration