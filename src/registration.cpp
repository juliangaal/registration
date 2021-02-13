
/**
  * @file registration.cpp
  * @author julian 
  * @date 2/13/21
 */

#include <angles/angles.h>
#include <registration/registration.h>
#include <registration/geometry.h>
#include <registration/misc.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

using namespace registration;

Registration::Registration(ros::NodeHandle& nh, const std::string& topic,  size_t queue_size, float max_distance)
: max_distance{max_distance}
, subscriber{nh.subscribe(topic, queue_size, &Registration::pclCallback, this)}
, publisher{nh.advertise<sensor_msgs::PointCloud2>("transformed_pcl", 10)}
, queue{}
{

}

void Registration::pclCallback(const sensor_msgs::PointCloud2::ConstPtr& pcl)
{
	queue.push(*pcl.get());
	
	// Before two scans haven't been received, do nothing
	if (queue.size() < 2)
	{
		return;
	}
	
	auto& model_cloud = queue.front();
	auto& scan_cloud = queue.back();
	
	// Check for empty pointcloud
	if (scan_cloud.data.empty() or model_cloud.data.empty())
	{
		ROS_WARN("Received empty pointcloud");
		return;
	}
	
	auto n_points = std::max(scan_cloud.width, model_cloud.width);
	
	// 2d Transforms in shape (tx, ty, theta): transation x, translation y, rotation theta
	Eigen::Vector3f global_trans(0, 0, 0);
	Eigen::Vector3f delta_trans(1.f, 1.f, 1.f);
	size_t it = 0;
	
	CorrVec correlations{n_points};
	
	while (delta_trans.z() > angles::from_degrees(1) && it <= 10)
	{
		// Calculate transform
		delta_trans = performRegistration(scan_cloud, model_cloud, correlations);
		
		ROS_INFO_STREAM(it << " - Transform:\n" << delta_trans);
		
		// transform pointcloud for next iteration
		geometry::transformPointCloud(delta_trans, scan_cloud);
		
		// update global transform
		global_trans += geometry::createRotationMatrix(delta_trans.z()) * delta_trans;
		
		// increate iterations
		++it;
	}
	
	ROS_INFO_STREAM("Done. Global transform:\n" << global_trans);
	publisher.publish(scan_cloud);
	
	publishTransform(global_trans, model_cloud.header);
	
	// Pop model from queue. Scan is model in next iteration
	queue.pop();
}

void Registration::publishTransform(const Eigen::Vector3f& tf, const std_msgs::Header& header)
{
	geometry_msgs::TransformStamped tf_stamped;
	tf_stamped.header.stamp = header.stamp;
	tf_stamped.header.frame_id = "base_link";
	tf_stamped.child_frame_id = header.frame_id;
	
	tf2::Transform transform;
	tf2::Quaternion rot;
	rot.setEuler (0.0f, 0.0f, tf.z());
	transform.setRotation (rot);
	transform.setOrigin (tf2::Vector3 (tf.x(), tf.y(), 0.0));
	
	tf2::convert(transform, tf_stamped.transform);
	
	static tf2_ros::TransformBroadcaster br;
	br.sendTransform(tf_stamped);
}

void Registration::updateCenters(geometry_msgs::Point32& m_center, geometry_msgs::Point32& s_center, const CorrPair& pair)
{
	const auto& [mp, sp] = pair;
	m_center.x += mp->x;
	m_center.y += mp->y;
	m_center.z += mp->z;
	s_center.x += sp->x;
	s_center.y += sp->y;
	s_center.z += sp->z;
}

void Registration::avgCenters(geometry_msgs::Point32& m_center, geometry_msgs::Point32& s_center, const CorrVec& correlations)
{
	m_center.x /= correlations.size();
	m_center.y /= correlations.size();
	m_center.z /= correlations.size();
	s_center.x /= correlations.size();
	s_center.y /= correlations.size();
	s_center.z /= correlations.size();
}

Eigen::Vector3f Registration::performRegistration(const sensor_msgs::PointCloud2& scan_cloud, const sensor_msgs::PointCloud2& model_cloud, CorrVec& correlations)
{
	// clear old correlations
	correlations.clear();
	
	// Centers of corresponding point clouds
	geometry_msgs::Point32 m_center;
	geometry_msgs::Point32 s_center;
	
	// Easier to iterate
	const auto* scan_points = reinterpret_cast<const geometry_msgs::Point32*>(scan_cloud.data.data());
	const auto* model_points = reinterpret_cast<const geometry_msgs::Point32*>(model_cloud.data.data());
	
	for (size_t scan_i = 0; scan_i < scan_cloud.width; ++scan_i)
	{
		// parameters for ICP
		float shortest_distance = std::numeric_limits<float>::max();
		CorrPair best_pair;
		bool found_corr = false;
		
		for (size_t model_i = 0; model_i < model_cloud.width; ++model_i)
		{
			auto distance = geometry::euclideanDistance(&scan_points[scan_i], &model_points[model_i]);
			if (distance < shortest_distance && distance < max_distance)
			{
				// new best point: make pait
				best_pair = std::make_pair(&model_points[model_i], &scan_points[model_i]);
				found_corr = true;
				shortest_distance = distance;
			}
		}
		
		if (found_corr)
		{
			// save best pair
			correlations.push_back(best_pair);
			// update centers of corresponding points
			updateCenters(m_center, s_center, best_pair);
		}
	}
	
	// average centers of corresponding points
	avgCenters(m_center, s_center, correlations);
	
	return calculateTransform(correlations, m_center, s_center);
}

/**
 * Calculates the Transformation (tx, ty, theta) between model and scan
 *
 * @param correlations correlations between model m and scan s points
 * @param m_center model center
 * @param s_center scan center
 * @return Transformation (tx, ty, theta)
 */
Eigen::Vector3f Registration::calculateTransform(const CorrVec& correlations,
								   const geometry_msgs::Point32& m_center,
								   const geometry_msgs::Point32& s_center)
{
	// Calculate Summations
	geometry_msgs::Point32 s_x;
	geometry_msgs::Point32 s_y;
	
	for (const auto& [scan_p, model_p]: correlations)
	{
		SKIP_NULLPTR(scan_p, model_p);
		s_x.x += (scan_p->x - m_center.x) * (model_p->x - s_center.x);
		s_y.y += (scan_p->y - m_center.y) * (model_p->y - s_center.y);
		s_x.y += (scan_p->x - m_center.x) * (model_p->y - s_center.y);
		s_y.x += (scan_p->y - m_center.y) * (model_p->x - s_center.x);
	}
	
	// Calculate estimated rotation
	float rotation = std::atan2(s_y.x - s_x.y, s_x.x + s_y.y);
	
	// Calculate estimated translation
	float cos = std::cos(rotation);
	float sin = std::sin(rotation);
	
	Eigen::Vector3f transformation
	{
			m_center.x - (s_center.x * cos - s_center.y * sin),
			m_center.y - (s_center.x * sin + s_center.y * cos),
			rotation
	};
	
	return transformation;
}
