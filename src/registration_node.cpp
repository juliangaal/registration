
/**
  * @file registration_node.cpp
  * @author julian gaal
  * @date 2/9/21
 */

#include <cmath>
#include <tuple>
#include <vector>
#include <queue>

#include <angles/angles.h>
#include <ros/ros.h>
#include <geometry_msgs/Point32.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <visualization_msgs/Marker.h>
#include <std_msgs/Header.h>
#include <pcl_ros/transforms.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_broadcaster.h>
#include <registration/misc.h>
#include <registration/geometry.h>

using namespace registration;

#define SKIP_NULLPTR(X, Y) if (X == nullptr or Y == nullptr) continue;

using MySyncPolicy = message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud2, sensor_msgs::PointCloud2>;
using CorrPair = std::pair<const geometry_msgs::Point32*, const geometry_msgs::Point32*>;
using CorrVec =  std::vector<CorrPair>;
Eigen::Vector3f global_transform;

float max_distance = 1.f;

ros::Publisher corr_pub;
ros::Publisher pcl_transform_pub;

void publishCorrMarkers(const CorrVec& correlations, const std_msgs::Header& header)
{
    visualization_msgs::Marker corr_lines;
    corr_lines.header = header;
    corr_lines.ns = "corr_lines";
    corr_lines.id = 0;
    corr_lines.action = visualization_msgs::Marker::ADD;
    corr_lines.type = visualization_msgs::Marker::LINE_LIST;
    corr_lines.pose.orientation.w = 1.0;
    corr_lines.scale.x = 0.01;
    corr_lines.color.r = 0.0;
    corr_lines.color.g = 1.0;
    corr_lines.color.b = 1.0;
    corr_lines.color.a = 1.0;

    corr_lines.points.reserve(correlations.size());
    geometry_msgs::Point p;

    for (const auto& [p1, p2]: correlations)
    {
        SKIP_NULLPTR(p1, p2);
        p.x = p1->x;
        p.y = p1->y;
        p.z = p1->z;
        corr_lines.points.push_back(p);
        p.x = p2->x;
        p.y = p2->y;
        p.z = p2->z;
        corr_lines.points.push_back(p);
    }

    corr_pub.publish(corr_lines);

}

void updateCenters(geometry_msgs::Point32& m_center, geometry_msgs::Point32& s_center, const CorrPair& pair)
{
	const auto& [mp, sp] = pair;
	m_center.x += mp->x;
	m_center.y += mp->y;
	m_center.z += mp->z;
	s_center.x += sp->x;
	s_center.y += sp->y;
	s_center.z += sp->z;
}

void avgCenters(geometry_msgs::Point32& m_center, geometry_msgs::Point32& s_center, const CorrVec& correlations)
{
	m_center.x /= correlations.size();
	m_center.y /= correlations.size();
	m_center.z /= correlations.size();
	s_center.x /= correlations.size();
	s_center.y /= correlations.size();
	s_center.z /= correlations.size();
}

Eigen::Vector3f calculateTransform(const CorrVec& correlations,
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
	
	ROS_INFO_STREAM("Calculated summation_x: " << s_x);
	ROS_INFO_STREAM("Calculated summation_y: " << s_y);
	
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

Eigen::Vector3f performRegistration(const sensor_msgs::PointCloud2& scan_cloud, const sensor_msgs::PointCloud2& model_cloud, CorrVec& correlations)
{
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
	
    publishCorrMarkers(correlations, model_cloud.header);
    
    return calculateTransform(correlations, m_center, s_center);
}

void publishTransform(const Eigen::Vector3f& tf, const std_msgs::Header& header)
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

void pclCallback(const sensor_msgs::PointCloud2::ConstPtr& pcl)
{
    static std::queue<sensor_msgs::PointCloud2> queue;
    queue.push(*pcl.get());
    if (queue.size() < 2)
    {
        return;
    }

    auto& model_cloud = queue.front();
    auto& scan_cloud = queue.back();

    if (scan_cloud.data.empty() or model_cloud.data.empty())
    {
        ROS_WARN("Received empty pointcloud");
        return;
    }

    auto n_points = std::max(scan_cloud.width, model_cloud.width);
    ROS_INFO_STREAM("Received 2 pointclouds (size " << n_points << ") " << std::abs((scan_cloud.header.stamp - model_cloud.header.stamp).toSec()) << "s apart");

    Eigen::Vector3f global_trans(0, 0, 0);
    Eigen::Vector3f delta_trans(1.f, 1.f, 1.f);
    size_t it = 0;

    while (delta_trans.z() > angles::from_degrees(1) && it <= 10)
    {
    	// Calculate transform
        CorrVec correlations{n_points};
        delta_trans = performRegistration(scan_cloud, model_cloud, correlations);

        ROS_INFO_STREAM(it << " - Translation: (" << delta_trans.x() << "/" << delta_trans.y() << "), rot: " << delta_trans.z());

        // transform pointcloud for next iteration
        geometry::transformPointCloud(delta_trans, scan_cloud);
        
        // update global transform
        global_trans += geometry::createRotationMatrix(delta_trans.z()) * delta_trans;
        
        ++it;
    }

    ROS_INFO_STREAM("Done. Global transform: " << global_trans << "\n");
    pcl_transform_pub.publish(scan_cloud);

    publishTransform(global_trans, model_cloud.header);

    // Pop model from queue
    queue.pop();
}



int main(int argc, char **argv)
{
    ros::init(argc, argv, "registration");
    ros::NodeHandle n;

    // PCL subscriber
    ros::Subscriber sub = n.subscribe("pcl", 1000, pclCallback);

    // Publishers
    pcl_transform_pub = n.advertise<sensor_msgs::PointCloud2>("rot", 10);
    corr_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 10);

    global_transform.setIdentity();

    ros::spin();

    return 0;
}