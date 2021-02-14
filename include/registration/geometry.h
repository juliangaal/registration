#pragma once

/**
  * @file geometry.h
  * @author julian 
  * @date 2/13/21
 */

#include <Eigen/Dense>
#include <ros/console.h>
#include <sensor_msgs/PointCloud2.h>

namespace registration::geometry
{

/**
 * Creates rotation matrix around z-axis from Transformation (tx,ty,theta):
 * * tx: translation x
 * * ty: translation y
 * * theta: rotation around z-axis
 *
 * @param transform ransformation (tx,ty,theta)
 * @return rotation matrix around z axis, _with_ translation
 */
inline Eigen::Matrix3f createRotationMatrix(const Eigen::Vector3f &transform)
{
    const auto& theta = transform.z();
    const auto& tx = transform.x();
    const auto& ty = transform.y();

    Eigen::Matrix3f rot = Eigen::Matrix3f::Identity();
    rot(0, 0) = std::cos(theta);
    rot(0, 1) = -std::sin(theta);
    rot(1, 0) = std::sin(theta);
    rot(1, 1) = std::cos(theta);

    rot(0, 2) = tx;
    rot(1, 2) = ty;

    return rot;
}

/**
 * Creates rotation matrix around z-axis from rotation angle:
 *
 * @param theta rotation around z
 * @return @return rotation matrix around z axis, _without_ any form of translation
 */
inline Eigen::Matrix3f createRotationMatrix(float theta)
{
    Eigen::Matrix3f rot = Eigen::Matrix3f::Identity();
    rot(0,0) = std::cos (theta);
    rot(0,1) = -std::sin(theta);
    rot(1,0) = std::sin(theta);
    rot(1,1) = std::cos(theta);
    return rot;
}

/**
 * Transform point cloud with translation tx, translation ty and rotation theta (around z axis)
 *
 * @param transform Transformation (tx, ty, theta)
 * @param out_cloud resulting pointcloud
 */
inline void transformPointCloud(const Eigen::Vector3f &transform, sensor_msgs::PointCloud2 &out_cloud)
{
    auto *out_points = reinterpret_cast<geometry_msgs::Point32 *>(out_cloud.data.data());

    Eigen::Matrix3f rot = createRotationMatrix(transform);

    Eigen::Vector3f v;
    for (auto index = 0u; index < out_cloud.width; ++index)
    {
        v = Eigen::Vector3f(out_points[index].x, out_points[index].y, out_points[index].z);
        v = rot * v;
        out_points[index].x = v[0];
        out_points[index].y = v[1];
        out_points[index].z = v[2];
    }
}

/**
 * Calculates euclidean distance between two geometry_msgs::Point
 *
 * @param p1 point 1
 * @param p2 point 2
 * @return euclidean distance
 */
inline float euclideanDistance(const geometry_msgs::Point32 *p1, const geometry_msgs::Point32 *p2)
{
	if (!p1 or !p2)
	{
		ROS_WARN("Received nullptr in euclideanDistance calculation! (If this happens during rostest, don't worry)");
		return 0.f;
	}
	
	return std::sqrt(std::pow(p2->x - p1->x, 2.f) + std::pow(p2->y - p1->y, 2.f) + std::pow(p2->z - p1->z, 2.f));
}

} // namespace registration::geometry