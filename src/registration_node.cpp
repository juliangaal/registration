
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
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/Point32.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <visualization_msgs/Marker.h>
#include <std_msgs/Header.h>
#include <pcl_ros/transforms.h>
#include <Eigen/Dense>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_broadcaster.h>

#define PTPTR_TO_COUT(X) "( " << X->x << " / " << X->y << " / " << X->z << " )"
#define PT_TO_COUT(X) "( " << X.x << " / " << X.y << " / " << X.z << " )"
#define SKIP_NULLPTR(X, Y) if (X == nullptr or Y == nullptr) continue;

using MySyncPolicy = message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud2, sensor_msgs::PointCloud2>;
using CorrPair = std::pair<const geometry_msgs::Point32*, const geometry_msgs::Point32*>;
using CorrVec =  std::vector<CorrPair>;
Eigen::Vector3f global_transform;

float max_distance = 1.f;

ros::Publisher corr_pub;
ros::Publisher pcl_transform_pub;

float euclideanDistance(const geometry_msgs::Point32* p1, const geometry_msgs::Point32* p2)
{
    return std::sqrt(std::pow(p2->x - p1->x, 2.f) + std::pow(p2->y - p1->y, 2.f) + std::pow(p2->z - p1->z, 2.f));
}

void avg_pt(geometry_msgs::Point32& pt, const CorrVec& correlations)
{
    pt.x /= correlations.size();
    pt.y /= correlations.size();
    pt.z /= correlations.size();
}

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

void calculateCorrelations(const sensor_msgs::PointCloud2& m_centerloud, const sensor_msgs::PointCloud2& model_cloud, CorrVec& correlations)
{
    correlations.clear();

    const auto* scan_points = reinterpret_cast<const geometry_msgs::Point32*>(m_centerloud.data.data());
    const auto* model_points = reinterpret_cast<const geometry_msgs::Point32*>(model_cloud.data.data());

    for (size_t scan_i = 0; scan_i < m_centerloud.width; ++scan_i)
    {
        float shortest_distance = std::numeric_limits<float>::max();
        CorrPair best_pair;
        bool found_corr = false;

        for (size_t model_i = 0; model_i < model_cloud.width; ++model_i)
        {
            auto distance = euclideanDistance(&scan_points[scan_i], &model_points[model_i]);
            if (distance < shortest_distance && distance < max_distance)
            {
                best_pair = std::make_pair(&model_points[model_i], &scan_points[model_i]);
                found_corr = true;
                shortest_distance = distance;
            }
        }

        if (found_corr)
        {
            correlations.push_back(best_pair);
        }
    }

    publishCorrMarkers(correlations, model_cloud.header);
}

Eigen::Matrix3f createRotationMatrix(float theta)
{
    Eigen::Matrix3f rot = Eigen::Matrix3f::Identity();
    rot(0,0) = std::cos (theta);
    rot(0,1) = -std::sin(theta);
    rot(1,0) = std::sin(theta);
    rot(1,1) = std::cos(theta);
    return rot;
}

void transformPointCloud(const Eigen::Vector3f& transform, sensor_msgs::PointCloud2& out_cloud)
{
    auto* out_points = reinterpret_cast<geometry_msgs::Point32*>(out_cloud.data.data());

    Eigen::Matrix3f rot = createRotationMatrix(transform.z());
    rot(0, 2) = transform.x();
    rot(1, 2) = transform.y();

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

Eigen::Vector3f calculateTransform(const CorrVec& correlations)
{
    // Calculate Centers of corresponding point clouds
    geometry_msgs::Point32 m_center;
    geometry_msgs::Point32 s_center;

    for (const auto& [p1, p2]: correlations)
    {
        SKIP_NULLPTR(p1, p2);
        m_center.x += p1->x;
        m_center.y += p1->y;
        m_center.z += p1->z;
        s_center.x += p2->x;
        s_center.y += p2->y;
        s_center.z += p2->z;
    }

    m_center.x /= correlations.size();
    m_center.y /= correlations.size();
    m_center.z /= correlations.size();
    s_center.x /= correlations.size();
    s_center.y /= correlations.size();
    s_center.z /= correlations.size();

    ROS_INFO_STREAM("Calculated centers: " << PT_TO_COUT(m_center) << ", " << PT_TO_COUT(s_center));

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

    ROS_INFO_STREAM("Calculated summation_x: " << PT_TO_COUT(s_x));
    ROS_INFO_STREAM("Calculated summation_y: " << PT_TO_COUT(s_y));

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
    auto& m_centerloud = queue.back();

    if (m_centerloud.data.empty() or model_cloud.data.empty())
    {
        ROS_WARN("Received empty pointcloud");
        return;
    }

    auto n_points = std::max(m_centerloud.width, model_cloud.width);
    ROS_INFO_STREAM("Received 2 pointclouds (size " << n_points << ") " << std::abs((m_centerloud.header.stamp - model_cloud.header.stamp).toSec()) << "s apart");

    Eigen::Vector3f global_trans(0, 0, 0);
    Eigen::Vector3f delta_trans(1.f, 1.f, 1.f);
    size_t it = 0;

    while (delta_trans.z() > angles::from_degrees(1) && it <= 10)
    {
        CorrVec correlations{n_points};
        calculateCorrelations(m_centerloud, model_cloud, correlations);

        delta_trans = calculateTransform(correlations);
        ROS_INFO_STREAM(it << " - Translation: (" << delta_trans.x() << "/" << delta_trans.y() << "), rot: " << delta_trans.z());

        transformPointCloud(delta_trans, m_centerloud);

        global_trans += createRotationMatrix(delta_trans.z()) * delta_trans;
        ++it;
    }

    ROS_INFO_STREAM("Done. Global transform: (" << global_trans.x() << "/" << global_trans.y() << "), rot: " << global_trans.z() << "\n");
    pcl_transform_pub.publish(m_centerloud);

//    publishTransform(global_trans, model_cloud->header);

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