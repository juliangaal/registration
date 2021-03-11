
/**
  * @file tsdf_node.cpp
  * @author julian 
  * @date 2/26/21
 */

#include <ros/ros.h>
#include <tf2/LinearMath/Vector3.h>
#include <geometry_msgs/Point32.h>
#include <sensor_msgs/PointCloud2.h>
#include <Eigen/Dense>
#include <cmath>
#include <utility>

const static Eigen::Vector3i map_size(20, 20, 20);
constexpr float tau = 5;
constexpr float map_resolution = 1;
ros::Publisher march_pub;
ros::Publisher center_pub;

struct RGBPoint
{
	RGBPoint(Eigen::Vector3f point, Eigen::Vector3i rgb)
	: point(std::move(point))
	, rgb(std::move(rgb))
	{}
	
	~RGBPoint() = default;
	
	Eigen::Vector3f point;
	Eigen::Vector3i rgb;
};

//void publish_tsdf(const std::vector<double>& tsdf_values)
//{
//	sensor_msgs::PointCloud2 cloud;
//	cloud.header.frame_id = "laser";
//	cloud.header.stamp = ros::Time::now();
//	cloud.height = 1;
//	cloud.width = tsdf_values.size();
//	cloud.point_step = sizeof(Eigen::Vector3f);
//	cloud.row_step = cloud.width * cloud.point_step;
//
//	cloud.data.resize(cloud.width * cloud.height * sizeof(Eigen::Vector3f));
//	cloud.fields.resize(3);
//	cloud.fields[0].name = "x";
//	cloud.fields[0].offset = 0;
//	cloud.fields[0].datatype = sensor_msgs::PointField::FLOAT32;
//	cloud.fields[0].count = 1;
//	cloud.fields[1].name = "y";
//	cloud.fields[1].offset = 4;
//	cloud.fields[1].datatype = sensor_msgs::PointField::FLOAT32;
//	cloud.fields[1].count = 1;
//	cloud.fields[2].name = "z";
//	cloud.fields[2].offset = 8;
//	cloud.fields[2].datatype = sensor_msgs::PointField::FLOAT32;
//	cloud.fields[2].count = 1;
//	cloud.fields[3].name = "r";
//	cloud.fields[3].offset = 12;
//	cloud.fields[3].datatype = sensor_msgs::PointField::INT32;
//	cloud.fields[3].count = 1;
//	cloud.fields[4].name = "g";
//	cloud.fields[4].offset = 16;
//	cloud.fields[4].datatype = sensor_msgs::PointField::INT32;
//	cloud.fields[4].count = 1;
//	cloud.fields[5].name = "b";
//	cloud.fields[5].offset = 20;
//	cloud.fields[5].datatype = sensor_msgs::PointField::INT32;
//	cloud.fields[5].count = 1;
//
//	auto pc2_points = reinterpret_cast<RGBPoint*>(cloud.data.data());
//	for (size_t i = 0; i < tsdf_values.size(); ++i)
//	{
//		pc2_points[i] = RGBPoint(tsdf_values[i], {255, 0, 0});
//	}
////	std::fill(pc2_points, pc2_points+tsdf_values.size(), tsdf_values);
//
//	pub.publish(cloud);
//}

sensor_msgs::PointCloud2 create_pcl(const std::vector<Eigen::Vector3f>& tsdf_values, bool scale = false)
{
	sensor_msgs::PointCloud2 cloud;
	cloud.header.frame_id = "laser";
	cloud.header.stamp = ros::Time::now();
	cloud.height = 1;
	cloud.width = tsdf_values.size();
	cloud.point_step = sizeof(Eigen::Vector3f);
	cloud.row_step = cloud.width * cloud.point_step;

	cloud.data.resize(cloud.width * cloud.height * sizeof(Eigen::Vector3f));
	cloud.fields.resize(3);
	cloud.fields[0].name = "x";
	cloud.fields[0].offset = 0;
	cloud.fields[0].datatype = sensor_msgs::PointField::FLOAT32;
	cloud.fields[0].count = 1;
	cloud.fields[1].name = "y";
	cloud.fields[1].offset = 4;
	cloud.fields[1].datatype = sensor_msgs::PointField::FLOAT32;
	cloud.fields[1].count = 1;
	cloud.fields[2].name = "z";
	cloud.fields[2].offset = 8;
	cloud.fields[2].datatype = sensor_msgs::PointField::FLOAT32;
	cloud.fields[2].count = 1;
//	cloud.fields[3].name = "r";
//	cloud.fields[3].offset = 12;
//	cloud.fields[3].datatype = sensor_msgs::PointField::INT32;
//	cloud.fields[3].count = 1;
//	cloud.fields[4].name = "g";
//	cloud.fields[4].offset = 16;
//	cloud.fields[4].datatype = sensor_msgs::PointField::INT32;
//	cloud.fields[4].count = 1;
//	cloud.fields[5].name = "b";
//	cloud.fields[5].offset = 20;
//	cloud.fields[5].datatype = sensor_msgs::PointField::INT32;
//	cloud.fields[5].count = 1;

	auto pc2_points = reinterpret_cast<Eigen::Vector3f*>(cloud.data.data());
	for (size_t i = 0; i < tsdf_values.size(); ++i)
	{
		pc2_points[i] = tsdf_values[i] * map_resolution;//RGBPoint(tsdf_values[i], {255, 0, 0});
	}
	
	return cloud;
}

Eigen::Vector3f to_map(Eigen::Vector3f& p)
{
	return Eigen::Vector3f(static_cast<int>(p.x() / map_resolution + map_resolution / 2.),
							static_cast<int>(p.y() / map_resolution + map_resolution / 2.),
							static_cast<int>(p.z() / map_resolution + map_resolution / 2.));
}

void tsdfCallback(const sensor_msgs::PointCloud2ConstPtr& pcl)
{
	std::vector<Eigen::Vector3f> marching_values(10, {0., 0., 0.});
	std::vector<Eigen::Vector3f> centers(10, {0., 0., 0.});
	std::vector<float> tsdf_values(map_size.norm(), 0.0);
	Eigen::Vector3f prev_center(0., 0., 0.);
	
	const auto *points = reinterpret_cast<const geometry_msgs::Point32*>(pcl->data.data());

	for(auto i = 0u; i < pcl->width; ++i)
	{
		Eigen::Vector3f point{points[i].x, points[i].y, points[i].z};
		float distance = point.norm();
		Eigen::Vector3f direction = point / distance;

		for (double step = distance - tau; step < distance + tau; step += map_resolution / 2.)
		{
			Eigen::Vector3f grid_march(direction.x() * step,
									 	direction.y() * step,
									 direction.z() * step);
			marching_values.push_back(grid_march);
			
			Eigen::Vector3f center = to_map(grid_march);
			
			if (center == prev_center)
			{
				continue;
			}
			
			centers.emplace_back(center);
			prev_center = center;
			
			float tsdf_value = distance - center.norm();
			ROS_INFO_STREAM("Point: " << point.x() << " " << point.y() << " -> Step: " << step);
		 	ROS_INFO_STREAM("March: " << grid_march.x() << " " << grid_march.y() << " " << grid_march.z());
			ROS_INFO_STREAM("Grid :  " << center.x() << " " << center.y() << " " << center.z() << ", TSDF: " << tsdf_value);
		}
	}

	ROS_INFO_STREAM("Published cloud");
	auto march_pcl = create_pcl(marching_values);
	auto center_pcl = create_pcl(centers, true);
	
	center_pub.publish(center_pcl);
	march_pub.publish(march_pcl);
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "registration");
	ros::NodeHandle nh;
	
	auto sub = nh.subscribe("cloud", 100, tsdfCallback);
	march_pub = nh.advertise<sensor_msgs::PointCloud2>("march", 1000);
	center_pub = nh.advertise<sensor_msgs::PointCloud2>("centers", 1000);
	
	ros::spin();
}