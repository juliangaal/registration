
/**
  * @file tsdf_node.cpp
  * @author julian 
  * @date 2/26/21
 */

#include <ros/ros.h>
#include <geometry_msgs/Point32.h>
#include <sensor_msgs/PointCloud2.h>
#include <tsdfslam/eigen_helpers.h>
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

template <typename T>
sensor_msgs::PointCloud2 create_pcl(const std::vector<T>& tsdf_values)
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
	    // Eigen::Vector3i can't be shown directly, possible bug? Conversion to float.
	    Eigen::Vector3f value{tsdf_values[i][0], tsdf_values[i][1], tsdf_values[i][2]};
		pc2_points[i] = value * map_resolution;//RGBPoint(tsdf_values[i], {255, 0, 0});
	}
	
	return cloud;
}

Eigen::Vector3i to_map(Eigen::Vector3f& p)
{
	return Eigen::Vector3i(static_cast<int>(p.x() / map_resolution + map_resolution / 2.),
							static_cast<int>(p.y() / map_resolution + map_resolution / 2.),
							static_cast<int>(p.z() / map_resolution + map_resolution / 2.));
}

int to_local_map(Eigen::Vector3i& center)
{
    Eigen::Vector3i local_center{map_size[0] / 2, map_size[1] / 2, map_size[2] / 2};
    Eigen::Vector3i corr_center{local_center[0] - center[0],
                                local_center[1] - center[1],
                                local_center[2] - center[2]};

    int size_y = map_size[1];
    int size_z = map_size[2];

    int i = corr_center[2];
    int j = corr_center[1];
    int k = corr_center[0];

    return i * size_y * size_z + j * size_z + k;
}

void tsdfCallback(const sensor_msgs::PointCloud2ConstPtr& pcl)
{
	std::vector<Eigen::Vector3f> marching_values(10, {0., 0., 0.});
	std::vector<Eigen::Vector3i> centers(5);
	std::vector<float> tsdf_values(map_size.prod(), 0.0);
	Eigen::Vector3i prev_center(0, 0, 0);
	
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
			
			Eigen::Vector3i center = to_map(grid_march);
			
			if (center == prev_center)
			{
				continue;
			}
			
			centers.emplace_back(center);
			prev_center = center;

            float tsdf_value = (point - center).norm();
            double tsdf_value2 = distance - center.norm();

            if (step > distance)
            {
                tsdf_value *= -1;
            }

			ROS_INFO_STREAM("--- " << to_local_map(center) << " | " << tsdf_values.size());
            ROS_INFO_STREAM("local map center: " << map_size[0] / 2 << "/" << map_size[1] / 2 << "/" << map_size[2] / 2);
			ROS_INFO_STREAM("Point : " << point.x() << " " << point.y() << " -> Step: " << step);
		 	ROS_INFO_STREAM("March : " << grid_march.x() << " " << grid_march.y() << " " << grid_march.z());
            ROS_INFO_STREAM("Center:  " << center.x() << " " << center.y() << " " << center.z() << ", TSDF: " << tsdf_value << " TSDF2: " << tsdf_value2);
		}

		ROS_INFO_STREAM("========");
	}

	ROS_INFO_STREAM("Published cloud");
	auto march_pcl = create_pcl(marching_values);
	auto center_pcl = create_pcl(centers);
	
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