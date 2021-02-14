
/**
  * @file registration_node_test.cpp
  * @author julian 
  * @date 2/13/21
 */

#include <ros/ros.h>
#include <angles/angles.h>
#include <std_msgs/Header.h>
#include <geometry_msgs/Point32.h>
#include <sensor_msgs/PointCloud2.h>
#include <gtest/gtest.h>

#include <registration/misc.h>
#include <registration/geometry.h>
#include <registration/registration.h>

#include <Eigen/Dense>

constexpr float epsilon = 0.0001;

TEST(MiscTest, centers)
{
	using namespace registration;
	
    geometry_msgs::Point32 center;
    center.x = 1;
    center.y = 1;
    center.z = 1;
    
    geometry_msgs::Point32 center2 = center;
    
    geometry_msgs::Point32 p1;
    p1.x = 1;
    p1.y = 2;
    p1.z = 3;
    
    geometry_msgs::Point32 p2 = p1;
    types::CorrPair corr = { &p1, &p2 };
    
    misc::updateCenters(center, center2, corr);
    
	ASSERT_TRUE(center.x == 2.f);
	ASSERT_TRUE(center.y == 3.f);
	ASSERT_TRUE(center.z == 4.f);
	ASSERT_TRUE(center2.x == 2.f);
	ASSERT_TRUE(center2.y == 3.f);
	ASSERT_TRUE(center2.z == 4.f);
	
	misc::avgCenters(center, center2, 2);
	ASSERT_FLOAT_EQ(center.x, 1.f);
	ASSERT_FLOAT_EQ(center.y, 1.5f);
	ASSERT_FLOAT_EQ(center.z, 2.f);
	ASSERT_FLOAT_EQ(center2.x, 1.f);
	ASSERT_FLOAT_EQ(center2.y, 1.5f);
	ASSERT_FLOAT_EQ(center2.z, 2.f);
}

TEST(GeometryTest, rotation_only)
{
	using namespace registration;
	
	Eigen::Vector3f trans{0, 0, M_PI};
	
	{
		auto rot = geometry::createRotationMatrix(trans);
		ASSERT_NEAR(rot(0, 0), -1.f, epsilon);
		ASSERT_NEAR(rot(0, 1), 0.f, epsilon);
		ASSERT_NEAR(rot(1, 0), 0.f, epsilon);
		ASSERT_NEAR(rot(1, 1), -1.f, epsilon);
		
		ASSERT_NEAR(rot(0, 2), 0.f, epsilon);
		ASSERT_NEAR(rot(1, 2), 0.f, epsilon);
		ASSERT_NEAR(rot(2, 0), 0.f, epsilon);
		ASSERT_NEAR(rot(2, 1), 0.f, epsilon);
		ASSERT_NEAR(rot(2, 2), 1.f, epsilon);
	}
	
	{
		auto rot = geometry::createRotationMatrix(M_PI);
		ASSERT_NEAR(rot(0, 0), -1.f, epsilon);
		ASSERT_NEAR(rot(0, 1), 0.f, epsilon);
		ASSERT_NEAR(rot(1, 0), 0.f, epsilon);
		ASSERT_NEAR(rot(1, 1), -1.f, epsilon);
		
		ASSERT_NEAR(rot(0, 2), 0.f, epsilon);
		ASSERT_NEAR(rot(1, 2), 0.f, epsilon);
		ASSERT_NEAR(rot(2, 0), 0.f, epsilon);
		ASSERT_NEAR(rot(2, 1), 0.f, epsilon);
		ASSERT_NEAR(rot(2, 2), 1.f, epsilon);
	}
}

TEST(GeometryTest, rotation_and_translation)
{
	using namespace registration;
	
	Eigen::Vector3f trans{1, 3, M_PI};
	
	auto rot = geometry::createRotationMatrix(trans);
	ASSERT_NEAR(rot(0, 0), -1.f, epsilon);
	ASSERT_NEAR(rot(0, 1),  0.f, epsilon);
	ASSERT_NEAR(rot(1, 0),  0.f, epsilon);
	ASSERT_NEAR(rot(1, 1), -1.f, epsilon);
	
	ASSERT_NEAR(rot(0, 2), 1.f, epsilon);
	ASSERT_NEAR(rot(1, 2), 3.f, epsilon);
	ASSERT_NEAR(rot(2, 0), 0.f, epsilon);
	ASSERT_NEAR(rot(2, 1), 0.f, epsilon);
	ASSERT_NEAR(rot(2, 2), 1.f, epsilon);
}

TEST(GeometryTest, euclideanDistance)
{
	using namespace registration;
	
	geometry_msgs::Point32 p1;
	p1.x = 1;
	p1.y = 1;
	p1.z = 1;
	
	geometry_msgs::Point32 p2 = p1;
	
	ASSERT_NEAR(geometry::euclideanDistance(&p1, &p2), 0.f, epsilon);
	
	p2.x = 2;
	p2.y = 2;
	p2.z = 2;
	
	ASSERT_NEAR(geometry::euclideanDistance(&p1, &p2), std::sqrt(3.0f), epsilon);
	
	p2.x = 3;
	p2.y = 2;
	p2.z = 1;
	
	ASSERT_NEAR(geometry::euclideanDistance(&p1, &p2), std::sqrt(5.0f), epsilon);
	
	ASSERT_FLOAT_EQ(geometry::euclideanDistance(&p1, nullptr), 0.f);
	
	ASSERT_FLOAT_EQ(geometry::euclideanDistance(nullptr, &p1), 0.f);
	
	ASSERT_FLOAT_EQ(geometry::euclideanDistance(nullptr, nullptr), 0.f);
}

TEST(GeometryTest, transformPointCloud)
{
	using namespace registration;
	
	// Pointclound in Ebene with 2 points
	sensor_msgs::PointCloud2 cloud;
	cloud.header = std_msgs::Header();
	cloud.height = 1;
	
	cloud.width = 2;
	
	// Define point structure
	using MyPoint = Eigen::Vector3f;
	
	cloud.point_step = sizeof(MyPoint);
	cloud.row_step = cloud.width * cloud.point_step;
	
	cloud.data.resize(cloud.width * cloud.height * sizeof(MyPoint));
	
	// Describe your MyPoint object
	cloud.fields.resize(3);
	// MyPoint.x
	cloud.fields[0].name = "x";
	cloud.fields[0].offset = 0;
	cloud.fields[0].datatype = sensor_msgs::PointField::FLOAT32;
	cloud.fields[0].count = 1;
	// MyPoint.y
	cloud.fields[1].name = "y";
	cloud.fields[1].offset = 4;
	cloud.fields[1].datatype = sensor_msgs::PointField::FLOAT32;
	cloud.fields[1].count = 1;
	// MyPoint.z
	cloud.fields[2].name = "z";
	cloud.fields[2].offset = 8;
	cloud.fields[2].datatype = sensor_msgs::PointField::FLOAT32;
	cloud.fields[2].count = 1;
	
	auto* points = reinterpret_cast<MyPoint*>(cloud.data.data());
	points[0] = MyPoint{1, 1, 0};
	points[1] = MyPoint{2, 2, 0};
	
	Eigen::Vector3f trans{1, 1, 0};
	
	geometry::transformPointCloud(trans, cloud);
	
	ASSERT_NEAR(points[0].x(), 2.f, epsilon);
	ASSERT_NEAR(points[0].y(), 2.f, epsilon);
	ASSERT_NEAR(points[0].z(), 0.f, epsilon);
	ASSERT_NEAR(points[1].x(), 3.f, epsilon);
	ASSERT_NEAR(points[1].y(), 3.f, epsilon);
	ASSERT_NEAR(points[1].z(), 0.f, epsilon);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "registration_test");
    if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug))
    {
        ros::console::notifyLoggerLevelsChanged(); // To show debug output in the tests
    }

    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}

