
/**
  * @file tsdfslam_test.cpp
  * @author julian 
  * @date 3/22/21
 */

#include <ros/ros.h>
#include <gtest/gtest.h>
#include <tsdfslam/local_map.h>
#include <Eigen/Dense>

constexpr float epsilon = 0.001;

TEST(LocalMapTest, local_map)
{
	using namespace tsdfslam;

	LocalMap<20, 20, 20> map;
	ASSERT_EQ(map.size(), 20 * 20 * 20);
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "tsdfslam_test");
	if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug))
	{
		ros::console::notifyLoggerLevelsChanged(); // To show debug output in the tests
	}
	
	testing::InitGoogleTest(&argc, argv);
	return RUN_ALL_TESTS();
}

