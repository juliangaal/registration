
/**
  * @file tsdfslam_test.cpp
  * @author julian 
  * @date 3/22/21
 */

#include <ros/ros.h>
#include <gtest/gtest.h>
#include <tsdfslam/local_map.h>
#include <Eigen/Dense>
#include <cassert>

constexpr float epsilon = 0.001;

TEST(LocalMapTest, local_map)
{
	using namespace tsdfslam;

	LocalMap<3, 3, 3> map;
	ASSERT_EQ(map.size(), 3 * 3 * 3);

	for (const auto& value: map.map())
    {
	    ASSERT_EQ(value, 0.0f);
    }

    {
        auto[x, y, z] = map.shift_to_local_map({0, 0, 0});
        ASSERT_EQ(x, 1);
        ASSERT_EQ(y, 1);
        ASSERT_EQ(z, 1);
    }

    {
        auto[x, y, z] = map.shift_to_local_map({-1, -1, 0});
        ASSERT_EQ(x, 0);
        ASSERT_EQ(y, 0);
        ASSERT_EQ(z, 1);
    }

    {
        auto[x, y, z] = map.shift_to_local_map({-1, -1, -1});
        ASSERT_EQ(x, 0);
        ASSERT_EQ(y, 0);
        ASSERT_EQ(z, 0);
    }

    {
        auto[x, y, z] = map.shift_to_local_map({1, 1, 1});
        ASSERT_EQ(x, 2);
        ASSERT_EQ(y, 2);
        ASSERT_EQ(z, 2);
    }

    {
        auto[x, y, z] = map.shift_to_local_map({1, 1, -1});
        ASSERT_EQ(x, 2);
        ASSERT_EQ(y, 2);
        ASSERT_EQ(z, 0);
    }
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

