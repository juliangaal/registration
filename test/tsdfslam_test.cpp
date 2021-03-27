
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
	constexpr static size_t SX = 3;
	constexpr static size_t SY = 3;
	constexpr static size_t SZ = 3;
	LocalMap<SX, SY, SZ> map;
	
	ASSERT_EQ(map.size(), SX * SY * SZ);

	for (const auto& value: map.map())
    {
	    ASSERT_EQ(value, 0.0f);
    }

    {
        auto center_shifted =  map.shift_to_local_map({0, 0, 0});
        ASSERT_EQ(center_shifted[0], 1);
        ASSERT_EQ(center_shifted[1], 1);
        ASSERT_EQ(center_shifted[2], 1);
    }

    {
        auto center_shifted =  map.shift_to_local_map({-1, -1, 0});
        ASSERT_EQ(center_shifted[0], 0);
        ASSERT_EQ(center_shifted[1], 0);
        ASSERT_EQ(center_shifted[2], 1);
    }

    {
        auto center_shifted =  map.shift_to_local_map({-1, -1, -1});
        ASSERT_EQ(center_shifted[0], 0);
        ASSERT_EQ(center_shifted[1], 0);
        ASSERT_EQ(center_shifted[2], 0);
    }

    {
        auto center_shifted =  map.shift_to_local_map({1, 1, 1});
        ASSERT_EQ(center_shifted[0], 2);
        ASSERT_EQ(center_shifted[1], 2);
        ASSERT_EQ(center_shifted[2], 2);
    }

    {
        auto center_shifted =  map.shift_to_local_map({1, 1, -1});
        ASSERT_EQ(center_shifted[0], 2);
        ASSERT_EQ(center_shifted[1], 2);
        ASSERT_EQ(center_shifted[2], 0);
    }
	
	// center in map should be center in representation
	{
		int tsdf_index = map._3d21d({0, 0, 0});
		ASSERT_EQ(map.size() / 2, tsdf_index);
	}
	
	// Move along x axis --> increment 1
	{
		int start_index = map.size() / 2;
		int index_increment = 1;
		
		
		for (int i = 0; i < SX; ++i)
		{
			int tsdf_index = map._3d21d({i, 0, 0});
			
			if (i <= SX / 2)
			{
				ASSERT_EQ(tsdf_index, start_index + i * index_increment);
			}
			else
			{
				ASSERT_EQ(tsdf_index, -1);
			}
		}
	}
	
	// Move along y axis --> increment SY
	{
		int start_index = map.size() / 2 + SY;
		int index_increment = SY;


		for (int i = 1; i < SX; ++i)
		{
			int tsdf_index = map._3d21d({0, i, 0});

			if (i <= SY / 2)
			{
				ASSERT_EQ(tsdf_index, start_index + (i-1) * index_increment);
			}
			else
			{
				ASSERT_EQ(tsdf_index, -1);
			}
		}
	}
	
	// Move along z axis --> increment SY * SZ
	{
		int start_index = map.size() / 2 + SY*SZ;
		int index_increment = SY*SZ;
		
		
		for (int i = 1; i < SZ; ++i)
		{
			int tsdf_index = map._3d21d({0, 0, i});
			
			if (i <= SY / 2)
			{
				ASSERT_EQ(tsdf_index, start_index + (i-1) * index_increment);
			}
			else
			{
				ASSERT_EQ(tsdf_index, -1);
			}
		}
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

