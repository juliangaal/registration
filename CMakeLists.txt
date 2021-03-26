cmake_minimum_required(VERSION 3.0.2)
project(slam)

set(CMAKE_CXX_STANDARD 17)
set(CATKIN_ENABLE_TESTING ON)

find_package(catkin REQUIRED COMPONENTS
        geometry_msgs
        roscpp
        rospy
        sensor_msgs
        tf2
        visualization_msgs
        message_filters
        angles
        pcl_ros)

find_package(Eigen3 REQUIRED)

catkin_package(
        #  INCLUDE_DIRS include
        #  LIBRARIES slam
        #  CATKIN_DEPENDS geometry_msgs roscpp rospy sensor_msgs tf2 visualization_msgs
        #  DEPENDS system_lib
)

catkin_install_python(PROGRAMS scripts/pcl_publisher.py
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

add_library(icp SHARED lib/icp/icp.cpp)
target_compile_options(icp PRIVATE -Wall -Weffc++ -Wpedantic -ffast-math -Wextra)
target_include_directories(icp PRIVATE ${catkin_INCLUDE_DIRS} include)
target_link_libraries(icp PUBLIC ${catkin_LIBRARIES})

add_executable(icp_node src/icp_node.cpp)
target_compile_options(icp_node PRIVATE -Wall -Weffc++ -Wpedantic -ffast-math -Wextra)
target_include_directories(icp_node PRIVATE ${catkin_INCLUDE_DIRS} include)
target_link_libraries(icp_node PRIVATE ${catkin_LIBRARIES} icp)

add_executable(tsdf_node src/tsdf_node.cpp)
target_compile_options(tsdf_node PRIVATE -Wall -Weffc++ -Wpedantic -ffast-math -Wextra)
target_include_directories(tsdf_node PRIVATE ${catkin_INCLUDE_DIRS} ${Eigen_INCLUDE_DIRS} include)
target_link_libraries(tsdf_node PRIVATE ${catkin_LIBRARIES})

if (CATKIN_ENABLE_TESTING)
    find_package(GTest REQUIRED)
    find_package(rostest REQUIRED)

    add_rostest_gtest(icp_test
            test/icp_test.test
            test/icp_test.cpp)

    add_rostest_gtest(tsdfslam_test
            test/tsdfslam_test.test
            test/tsdfslam_test.cpp)

    target_include_directories(icp_test PRIVATE ${catkin_INCLUDE_DIRS} include)
    target_link_libraries(icp_test ${catkin_LIBRARIES} ${GTEST_LIBRARIES} icp)

    target_include_directories(tsdfslam_test PRIVATE ${catkin_INCLUDE_DIRS} include)
    target_link_libraries(tsdfslam_test ${catkin_LIBRARIES} ${GTEST_LIBRARIES})
endif ()