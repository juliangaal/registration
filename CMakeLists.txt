cmake_minimum_required(VERSION 3.0.2)
project(registration)

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
  pcl_ros
)

find_package(Eigen3 REQUIRED)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES registration
#  CATKIN_DEPENDS geometry_msgs roscpp rospy sensor_msgs tf2 visualization_msgs
#  DEPENDS system_lib
)

catkin_install_python(PROGRAMS scripts/pcl_publisher.py
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

add_library(registration SHARED src/registration.cpp)
target_compile_options(registration PRIVATE -Wall -Weffc++ -Wpedantic -ffast-math -Wextra)
target_include_directories(registration PRIVATE ${catkin_INCLUDE_DIRS} include)
target_link_libraries(registration PUBLIC ${catkin_LIBRARIES})

add_executable(${PROJECT_NAME}_node src/${PROJECT_NAME}_node.cpp)
target_compile_options(registration PRIVATE -Wall -Weffc++ -Wpedantic -ffast-math -Wextra)
target_include_directories(${PROJECT_NAME}_node PRIVATE ${catkin_INCLUDE_DIRS} include)
target_link_libraries(${PROJECT_NAME}_node PRIVATE ${catkin_LIBRARIES} registration)

if (CATKIN_ENABLE_TESTING)
    find_package(GTest REQUIRED)
    find_package(rostest REQUIRED)

    add_rostest_gtest(registration_test
        test/${PROJECT_NAME}_test.test
        test/${PROJECT_NAME}_test.cpp
    )

    target_include_directories(registration_test PRIVATE ${catkin_INCLUDE_DIRS} include)
    target_link_libraries(registration_test ${catkin_LIBRARIES} ${GTEST_LIBRARIES} registration)
endif()