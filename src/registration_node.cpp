
/**
  * @file registration_node.cpp
  * @author julian gaal
  * @date 2/9/21
 */

#include <ros/ros.h>
#include <registration/registration.h>

using namespace registration;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "registration");
    ros::NodeHandle nh;

    Registration registration(nh, "pcl", 10, 1.f);
    ros::spin();

    return 0;
}