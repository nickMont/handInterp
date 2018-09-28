#include <Eigen/Geometry>
#include "viconInterp.hpp"
#include <string>
#include <iostream>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "vicon_hand");
    ros::NodeHandle nh;

    try
    {
        vicon_hand::viconInterpreter vicon_hand(nh);
        ros::spin();
    }
    catch(const std::exception &e)
    {
        ROS_ERROR("%s: %s", nh.getNamespace().c_str(), e.what());
        return 1;
    }
    return 0;
}