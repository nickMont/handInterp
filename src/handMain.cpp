#include <Eigen/Geometry>
#include "handInterp.hpp"
#include <string>
#include <iostream>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "hand_to_quad");
    ros::NodeHandle nh;

    try
    {
        vicon_hand::viconHand vicon_hand(nh);
        ros::spin();
    }
    catch(const std::exception &e)
    {
        ROS_ERROR("%s: %s", nh.getNamespace().c_str(), e.what());
        return 1;
    }
    return 0;
}