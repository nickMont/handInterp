#pragma once

#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/TransformStamped.h>
#include <Eigen/Geometry>
#include <mg_msgs/PVA.h>
#include <vicon_interp/handMsg.h>

namespace vicon_hand
{
class viconHand
{
public:
	viconHand(ros::NodeHandle &nh);

	void handCallback(const vicon_interp::handMsg::ConstPtr &msg);

private:
    ros::Subscriber handSub_;
    ros::Publisher pvaPub_;
    double scalefactor_;
    int numFingers_;
    Eigen::Vector3d handVec0_;

};

} //end namespace

