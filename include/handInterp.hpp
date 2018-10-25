#pragma once

#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/TransformStamped.h>
#include <nav_msgs/Odometry.h>
#include <Eigen/Geometry>
#include <mg_msgs/PVA.h>
#include <vicon_hand/handMsg.h>

namespace vicon_hand
{
class viconHand
{
public:
	viconHand(ros::NodeHandle &nh);

	void handCallback(const vicon_hand::handMsg::ConstPtr &msg);
	int getIndexMatchingName(const std::string& stringToMatch, 
		const std::vector<std::string> stringmat, const int listLen);
	void poseCallback(const ros::MessageEvent<nav_msgs::Odometry const>& event);

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

private:
    ros::Subscriber handSub_, quadPoseSub_[20];
    ros::Publisher pvaPub_[20];
    double scalefactor_;
    int numFingers_, numQuads_;
    Eigen::Vector3d handVec0_, initPos_[20];
    std::vector<std::string> quadTopics_;
    bool hasInitPos_[20];

};
} //end namespace

