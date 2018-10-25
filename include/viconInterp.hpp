#pragma once

#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/TransformStamped.h>
#include <vicon/Subject.h>
#include <Eigen/Geometry>
#include <vicon_hand/handMsg.h>

namespace vicon_hand
{
class viconInterpreter
{
public:
	viconInterpreter(ros::NodeHandle &nh);

	void poseCallback(const ros::MessageEvent<nav_msgs::Odometry const>& event);
	void timerCallback(const ros::TimerEvent &event);
	int getIndexMatchingName(const std::string& stringToMatch, const std::vector<std::string> stringmat, const int listLen);

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

private:
    std::vector<std::string> allTopicNames_;
    int numTopics_;
    Eigen::Vector3d objectPositions_[20];
    Eigen::Quaterniond objectOrientations_[20];
    ros::Subscriber poseSub_[20];
    ros::Publisher handPosePub_;
    ros::Timer timerPub_;
    double lastObserved_[20];
    bool allFingerBools_[20];

};
} //end namespace

