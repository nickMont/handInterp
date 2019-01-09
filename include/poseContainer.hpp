#pragma once

#include <ros/ros.h>
#include <nag_msgs/odometry.h>
#include <cmath>
#include <string>
#include <iostream>
#include <Eigen/Geometry>

namespace handIn
{
//forward declare quadContainer
class quadContainer;

class poseContainer
{
public:
	poseContainer(ros::nodeHandle &nh);
	void configure();
	void setQuadPointer(std::shared_ptr<handIn::quadContainer> quadptr);
	void createPoseSub (const int ij, const string quadname);

private:
	void poseEventCallback(const ros::MessageEvent<nav_msgs::Odometry const>& event);
	int getIndexMatchingName(const std::string& stringToMatch, 
		const std::vector<std::string>& stringmat, const int listLen);

	std::shared_ptr<handIn::quadContainer> quadContainerPtr_;
	std::string namebox_[10];
	ros::Subscriber quadPoseSubs_[10];
	bool hasInitPose_[10], hasPointer_[10];
	ros::nodeHandle nh_;
	int numQuads_;
};

}