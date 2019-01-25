#pragma once

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
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
	poseContainer(ros::NodeHandle &nh);
	void configure();
	void setQuadPointer(std::shared_ptr<handIn::quadContainer> quadptr);
	void createPoseSub (const int ij, const string quadname);
	bool hasData(const int index);

private:
	void poseEventCallback(const ros::MessageEvent<nav_msgs::Odometry const>& event);
	int getIndexMatchingName(const std::string& stringToMatch, 
		const std::vector<std::string>& stringmat, const int listLen);

	std::shared_ptr<handIn::quadContainer> quadContainerPtr_;
	std::string namebox_[10];
	ros::Subscriber quadPoseSubs_[10];
	bool hasInitPose_[10], hasPointer_[10];
	ros::NodeHandle nh_;
	int numQuads_;
};

}