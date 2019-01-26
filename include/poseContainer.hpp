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
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	poseContainer(ros::NodeHandle &nh);
	void configure(const int numQuads);
	void setQuadPointer(const int ij, std::shared_ptr<handIn::quadContainer> quadptr);
	void createPoseSub (const int ij, const std::string quadname);
	bool hasData(const int index);

private:
	void poseEventCallback(const ros::MessageEvent<nav_msgs::Odometry const>& event);
	int getIndexMatchingName(const std::string& stringToMatch, 
		const std::vector<std::string>& stringmat, const int listLen);
	int getIndexMatchingName(const std::string& stringToMatch, 
        const std::string stringmat[10], const int listLen);

	std::shared_ptr<handIn::quadContainer> quadContainerPtr_[10];
	std::string namebox_[10];
	ros::Subscriber quadPoseSubs_[10];
	bool hasInitPos_[10], hasPointer_[10];
	ros::NodeHandle nh_;
	int numQuads_;
};

}