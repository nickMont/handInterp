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
class quadContainer
{
public:
	quadContainer(ros::nodeHandle &nh);
	void configure();
	void setName(const string quadname);
	void setOdom(const nav_msgs::Odometry &msg);
	Eigen::Vector3d getPos();
	Eigen::Quaterniond getQuat();
	nav_msgs::Odometry getOdom();
	string getName();


private:
	nav_msgs::Odometry lastOdom_;
	Eigen::Vector3d lastPos_;
	Eigen::Quaterniond lastQuat_;
	int quadIntIndex_;
	string quadName_;
};

}