#pragma once

#include <ros/ros.h>
#include <nag_msgs/odometry.h>
#include <cmath>
#include <string>
#include <iostream>
#include <Eigen/Geometry>

namespace handIn
{
class quadContainer
{
public:
	quadContainer(ros::nodeHandle &nh);
	void configure();
	void setName(const string quadname);
	void odomCallback(const nav_msgs::Odometry &msg);
	void setInitPos(const Eigen::Vector3d &pos);
	Eigen::Vector3d getPos();
	Eigen::Vector3d getInitPos();
	Eigen::Quaterniond getQuat();
	nav_msgs::Odometry getOdom();
	string getName();


private:
	ros::nodeHandle nh_;
	nav_msgs::Odometry lastOdom_;
	Eigen::Vector3d lastPos_, initPos_;
	Eigen::Quaterniond lastQuat_;
	int quadIntIndex_;
	std::string quadName_;
};

}