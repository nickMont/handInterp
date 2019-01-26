#pragma once

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <cmath>
#include <string>
#include <iostream>
#include <Eigen/Geometry>


namespace handIn
{
class quadContainer
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	quadContainer(ros::NodeHandle &nh);
	void configure();
	void setName(const std::string quadname);
	void odomCallback(const nav_msgs::Odometry &msg);
	void setInitPos(const Eigen::Vector3d &pos);
	Eigen::Vector3d getPos();
	Eigen::Vector3d getInitPos();
	Eigen::Quaterniond getQuat();
	nav_msgs::Odometry getOdom();
	std::string getName();
	bool getStatus();
	//pointer-based getters
	void getPosPointer(Eigen::Vector3d *tmp);
	void getStatusPointer(bool *tmp);

private:
	ros::NodeHandle nh_;
	nav_msgs::Odometry lastOdom_;
	Eigen::Vector3d lastPos_, initPos_;
	Eigen::Quaterniond lastQuat_;
	int quadIntIndex_;
	std::string quadName_;
	bool hasPose_;
};

}