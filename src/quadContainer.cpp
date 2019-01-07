#pragma once
#include quadContainer.hpp


namespace handIn
{

quadContainer::quadContainer(ros::NodeHandle &nh)
{
	nh_ = &nh;
}


void quadContainer::configure(const int nk)
{
	quadIntIndex_ = nk;
}


void quadContainer::setName(const string quadname)
{
	quadName_ = quadname;
}


void quadContainer::setOdom(const nav_msgs::Odometry &msg)
{
	lastOdom_ = msg;
	lastPos_(0) = lastOdom_.pose.pose.position.x;
	lastPos_(1) = lastOdom_.pose.pose.position.y;
	lastPos_(2) = lastOdom_.pose.pose.position.z;
	lastQuat_.x() = lastOdom_.pose.pose.orientation.x;
	lastQuat_.y() = lastOdom_.pose.pose.orientation.y;
	lastQuat_.z() = lastOdom_.pose.pose.orientation.z;
	lastQuat_.w() = lastOdom_.pose.pose.orientation.w;
}


Eigen::Vector3d quadContainer::getPos()
{
	return lastPos_;
}


Eigen::Quaterniond quadContainer::getQuat()
{
	return lastQuat_;
}


nav_msgs::Odometry quadContainer::getOdom()
{
	return lastOdom_;
}


string quadContainer::getName()
{
	return quadName_;
}


} //ns
