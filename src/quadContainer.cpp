#include "quadContainer.hpp"

namespace handIn
{

quadContainer::quadContainer(ros::NodeHandle &nh)
{
	nh_ = nh;
	hasPose_=false;
}


void quadContainer::configure(const int nk)
{
	quadIntIndex_ = nk;
}


void quadContainer::setName(const std::string quadname)
{
	quadName_ = quadname;
}


void quadContainer::odomCallback(const nav_msgs::Odometry &msg)
{
	hasPose_ = true;
	lastOdom_ = msg;
	lastPos_(0) = lastOdom_.pose.pose.position.x;
	lastPos_(1) = lastOdom_.pose.pose.position.y;
	lastPos_(2) = lastOdom_.pose.pose.position.z;
	lastQuat_.x() = lastOdom_.pose.pose.orientation.x;
	lastQuat_.y() = lastOdom_.pose.pose.orientation.y;
	lastQuat_.z() = lastOdom_.pose.pose.orientation.z;
	lastQuat_.w() = lastOdom_.pose.pose.orientation.w;
}


void quadContainer::setInitPos(const Eigen::Vector3d &pos)
{
	lastPos_(0) = lastOdom_.pose.pose.position.x;
	lastPos_(1) = lastOdom_.pose.pose.position.y;
	lastPos_(2) = lastOdom_.pose.pose.position.z;
	initPos_ = lastPos_;
}


Eigen::Vector3d quadContainer::getPos()
{
	return lastPos_;
}


void quadContainer::getPosPointer(Eigen::Vector3d *tmp)
{
	*tmp(0) = lastPos_(0);
	*tmp(1) = lastPos_(1);
	*tmp(2) = lastPos_(2);
}


Eigen::Vector3d quadContainer::getInitPos()
{
	return initPos_;
}


Eigen::Quaterniond quadContainer::getQuat()
{
	return lastQuat_;
}


nav_msgs::Odometry quadContainer::getOdom()
{
	return lastOdom_;
}


std::string quadContainer::getName()
{
	return quadName_;
}


bool quadContainer::getStatus()
{
	return hasPose_;
}

void quadContainer::getStatusPointer(bool *tmp)
{
	*tmp = hasPose_;
}


} //ns
