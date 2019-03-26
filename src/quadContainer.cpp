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


void quadContainer::setName(const int nk, const std::string quadname)
{
	quadIntIndex_ = nk;
	quadName_ = quadname;
}

//unused, for now
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
	lastVel_(0) = lastOdom_.twist.twist.linear.x;
	lastVel_(1) = lastOdom_.twist.twist.linear.y;
	lastVel_(2) = lastOdom_.twist.twist.linear.z;
}


void quadContainer::setOdom(const nav_msgs::Odometry &msg)
{
	lastOdom_ = msg;
}


void quadContainer::setInitPos(const Eigen::Vector3d &pos)
{
	lastPos_(0) = lastOdom_.pose.pose.position.x;
	lastPos_(1) = lastOdom_.pose.pose.position.y;
	lastPos_(2) = lastOdom_.pose.pose.position.z;
	initPos_ = lastPos_;
	lastVel_=Eigen::Vector3d::Zero();
}


Eigen::Vector3d quadContainer::getPos()
{
	return lastPos_;
}

void quadContainer::getPosPointer(Eigen::Vector3d *tmp)
{
	Eigen::Vector3d ret(lastPos_);
	*tmp = ret;
}

void quadContainer::getVelPointer(Eigen::Vector3d *tmp)
{
	Eigen::Vector3d ret(lastVel_);
	*tmp = ret;
}

void quadContainer::getQuatPointer(Eigen::Quaterniond *tmp)
{
	Eigen::Quaterniond ret(lastQuat_);
	*tmp = ret;
}

void quadContainer::getYawPointer(double *tmp)
{
	double ret = atan2(2.0*(lastQuat_.y()*lastQuat_.z() + lastQuat_.w()*lastQuat_.x()),
		lastQuat_.w()*lastQuat_.w() - lastQuat_.x()*lastQuat_.x() - lastQuat_.y()*lastQuat_.y() + lastQuat_.z()*lastQuat_.z());
	*tmp = ret;
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
