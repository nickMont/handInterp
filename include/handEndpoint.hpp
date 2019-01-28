#pragma once

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <cmath>
#include <iostream>
#include <Eigen/Geometry>
#include <unistd.h> 
#include <csignal>
#include <stdio.h> 
#include <sys/socket.h> 
#include <stdlib.h> 
#include <netinet/in.h> 
#include <string.h>
#include <boost/program_options.hpp>
#include "hand_endpoint/hands.h"
#include "hand_endpoint/gesture.h"

namespace handIn
{
class commander;

class handEndpoint
{
public:
	typedef Eigen::Matrix<double, 14, 1> Eigen14d;
	typedef Eigen::Matrix<double, 55, 1> Eigen55d;
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	handEndpoint(ros::NodeHandle &nh);
	void configure(const int useROS, const std::string topicOrPortName, const int getGestureFromNN, const std::string gestureTopic);
	void setCommanderPtr(std::shared_ptr<handIn::commander> commptr);
	void handAction(const float &datmat);
	double getRosTime();
	void gestureCallback(const hand_endpoint::gesture::ConstPtr &msg);
	void createPipeAndSpin(const int PORT);
	void handCallback(const hand_endpoint::hands::ConstPtr &msg);

private:
	ros::NodeHandle nh_;
	ros::Subscriber handSub_, gsSub_, endpointFromNN_, handCenterSubRight_, handCenterSubLeft_;
	ros::Publisher endpointToNN_;
	int gestureInput_, thisGestR_, thisGestL_;
	std::shared_ptr<handIn::commander> commander_;
	double lastTProc_, gestTime_;

};

}