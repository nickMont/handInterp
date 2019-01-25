#pragma once

#include <ros/ros.h>
#include <nag_msgs/odometry.h>
#include <cmath>
#include <string>
#include <iostream>
#include <Eigen/Geometry>

namespace handIn
{
class quadContainer;

class commander
{
public:
	commander(ros::nodeHandle &nh);
	void configure(const int nk, const std::string &names[]);
	void getGestureList(const std::string &filename);
	void statusTimerCallback(const ros::TimerEvent &event);
	void setQuadPointer(const std::string &name, std::shared_ptr<handIn::quadContainer> commptr);
	void sendHand(const double &handData[55]);
	void matchAndPerformAction(int rR, int lL);
	int getIndexMatchingName(const std::string& stringToMatch, 
        const std::vector<std::string>& stringmat, const int listLen);
	int findIndexInList(int match, const Eigen::MatrixXd &ref, const int colno);
	Eigen::VectorXd getRefsForCircularFlight(const double zFlight, const double omega, const double turnRadius);
	Eigen::VectorXd leastSquares(const Eigen::MatrixXd &A, const Eigen::VectorXd &z);

private:
	ros::nodeHandle nh_;
	double hand_[55];
	int numQuads_;
	bool isConfigured_, hasName_[10], isInitialized_[10], hasPtr_[10], isGreen_;
	std::string quadList_[10];

}