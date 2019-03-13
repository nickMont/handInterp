#pragma once

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <cmath>
#include <string>
#include <iostream>
#include <Eigen/Geometry>
#include "quadContainer.hpp"
#include <mg_msgs/PVA.h>

namespace handIn
{
class commander
{
public:
	typedef Eigen::Matrix<double, 55, 1> Vector55d;
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	commander();
	//Configuration and interface functions
	void setnodehandle(ros::NodeHandle &nh);
	void configure(const int nk, const std::string names[10]);
	void getGestureList(const std::string &filename);
	void statusTimerCallback(const ros::TimerEvent &event);
	void setQuadPointer(const std::string &name, std::shared_ptr<handIn::quadContainer> commptr);
	void sendHand(const Vector55d &handData);

	//Overloaded reference publisher function for convenience.  PV, PVA, PVAy
	void publishAllReferences(const Eigen::MatrixXd &posRefs, const Eigen::MatrixXd &velRefs);
	void publishAllReferences(const Eigen::MatrixXd &posRefs, const Eigen::MatrixXd &velRefs, const Eigen::MatrixXd &accRefs);
	void publishAllReferences(const Eigen::MatrixXd &posRefs, const Eigen::MatrixXd &velRefs, const Eigen::MatrixXd &accRefs,
		const Eigen::MatrixXd &yawRefs);

	//misc
	void matchAndPerformAction(int rR, int lL);
	int getIndexMatchingName(const std::string& stringToMatch, 
        const std::vector<std::string>& stringmat, const int listLen);
	int getIndexFromNamelist(const std::string& stringToMatch, const int listLen);
	int findIndexInList(int match, const Eigen::Matrix<int,100,2> &ref, const int colno);
	Eigen::VectorXd leastSquares(const Eigen::MatrixXd &A, const Eigen::VectorXd &z);

	//Gesture-based actions
	Eigen::VectorXd getRefsForCircularFlight(const double zFlight, const double omega, const double turnRadius);

private:
	ros::NodeHandle nh_;
	Vector55d hand_;
	int numQuads_, numGesturesInCatalog_;
	bool isConfigured_, hasName_[10], isInitialized_[10], hasPtr_[10], isGreen_, gestureHasBeenInitialized_;
	std::string quadList_[10];
	ros::Timer statusTimer_;
	ros::Publisher pvaPub_[10];
	std::shared_ptr<handIn::quadContainer> quadPoseContainer_[10];
	Eigen::Matrix<int,100,2> gesturePairingsRight_, gesturePairingsLeft_;

};

} //ns