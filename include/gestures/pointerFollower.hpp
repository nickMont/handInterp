#pragma once

#include <ros/ros.h>
#include "quadContainer.hpp"

namespace handIn
{
class pointerFollower
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	pointerFollower();
	pointerFollower(int numQuads);
	void setNumQuads(int numQuads);
	void setQuadPointer(const int ij, std::shared_ptr<handIn::quadContainer> quadptr);
	Eigen::Matrix<double,4,10> returnPosRefs(const Eigen::Vector3d handCenterPos, const Eigen::Quaterniond handRot);

private:
	//initializer bools
	bool isInitialized_, hasInitSwarm_, hasPointer_[10];
	std::shared_ptr<handIn::quadContainer> quadContainerPtr_[10];
	int numQuads_;
	Eigen::Matrix<double,3,10> initConfig_;
	Eigen::Vector3d initHandPos_;


};

} //ns