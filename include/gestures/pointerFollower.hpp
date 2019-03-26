#pragma once

#include <ros/ros.h>
#include "poseContainer.hpp"
#include <Eigen/Geometry>

namespace handIn
{
class pointerFollower
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	pointerFollower();
	pointerFollower(int numQuads);
	void reinitialize();
	void reinitialize(Eigen::Vector3d &handIn);
	void resetScalefactor(double sf);
	void setNumQuads(int numQuads);
	void setQuadPointer(std::shared_ptr<handIn::poseContainer> quadptr);
	Eigen::Matrix<double,4,10> returnPosRefs(const Eigen::Vector3d& handCenterPos, const Eigen::Quaterniond& handRot);
	Eigen::Matrix<double,7,10> returnPosVelRefs(const Eigen::Vector3d& handCenterPos, const Eigen::Quaterniond& handRot, const Eigen::Vector3d& velRef);

private:
	//initializer bools
	bool isInitialized_, hasInitSwarm_, hasPosePointer_, hasHandInitLoc_;
	std::shared_ptr<handIn::poseContainer> poseContainerPtr_;
	int numQuads_;
	Eigen::Matrix<double,3,10> initConfig_;
	Eigen::Vector3d handCenterInit_;
	double scalefactor_;


};

} //ns