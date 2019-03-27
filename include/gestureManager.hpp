#pragma once

#include <cmath>
#include <string>
#include <iostream>
#include <Eigen/Geometry>
#include <pointerFollower.hpp>
#include <poseContainer.hpp>

namespace handIn
{

class gestureManager
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	gestureManager();
	void configure(const int numQuads);
	void setQuadPointer(std::shared_ptr<handIn::poseContainer> quadptr);

private:
	pointerFollower pointerFollower_;
	std::shared_ptr<handIn::poseContainer> poseContainerPtr_;

};

}