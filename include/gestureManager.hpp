#pragma once

#include <cmath>
#include <string>
#include <iostream>
#include <Eigen/Geometry>
#include <pointerFollower.hpp>

namespace handIn
{

class gestureManager
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	gestureManager();

private:
	pointerFollower pointerFollower_;

};

}