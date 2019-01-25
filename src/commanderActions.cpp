#include "commander.hpp"

namespace handIn
{
Eigen::VectorXd commander::getRefsForCircularFlight(const double zFlight, const double omega, const double turnRadius)
{
	if(!isGreen_)
	{return Eigen::Vector3d(9001,9001,9001);}
	
	Eigen::MatrixXd retvec;
	retvec.resize(numQuads_*2*3, 1); //pos, vel

	//note: the lastGesture_ -> currentGesture_ comparison should be done in switch callback
	if(!gestureHasBeenInitialized_)
	{
		Eigen::MatrixXd currentPoses;
		currentPoses.resize(numQuads_*3,1);
		Eigen::Vector3d tmp;
		for(int ij=0; ij<numQuads_; ij++)
		{
			tmp = quadPoseContainer_[ij].getPos();
			currentPoses(3*ij+0) = tmp(0);
			currentPoses(3*ij+1) = tmp(1);
			currentPoses(3*ij+2) = tmp(2);
		}
		gestureHasBeenInitialized_=true;
		Eigen::MatrixXd A;
		A.resize(numQuads_*3, numQuads_*3);
		for(int ij=0; ij<numQuads_*3; ij++) {A(ij,ij)=1;} //Eigen::Diag does not fully initialize which causes errors when inverting
	}
	if(omega==0) //rotational speed of circle
	{

	}


	return Eigen::Vector3d(9001,9001,9001);
}


Eigen::VectorXd commander::leastSquares(const Eigen::MatrixXd &A, const Eigen::VectorXd &z)
{
	//size checks here
	if(A.rows()-z.length() !=0)
	{
		return (A.transpose()*A).inverse()*A.transpose()*z;
	}
	else
	{return Eigen::Vector3d(9001,9001,9001);} //standard error return
}


} //end ns

