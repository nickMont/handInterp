#include "commander.hpp"
#include <cstdio>
#include <vector>

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
			quadPoseContainer_[ij]->getPosPointer(&tmp);
			currentPoses(3*ij+0) = tmp(0);
			currentPoses(3*ij+1) = tmp(1);
			currentPoses(3*ij+2) = tmp(2);
		}
		gestureHasBeenInitialized_=true;

		/*//Figure out which quad should occupy which position in the new "circle"
		Eigen::MatrixXd A;
		A.resize(numQuads_*3, numQuads_*3);
		for(int ij=0; ij<numQuads_*3; ij++) {A(ij,ij)=1;} //Eigen::Diag does not fully initialize off-diagonal entries which causes errors when inverting
		Eigen::VectorXd tmp2;
		tmp2.resize(numQuads_*3);
		tmp2 = leastSquares(A,currentPoses);*/

		int numconfigs = numQuads_;
		for(int ij=0; ij<numconfigs; ij++)
		{
			//initialization goes here
		}

	}
	if(omega!=0) //rotational speed of circle
	{

	}
	return Eigen::Vector3d(9001,9001,9001);
}


//talks to gestureManager object. gestureManager manages all gesture objects and returns references based on input gesture
void commander::processGesture(const int gestureIndex)
{

}



/*
Eigen::VectorXd commander::getMinimumDistanceInitialConfiguration(const Eigen::VectorXd &gesture)
{}
*/


//Overloaded function handle to handle publishing references. Call as:
// 2 inputs:  P,V
// 3 inputs:  P,V,A
// 4 inputs:  P,V,A,y
void commander::publishAllReferences(const Eigen::MatrixXd &posRefs, const Eigen::MatrixXd &velRefs)
{
	mg_msgs::PVA pvaMsg;

	for(int ij=0; ij<numQuads_; ij++)
	{
		pvaMsg.Pos.x = posRefs(3*ij+0);
		pvaMsg.Pos.y = posRefs(3*ij+1);
		pvaMsg.Pos.z = posRefs(3*ij+2);
		pvaMsg.Vel.x = velRefs(3*ij+0);
		pvaMsg.Vel.y = velRefs(3*ij+1);
		pvaMsg.Vel.z = velRefs(3*ij+2);
		pvaMsg.Acc.x = 0;
		pvaMsg.Acc.y = 0;
		pvaMsg.Acc.z = 0;
		pvaMsg.yaw = 0.0;

		pvaPub_[ij].publish(pvaMsg);
	}
}


void commander::publishAllReferences(const Eigen::MatrixXd &posRefs, const Eigen::MatrixXd &velRefs, const Eigen::MatrixXd &accRefs)
{
	mg_msgs::PVA pvaMsg;

	for(int ij=0; ij<numQuads_; ij++)
	{
		pvaMsg.Pos.x = posRefs(3*ij+0);
		pvaMsg.Pos.y = posRefs(3*ij+1);
		pvaMsg.Pos.z = posRefs(3*ij+2);
		pvaMsg.Vel.x = velRefs(3*ij+0);
		pvaMsg.Vel.y = velRefs(3*ij+1);
		pvaMsg.Vel.z = velRefs(3*ij+2);
		pvaMsg.Acc.x = accRefs(3*ij+0);
		pvaMsg.Acc.y = accRefs(3*ij+1);
		pvaMsg.Acc.z = accRefs(3*ij+2);
		pvaMsg.yaw = 0.0;

		pvaPub_[ij].publish(pvaMsg);
	}
}


void commander::publishAllReferences(const Eigen::MatrixXd &posRefs, const Eigen::MatrixXd &velRefs, const Eigen::MatrixXd &accRefs,
	const Eigen::MatrixXd &yawRefs)
{
	mg_msgs::PVA pvaMsg;

	for(int ij=0; ij<numQuads_; ij++)
	{
		pvaMsg.Pos.x = posRefs(3*ij+0);
		pvaMsg.Pos.y = posRefs(3*ij+1);
		pvaMsg.Pos.z = posRefs(3*ij+2);
		pvaMsg.Vel.x = velRefs(3*ij+0);
		pvaMsg.Vel.y = velRefs(3*ij+1);
		pvaMsg.Vel.z = velRefs(3*ij+2);
		pvaMsg.Acc.x = accRefs(3*ij+0);
		pvaMsg.Acc.y = accRefs(3*ij+1);
		pvaMsg.Acc.z = accRefs(3*ij+2);
		pvaMsg.yaw = yawRefs(ij);

		pvaPub_[ij].publish(pvaMsg);
	}
}


Eigen::VectorXd commander::leastSquares(const Eigen::MatrixXd &A, const Eigen::VectorXd &z)
{
	//size checks here
	if(A.rows()-z.rows() !=0)
	{
		return (A.transpose()*A).inverse()*A.transpose()*z;
	}
	else
	{return Eigen::Vector3d(9001,9001,9001);} //standard error return
}


} //end ns

