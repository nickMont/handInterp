//This class implements the pointerFollower gesture, which asks the swarm to maintain its original relative displacement and follow the center/yaw of the pointing hand.
#include "pointerFollower.hpp"

namespace handIn
{
pointerFollower::pointerFollower()
{
	isInitialized_=false;
	hasInitSwarm_=false;
	for(int ij=0;ij<10;ij++)
	{hasPointer_[ij]=false;}
	initConfig_ = Eigen::Matrix<double,3,10>::Zero();
}


pointerFollower::pointerFollower(int numQuads)
{
	numQuads_=numQuads;
	hasInitSwarm_=false;
	isInitialized_=false;
	for(int ij=0;ij<10;ij++)
	{hasPointer_[ij]=false;}
}


void pointerFollower::setNumQuads(int numQuads)
{numQuads_=numQuads;}


//pass location in memory of quad array
void pointerFollower::setQuadPointer(const int ij, std::shared_ptr<handIn::quadContainer> quadptr)
{
	quadContainerPtr_[ij] = quadptr;
	hasPointer_[ij] = true;
	bool derp(true);
	for(int ik=0;ik<numQuads_;ik++)
	{if(!hasPointer_[ik]){derp=false;}}
	if(derp)
	{
		Eigen::Vector3d tmppos;

		for(int ij=0; ij<numQuads_; ij++)
		{
			quadContainerPtr_[ij]->getPosPointer(&tmppos);
			initConfig_(0,ij)=tmppos(0);
			initConfig_(1,ij)=tmppos(1);
			initConfig_(2,ij)=tmppos(2);
		}

		isInitialized_=true;
	}
}


//returns references as [[pos3x1; yaw] x10]
Eigen::Matrix<double,4,10> pointerFollower::returnPosRefs(const Eigen::Vector3d handCenterPos, const Eigen::Quaterniond handRot)
{
	Eigen::Matrix<double,4,10> retmat;
	retmat = Eigen::Matrix<double,4,10>::Zero();

	if(isInitialized_)
	{
		//see SE
		float yaw = atan2(2.0*(handRot.y()*handRot.z() + handRot.w()*handRot.x()), handRot.w()*handRot.w() - handRot.x()*handRot.x() - handRot.y()*handRot.y() + handRot.z()*handRot.z());
		for(int ij=0; ij<numQuads_; ij++)
		{retmat(3,ij)=yaw;} //all quads should adopt same yaw

		//modify retmat with position
		for(int ij=0; ij<numQuads_; ij++)
		{

		}

	}else
	{
		std::cout << "You tried to return references without initializing!" << std::endl;
	}
	return retmat;
}



} //ns
