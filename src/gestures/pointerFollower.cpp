//This class implements the pointerFollower gesture, which asks the swarm to maintain its original relative displacement and follow the center/yaw of the pointing hand.
#include "pointerFollower.hpp"

namespace handIn
{
pointerFollower::pointerFollower()
{
	isInitialized_=false;
	hasInitSwarm_=false;
	hasPosePointer_=false;
	initConfig_ = Eigen::Matrix<double,3,10>::Zero();

	scalefactor_=2;
}


pointerFollower::pointerFollower(int numQuads)
{
	numQuads_=numQuads;
	hasInitSwarm_=false;
	isInitialized_=false;
	hasPosePointer_=false;
}


void pointerFollower::reinitialize()
{
	Eigen::Matrix<double,3,10> tmp(Eigen::Matrix<double,3,10>::Zero());
	poseContainerPtr_->returnAllCurrentPos(&tmp);
	initConfig_ = tmp;
	hasHandInitLoc_=false;
	isInitialized_=true;
}


void pointerFollower::reinitialize(Eigen::Vector3d &handIn)
{
	Eigen::Matrix<double,3,10> tmp(Eigen::Matrix<double,3,10>::Zero());
	poseContainerPtr_->returnAllCurrentPos(&tmp);
	initConfig_ = tmp;
	handCenterInit_ = handIn;
	hasHandInitLoc_ = true;
	isInitialized_ = true;
}


void pointerFollower::resetScalefactor(double sf)
{
	scalefactor_ = sf;
}


void pointerFollower::setNumQuads(int numQuads)
{numQuads_=numQuads;}


//pass location in memory of quad array
void pointerFollower::setQuadPointer(std::shared_ptr<handIn::poseContainer> quadptr)
{
	poseContainerPtr_ = quadptr;
	hasPosePointer_ = true;
	if(hasHandInitLoc_)
	{
		//dummyvar to prevent data mixup errors
		Eigen::Matrix<double,3,10> tmp(Eigen::Matrix<double,3,10>::Zero());
		poseContainerPtr_->returnAllCurrentPos(&tmp);
		initConfig_ = tmp;

		isInitialized_=true;
	}
}


//returns references as [[pos3x1; yaw] x10]
Eigen::Matrix<double,4,10> pointerFollower::returnPosRefs(const Eigen::Vector3d& handCenterPos, const Eigen::Quaterniond& handRot)
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
			//quadContainerPtr_[ij]->getPosPointer(&tmppos);
			retmat.block(0,ij,2,0) = initConfig_.block(0,ij,2,0) + (handCenterPos-handCenterInit_)*scalefactor_;
		}

	}else
	{
		std::cout << "You tried to return references without initializing!" << std::endl;
	}
	return retmat;
}


//returns references as [[pos3x1; yaw] x10]
Eigen::Matrix<double,7,10> pointerFollower::returnPosVelRefs(const Eigen::Vector3d& handCenterPos, const Eigen::Quaterniond& handRot, const Eigen::Vector3d& velRef)
{
	Eigen::Matrix<double,7,10> retmat;
	retmat = Eigen::Matrix<double,7,10>::Zero();

	if(isInitialized_)
	{
		//see SE
		float yaw = atan2(2.0*(handRot.y()*handRot.z() + handRot.w()*handRot.x()), handRot.w()*handRot.w() - handRot.x()*handRot.x() - handRot.y()*handRot.y() + handRot.z()*handRot.z());
		for(int ij=0; ij<numQuads_; ij++)
		{retmat(3,ij)=yaw;} //all quads should adopt same yaw

		Eigen::Vector3d tmppos, thisPos;
		//modify retmat with position
		for(int ij=0; ij<numQuads_; ij++)
		{
			//quadContainerPtr_[ij]->getPosPointer(&tmppos);
			retmat.block(0,ij,2,0) = initConfig_.block(0,ij,2,0) + (handCenterPos-handCenterInit_)*scalefactor_;
		}

	}else
	{
		std::cout << "You tried to return references without initializing!" << std::endl;
	}
	return retmat;
}


} //ns
