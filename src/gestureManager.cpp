#include "gestureManager.hpp"

namespace handIn
{
gestureManager::gestureManager()
{
    pointerFollower_();
}


void gestureManager::configure(const int numQuads)
{
	numQuads_ = numQuads;

	return;
}


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







//get index of quadPointer from ros subscriber name to match objects in MessageEvent callback
int gestureManager::getIndexMatchingName(const std::string& stringToMatch, 
        const std::vector<std::string>& stringmat, const int listLen)
{
    int nn(-1); //default retval of error

    for(int ij=0;ij<listLen;ij++)
    {
        if(strcmp( stringToMatch.c_str() , (stringmat[ij]).c_str() )==0)
        {
            nn=ij;
        }
    }

    return nn;
}


int gestureManager::getIndexMatchingName(const std::string& stringToMatch, 
        const std::string stringmat[10], const int listLen)
{
    int nn(-1); //default retval of error

    for(int ij=0;ij<listLen;ij++)
    {
        if(strcmp( stringToMatch.c_str() , (stringmat[ij]).c_str() )==0)
        {
            nn=ij;
        }
    }

    return nn;
}



} //ns
