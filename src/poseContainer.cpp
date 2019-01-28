#include "poseContainer.hpp"

namespace handIn
{
poseContainer::poseContainer(ros::NodeHandle &nh)
{
	nh_ = nh;
}


void poseContainer::configure(const int numQuads)
{
	for(int ij=0; ij<10; ij++)
	{
		hasInitPos_[ij] = false;
		hasPointer_[ij] = false;
	}
	numQuads_ = numQuads;

	return;
}


//pass location in memory of quad array
void poseContainer::setQuadPointer(const int ij, std::shared_ptr<handIn::quadContainer> quadptr)
{
	quadContainerPtr_[ij] = quadptr;
	hasPointer_[ij] = true;
}


//only create subscribers that are needed
void poseContainer::createPoseSub(const int ij, const std::string quadname)
{
	quadPoseSubs_[ij] = nh_.subscribe("/"+quadname+"/WRW/local_odom",10,&poseContainer::poseEventCallback,
		  this, ros::TransportHints().unreliable().reliable().tcpNoDelay(true));
	namebox_[ij] = quadname+"/WRW/local_odom";
	return;
}


//use MessageEvent syntax to create one ROS callback for all pose topics
void poseContainer::poseEventCallback(const ros::MessageEvent<nav_msgs::Odometry const>& event)
{
	 //extract message contents
	 const nav_msgs::Odometry::ConstPtr& msg = event.getMessage();

	//get topic name in callback by using messageevent syntax
  	ros::M_string& header = event.getConnectionHeader();
  	std::string publisherName = header.at("topic");
  	//remove leading "/"
  	publisherName = publisherName.substr(1,publisherName.length());
	
	//match publisher name to 
  	int nk = getIndexMatchingName(publisherName,namebox_,numQuads_);

  	if(nk==-1) //errors
  	{return;}
  	else if(!hasPointer_[nk])
  	{ROS_INFO("Quad object missing!"); return;}

  	if(!hasInitPos_[nk])
    {
        hasInitPos_[nk] = true;
    }

    quadContainerPtr_[nk]->setOdom(*msg);
    return;
}


bool poseContainer::hasData(const int index)
{
  return hasInitPos_[index];
}


//get index of quadPointer from ros subscriber name to match objects in MessageEvent callback
int poseContainer::getIndexMatchingName(const std::string& stringToMatch, 
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


int poseContainer::getIndexMatchingName(const std::string& stringToMatch, 
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
