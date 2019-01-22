#include commander.hpp


namespace handIn
{

commander::commander(ros::NodeHandle &nh)
{
	nh_ = &nh;
	statusTimer_ = nh_.createTimer(ros::Duration(1.0/2.0), &commander::statusTimerCallback, this, false);
}


void commander::configure(const int nk, const std::string &names[])
{
	isConfigured_ = true;
	numQuads_ = nk;
 	for(int ij=0; ij<nk; ij++)
 	{
 		quadList_[ij] = names[ij];
 		hasName_[ij] = true;
 	}
}


void commander::getGestureList(const std::string &filename)
{
	//file reader goes here <<>>

	
	nrow = dat(0);
	gesturePairings.resize(nrow,3);
}


//note: isGreen_ is the "all good to go" status flag.
//  All other events will be halted if isGreen_=false
void commander::statusTimerCallback(const ros::TimerEvent &event)
{
	if(isGreen_)
	{return;}
	
	if(isConfigured_)
	{
		for(int ij=0;ij<numQuads_;ij++)
		{
			if(hasPtr_[ij]) //don't reference a null pointer
			{
				if(quadPoseContainer_[ij]->getStatus() && hasName_[ij]) //if poses are good
				{
					isInitialized_[ij] = true;
				}
			}
		}
	}

	//second operation, run double check to see if all are good
	if(isConfigured_ && !isGreen_)
	{
		allTrue = true;
		for(int ij=0;ij<numQuads_;ij++)
		{
    		if(isInitialized_[ij]){allTrue = false;}
    	}
    	isGreen_=allTrue;
	}
	// ^ can be done in first loop but splitting it out makes it easier to read

	return;
}


void commander::setQuadPointer(const std::string &name, std::shared_ptr<handIn::quadContainer> commptr)
{
	int nk = getIndexMatchingName(name, quadList_, numQuads_);

	if(nk==-1)
	{ROS_INFO("Bad quad name while initalizing commander."); return;}

	quadPoseContainer_[nk] = commptr;
	hasPtr_[nk] = true;
	if(hasName_[nk] && commptr->getStatus())
	{
		isInitialized_[ij] = true;
	}
}


void commander::sendHand(const float &handData[55])
{
	int rGest = round(handData[53]);
	int lGest = round(handData[54]);
	matchAndPerformAction(rGest, lGest);
}


void commander::matchAndPerformAction(int rR, int lL)
{

}


//get index of quadPointer from ros subscriber name to match objects in MessageEvent callback
int commander::getIndexMatchingName(const std::string& stringToMatch, 
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


} //ns
