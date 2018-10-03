#include "viconInterp.hpp"


namespace vicon_hand
{

/* Initialize and read ros params.  In particular, read the list of topics and create a subscriber for each.  The 
MessageEvent syntax is used to share a common callback.  It is assumed that topic names exactly match those provided
in the .launch file, including the '/'.  If not, the topic matching will fail.*/
viconInterpreter::viconInterpreter(ros::NodeHandle &nh)
{
    std::vector<std::string> objNames;
    nh.getParam("vicon_to_hand/topicList", objNames);

    // Get size of objNames
    numTopics_ = objNames.size();
    // For each name, create subscriber and fill class list.
    allTopicNames_.resize(numTopics_);
    for(int ij=0; ij<numTopics_; ij++)
    {
        allFingerBools_[ij] = false;
        allTopicNames_[ij] = (objNames[ij]).c_str();
        poseSub_[ij] = nh.subscribe(allTopicNames_[ij],10,&viconInterpreter::poseCallback, this,
                ros::TransportHints().unreliable()); //use UDP to avoid meltdown phenomenon
    }
    timerPub_ = nh.createTimer(ros::Duration(1.0/20.0), &viconInterpreter::timerCallback, this, false);
    handPosePub_ = nh.advertise<vicon_hand::handMsg>("/handPoseMsgs",10);
    ROS_INFO("Vicon to hand startup complete.");
}


/* Callback to record most recent positions for each object.  The MessageEvent syntax is used to allow a
common callback for all vicon marker topics. */
void viconInterpreter::poseCallback(const ros::MessageEvent<vicon::Subject const>& event)
{
	const vicon::Subject::ConstPtr& msg = event.getMessage();

    //Get topic name 
    ros::M_string& header = event.getConnectionHeader();
    std::string publisherName = header.at("topic");

    /*
    NOTE: Assumes that the topic for each object corresponds to its name, e.g.,
    the position of "left thumb" is published on /leftThumb rather than /leftThumb/pose.
    If this is done, the following code can be used instead:
    std::size_t secondSlashIndex = publisherName.find("/",2);
    publisherName = publisherName.substr(0*,secondSlashIndex-1);
    */

    //Find index in array that matches the topic name
    int nk = getIndexMatchingName(publisherName.c_str(), allTopicNames_, numTopics_);

    allFingerBools_[nk] = true;
    
    objectPositions_[nk](0) = msg->position.x;
    objectPositions_[nk](1) = msg->position.y;
    objectPositions_[nk](2) = msg->position.z;
    objectOrientations_[nk].x() = msg->orientation.x;
    objectOrientations_[nk].y() = msg->orientation.y;
    objectOrientations_[nk].z() = msg->orientation.z;
    objectOrientations_[nk].w() = msg->orientation.w;
    lastObserved_[nk] = (msg->header.stamp).toSec();

    return;
}


/*Publish hand output to whereever at a lower rate. Otherwise, this topic will publish
data at around 1k Hz.  If wifi is being used, this will induce a meltdown. */
void viconInterpreter::timerCallback(const ros::TimerEvent &event)
{
    static bool initialized(false);
    if(!initialized)
    {
        int numFingersInit;
        for(int ij=0; ij<numTopics_; ij++)
        {
            if(allFingerBools_[ij])
                {numFingersInit++;}
        }
        if(numFingersInit==numTopics_) {initialized=true;}
        else {return;}
    }

    vicon_hand::handMsg msg;
    msg.tLast.resize(numTopics_);
    msg.poseArray.resize(numTopics_);
    msg.orientationArray.resize(numTopics_);
    msg.header.stamp = ros::Time::now();
    for(int ij=0; ij<numTopics_; ij++)
    {
        msg.tLast[ij] = lastObserved_[ij];
        msg.poseArray[ij].x = objectPositions_[ij](0);
        msg.poseArray[ij].y = objectPositions_[ij](1);
        msg.poseArray[ij].z = objectPositions_[ij](2);
        msg.orientationArray[ij].x = objectOrientations_[ij].x();
        msg.orientationArray[ij].y = objectOrientations_[ij].y();
        msg.orientationArray[ij].z = objectOrientations_[ij].z();
        msg.orientationArray[ij].w = objectOrientations_[ij].w();
    }
    handPosePub_.publish(msg);
    return;
}


int viconInterpreter::getIndexMatchingName(const std::string& stringToMatch, 
        const std::vector<std::string> stringmat, const int listLen)
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




