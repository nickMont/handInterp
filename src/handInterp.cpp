#include "handInterp.hpp"

namespace vicon_hand
{

/* Initialize and read ros params.  In particular, read the list of topics and create a subscriber for each.  The 
MessageEvent syntax is used to share a common callback.  It is assumed that topic names exactly match those provided
in the .launch file, including the '/'.  If not, the topic matching will fail.*/
viconHand::viconHand(ros::NodeHandle &nh)
{
    std::vector<std::string> quadList;
    nh.getParam("hand_to_pva/quadList", quadList);
    numQuads_ = quadList.size();
    quadTopics_ = quadList;
    
    scalefactor_=0;
    ros::param::get("hand_to_pva/scalefactor",scalefactor_);
    ros::param::get("hand_to_pva/numFingers",numFingers_);

    handSub_ = nh.subscribe("/handPoseMsgs",1, &viconHand::handCallback, this,
                ros::TransportHints().unreliable()); //use UDP to avoid meltdown phenomenon
    for(int ij=0; ij<numQuads_; ij++)
    {
        hasInitPos_[ij] = false;
        quadTopics_[ij] = ("/"+quadList[ij]+"/local_odom").c_str();
        pvaPub_[ij] = nh.advertise<mg_msgs::PVA>(quadList[ij]+"/px4_control/PVA_Ref", 1);
        quadPoseSub_[ij] = nh.subscribe(quadTopics_[ij],1,&viconHand::poseCallback, this,
                ros::TransportHints().unreliable());
        ROS_INFO("%s subscriber created",quadTopics_[ij].c_str());
    }

    ROS_INFO("Hand to PVA startup complete.");

    if(scalefactor_<0.001)
    {ROS_INFO("WARNING: Scale factor is either negative or unset.");}
    if(numFingers_<1 || numFingers_>5)
    {numFingers_=1; ROS_INFO("Number of fingers on one hand is incorrect");}
}


/* Pose callback for each quad.  For simple relative positioning, this should be sufficient.*/
void viconHand::poseCallback(const ros::MessageEvent<nav_msgs::Odometry const>& event)
{
    const nav_msgs::Odometry::ConstPtr& msg = event.getMessage();

    //Get topic name 
    ros::M_string& header = event.getConnectionHeader();
    std::string publisherName = header.at("topic");

    int nk = getIndexMatchingName(publisherName.c_str(), quadTopics_, numQuads_);

    if(!hasInitPos_[nk])
    {
        ROS_INFO("Quad initialized on topic %s",publisherName.c_str());
        initPos_[nk](0) = msg->pose.pose.position.x;
        initPos_[nk](1) = msg->pose.pose.position.y;
        initPos_[nk](2) = msg->pose.pose.position.z;
        hasInitPos_[nk] = true;
    }

    //Delete init pos if it leaves?
}



/* Callback to record most recent positions for each object.  The MessageEvent syntax is used to allow a
common callback for all vicon marker topics. */
void viconHand::handCallback(const vicon_hand::handMsg::ConstPtr &msg)
{
    static bool initialized(false);
    static double twoPi(8.0*atan(1.0));
    static double onePi(4.0*atan(1.0));
    mg_msgs::PVA pva_msg;

    Eigen::Vector3d fingerCenterPose[numFingers_],  handAvg;
    double qx, qy, qz, qw, thisyaw, handOrientationAvg;
    int numNearWraparound = 0;
    Eigen::MatrixXd yawStore;
    //must dynamically resize because numFingers_ isn't known at compile time
    yawStore.resize(numFingers_,1);


    for(int ij=0; ij<numFingers_; ij++)
    {
        fingerCenterPose[ij](0) = msg->poseArray[ij].x;
        fingerCenterPose[ij](1) = msg->poseArray[ij].y;
        fingerCenterPose[ij](2) = msg->poseArray[ij].z;
        qx = msg->orientationArray[ij].x;
        qy = msg->orientationArray[ij].y;
        qz = msg->orientationArray[ij].z;
        qw = msg->orientationArray[ij].w;        

        //Average euler angles
        thisyaw = atan2(2.0*(qy*qz + qw*qx), qw*qw - qx*qx - qy*qy + qz*qz);
        yawStore(ij) = thisyaw;
	    
        if((thisyaw>onePi-0.15) || (thisyaw<onePi+0.15))
        {numNearWraparound++;}

        handAvg = handAvg + (1.0/numFingers_)*fingerCenterPose[ij];
    }


    //Add 2pi to yaw values near -pi
    if(numNearWraparound==numFingers_)
    {
        for(int ij=0;ij<numFingers_; ij++)
        {
            if(yawStore(ij)<0)
            {yawStore(ij)=yawStore(ij)+twoPi;}
        }
    }
    handOrientationAvg = 0;
    //Cannot take .mean() at compile time due to resizing
    for(int ij=0;ij<numFingers_; ij++)
    {handOrientationAvg = handOrientationAvg + (1.0/numFingers_)*yawStore(ij);}


    if(!initialized)
    {
        ROS_INFO("Hand calibrated");
        for(int ij=0;ij<3;ij++)
        {handVec0_(ij) = handAvg(ij); initialized=true;}
    }
    else
    {
        Eigen::Vector3d dv = scalefactor_*(handAvg - handVec0_);
        Eigen::Vector3d thisPos;
        for(int ij=0; ij<numQuads_; ij++)
        {
            if(hasInitPos_[ij])
            {   
                thisPos = initPos_[ij] + dv;
                pva_msg.Pos.x = thisPos(0);
                pva_msg.Pos.y = thisPos(1);
                pva_msg.Pos.z = thisPos(2);
                pva_msg.yaw = handOrientationAvg;
                pvaPub_[ij].publish(pva_msg);
            }
        }
    }
}

int viconHand::getIndexMatchingName(const std::string& stringToMatch, 
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




