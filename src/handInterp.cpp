#include "handInterp.hpp"

namespace vicon_hand
{

/* Initialize and read ros params.  In particular, read the list of topics and create a subscriber for each.  The 
MessageEvent syntax is used to share a common callback.  It is assumed that topic names exactly match those provided
in the .launch file, including the '/'.  If not, the topic matching will fail.*/
viconHand::viconHand(ros::NodeHandle &nh)
{
    scalefactor_=0;
    ros::param::get("hand_to_pva/scalefactor",scalefactor_);
    ros::param::get("hand_to_pva/numFingers",numFingers_);
    handSub_ = nh.subscribe("/handPoseMsgs",1, &viconHand::handCallback, this,
                ros::TransportHints().unreliable()); //use UDP to avoid meltdown phenomenon
    pvaPub_ = nh.advertise<mg_msgs::PVA>("px4_control/PVA", 1);
    ROS_INFO("Hand to PVA startup complete.");

    if(scalefactor_<0.001)
    {ROS_INFO("WARNING: Scale factor is either negative or unset.");}
    if(numFingers_<1 || numFingers_>5)
    {numFingers_=1; ROS_INFO("Number of fingers on one hand is incorrect");}
}


/* Callback to record most recent positions for each object.  The MessageEvent syntax is used to allow a
common callback for all vicon marker topics. */
void viconHand::handCallback(const vicon_hand::handMsg::ConstPtr &msg)
{
    static bool initialized(false);
    mg_msgs::PVA pva_msg;

    Eigen::Vector3d fingerCenterPose[numFingers_];
    Eigen::Quaterniond fingerPointerQuaternion[numFingers_];
    Eigen::Vector3d handAvg;
    Eigen::Vector3d handOrientationAvg;

    for(int ij=0; ij<numFingers_; ij++)
    {
        fingerCenterPose[ij](0) = msg->poseArray[ij].x;
        fingerCenterPose[ij](1) = msg->poseArray[ij].y;
        fingerCenterPose[ij](2) = msg->poseArray[ij].z;
        fingerPointerQuaternion[ij].x() = msg->orientationArray[ij].x;
        fingerPointerQuaternion[ij].y() = msg->orientationArray[ij].y;
        fingerPointerQuaternion[ij].z() = msg->orientationArray[ij].z;
        fingerPointerQuaternion[ij].w() = msg->orientationArray[ij].w;

        //Average euler angles
        handOrientationAvg = handOrientationAvg + (1.0/numFingers_)*(fingerPointerQuaternion[ij].toRotationMatrix()).eulerAngles(0, 1, 2);

        handAvg = handAvg + (1.0/numFingers_)*fingerCenterPose[ij];
    }


    if(!initialized)
    {
        for(int ij=0;ij<3;ij++)
        {handVec0_(ij) = handAvg(ij); initialized=true;}
    }
    else
    {
        Eigen::Vector3d dv = scalefactor_*(handAvg - handVec0_);
        pva_msg.Pos.x = dv(0);
        pva_msg.Pos.y = dv(1);
        pva_msg.Pos.z = dv(2);
        pva_msg.yaw = handOrientationAvg(2);
        pvaPub_.publish(pva_msg);
    }
}

} //ns




