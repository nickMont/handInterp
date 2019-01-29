#include <Eigen/Geometry>
#include "commander.hpp"
#include "poseContainer.hpp"
#include "quadContainer.hpp"
#include "handEndpoint.hpp"
#include <string>
#include <iostream>

const char strSIGTERM[] = "SIGTERM";
const char strSIGINT[] = "SIGINT";
const char strSIGHUP[] = "SIGHUP";
const char *ptrSigString = nullptr;
static volatile sig_atomic_t sigterm_caught = 0;
extern "C" void signalHandler(int signum) {
    if(!sigterm_caught) {
        if(signum == SIGTERM || signum == SIGINT || signum == SIGHUP) {
            if(!ptrSigString) {
                if(signum == SIGTERM) ptrSigString = strSIGTERM;
                if(signum == SIGINT) ptrSigString = strSIGINT;
                if(signum == SIGHUP) ptrSigString = strSIGHUP;
            }
        sigterm_caught = 1;
        }
    }
}


int main(int argc, char **argv)
{
    std::signal(SIGTERM, signalHandler);
    std::signal(SIGINT, signalHandler);
    std::signal(SIGHUP, signalHandler);
    std::signal(SIGQUIT, signalHandler);
    ros::init(argc, argv, "gestureGeneral");

    ros::NodeHandle nh;

    //Contains all pose subscribers
    auto poseSubs = std::make_shared<handIn::poseContainer>();
    //Contains endpoint to VR driver
    auto handSub = std::make_shared<handIn::handEndpoint>();
    //Converts hand+pose->gesture
    auto commandGenerator = std::make_shared<handIn::commander>();
    
    int sourceForHandData, sourceForGestureData, numQuads;
    std::string topic, gestureTopic, hTopic, gTopic;
    hTopic = "5555";
    gTopic = "5555";
    ros::param::get(ros::this_node::getName()+"/sourceForHandData",sourceForHandData);
    ros::param::get(ros::this_node::getName()+"/topicOrPortNumber",hTopic);
    ros::param::get(ros::this_node::getName()+"/useGestureNN",sourceForGestureData);
    ros::param::get(ros::this_node::getName()+"/topicOrPortNumberGesture",gTopic);
    int useRosInsteadOfPort, useRosForGestureSource;
    ros::param::get(ros::this_node::getName()+"/useRosInsteadOfPort",useRosInsteadOfPort);
    ros::param::get(ros::this_node::getName()+"/useRosForGestureSource",useRosForGestureSource);
    handSub->configure(useRosInsteadOfPort, hTopic, useRosForGestureSource, gTopic);
    
    //read in numquads
    numQuads=10;

    poseSubs->configure(numQuads);
    handSub->configure(1,hTopic,1,gTopic);
    commandGenerator->setnodehandle(nh);

    //NOTE: THIS CODE IS WORDY BUT STD LIBRARY IS NOT CAPABLE OF VECTORIZING THIS IN OLDER VERSIONS
    //      OF C++. WILL REWRITE IF WE UPGRADE TO A NEWER C++.
    //Each quad is its own object, with its own position that is updated by poseSubs
    std::string quadname;
    std::string quadList[10];
    int ij=1;
    auto quad1 = std::make_shared<handIn::quadContainer>(nh);
    ros::param::get(ros::this_node::getName()+"/quad"+std::to_string(ij),quadname);
    quad1->configure(ij); quad1->setName(ij, quadname);
    poseSubs->createPoseSub(ij,quadname); quadList[ij] = quadname;
    ij=2;
    auto quad2 = std::make_shared<handIn::quadContainer>(nh);
    ros::param::get(ros::this_node::getName()+"/quad"+std::to_string(ij),quadname);
    quad2->configure(ij); quad2->setName(ij, quadname);
    poseSubs->createPoseSub(ij,quadname); quadList[ij] = quadname;
    ij=3;
    auto quad3 = std::make_shared<handIn::quadContainer>(nh);
    ros::param::get(ros::this_node::getName()+"/quad"+std::to_string(ij),quadname);
    quad3->configure(ij); quad3->setName(ij, quadname);
    poseSubs->createPoseSub(ij,quadname); quadList[ij] = quadname;
    ij=4;
    auto quad4 = std::make_shared<handIn::quadContainer>(nh);
    ros::param::get(ros::this_node::getName()+"/quad"+std::to_string(ij),quadname);
    quad4->configure(ij); quad4->setName(ij, quadname);
    poseSubs->createPoseSub(ij,quadname); quadList[ij] = quadname;
    ij=5;
    auto quad5 = std::make_shared<handIn::quadContainer>(nh);
    ros::param::get(ros::this_node::getName()+"/quad"+std::to_string(ij),quadname);
    quad5->configure(ij); quad5->setName(ij, quadname);
    poseSubs->createPoseSub(ij,quadname); quadList[ij] = quadname;
    ij=6;
    auto quad6 = std::make_shared<handIn::quadContainer>(nh);
    ros::param::get(ros::this_node::getName()+"/quad"+std::to_string(ij),quadname);
    quad6->configure(ij); quad6->setName(ij, quadname);
    poseSubs->createPoseSub(ij,quadname); quadList[ij] = quadname;
    ij=7;
    auto quad7 = std::make_shared<handIn::quadContainer>(nh);
    ros::param::get(ros::this_node::getName()+"/quad"+std::to_string(ij),quadname);
    quad7->configure(ij); quad7->setName(ij, quadname);
    poseSubs->createPoseSub(ij,quadname); quadList[ij] = quadname;
    ij=8;
    auto quad8 = std::make_shared<handIn::quadContainer>(nh);
    ros::param::get(ros::this_node::getName()+"/quad"+std::to_string(ij),quadname);
    quad8->configure(ij); quad8->setName(ij, quadname);
    poseSubs->createPoseSub(ij,quadname); quadList[ij] = quadname;
    ij=9;
    auto quad9 = std::make_shared<handIn::quadContainer>(nh);
    ros::param::get(ros::this_node::getName()+"/quad"+std::to_string(ij),quadname);
    quad9->configure(ij); quad9->setName(ij, quadname);
    poseSubs->createPoseSub(ij,quadname); quadList[ij] = quadname;
    ij=10;
    auto quad10 = std::make_shared<handIn::quadContainer>(nh);
    ros::param::get(ros::this_node::getName()+"/quad"+std::to_string(ij),quadname);
    quad10->configure(ij); quad10->setName(ij, quadname);
    poseSubs->createPoseSub(ij,quadname); quadList[ij] = quadname;

    commandGenerator->configure(numQuads, quadList);

    ros::spin();

    return 0;
}
