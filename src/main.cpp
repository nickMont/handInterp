#include <Eigen/Geometry>
#include "estimationNode.hpp"
#include "gbxStreamEndpointGPSKF.hpp"
#include "rosStreamEndpointGPSKF.hpp"
#include "rosPlayback.hpp"
#include "viconStream.hpp"
#include <string>
#include <iostream>
#include "mathHelperFunctions.hpp"

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

    //Each quad is its own object, with its own position that is updated by poseSubs
    auto quads[10] = std::make_shared<handIn::quadContainer>()[10];
    //Contains all pose subscribers
    auto poseSubs = std::make_shared<handIn::poseContainer>();
    //Contains endpoint to VR driver
    auto handSub = std::make_shared<handIn::handEndpoint>();
    //Converts hand+pose->gesture
    auto commandGenerator = std::make_shared<handIn::commander>();

    poseSubs->configure();

    //read in numquads
    numQuads=10;
    std::string quadname;
    for(int ij=0; ij<numQuads; ij++)
    {
        ros::param::get(ros::this_node::getName()+"/quad"+str(ij),quadname);
        quads[ij]->configure(ij);
        quads[ij]->setName(ij, quadname);
        poseSubs->createNewSubscriber(ij,quadname);
    }

    handSub->configure();
    commandGenerator->configure();

    ros::spin();

    return 0;
}
