#include "handEndpoint.hpp"

namespace handIn
{

handEndpoint::handEndpoint(ros::NodeHandle &nh)
{
	nh_ = nh;
	hasHandCenter_[0] = false;
	hasHandCenter_[1] = false;
	hasCommander_=false;
}


void handEndpoint::configure(const int useROS, const std::string topicOrPortName, const int getGestureFromNN, const std::string gestureTopic)
{
	if(useROS==0)
	{
		handSub_ = nh_.subscribe(topicOrPortName,1,&handEndpoint::handCallback,
		  this, ros::TransportHints().unreliable().reliable().tcpNoDelay(true));
	}
	else if(useROS==1)
	{
		createPipeAndSpin(std::stoi(topicOrPortName));
	}

	//Gesture data comes from:
	// 0: ROS hand data
	// 1: Via buffer
	// 2: Piped externally to NN
	if(getGestureFromNN==0)
	{
		gsSub_ = nh.subscribe(gestureTopic,1,&handEndpoint::handCallback,
		  this, ros::TransportHints().unreliable().reliable().tcpNoDelay(true));
		gestureInput_ = 0;
	}
	else if(getGestureFromNN==1)
	{
		gestureInput_ = 1;
	}else if(getGestureFromNN==2) //NOTE: Timing is not implemented, waiting to test with new gloves
	{
		endpointToNN_ = nh_.advertise<hand_endpoint::hand>("/hands", 1;
		endpointFromNN_ = nh_.subscribe("/gestureTopic",1,&handEndpoint::gestureCallback,
			this, ros::TransportHints().unreliable.reliable().tcpNoDelay(true));
	}

	handCenterSubRight_ = nh_.subscribe("/rightHand/local_odom",1,&handEndpoint::handCenterCallback,
		this, ros::TransportHints().unreliable.reliable().tcpNoDelay(true));
	handCenterSubLeft_ = nh_.subscribe("/leftHand/local_odom",1,&handEndpoint::handCenterCallback,
		this, ros::TransportHints().unreliable.reliable().tcpNoDelay(true));)
	return;
}


void handEndpoint::setCommanderPtr(std::shared_ptr<handIn::commander> commptr)
{
	commander_ = commptr;
	hasCommander_=true;
	return;
}


void handEndpoint::handAction(const float &datmat[])
{
	lastTProc_ = getRosTime();
	if(hasCommander_)
	{
		commander_->sendHand(datmat);
	}
}


//double check this return <<<
Eigen14d handEndpoint::getHandCenters()
{
	Eigen14d datArr;
	datArr( 0) = rightHand(0);
	datArr( 1) = rightHand(1);
	datArr( 2) = rightHand(2);
	datArr( 3) = rightHandQ.x();
	datArr( 4) = rightHandQ.y();
	datArr( 5) = rightHandQ.z();
	datArr( 6) = rightHandQ.w();
	datArr( 7) = leftHand(0);
	datArr( 8) = leftHand(1);
	datArr( 9) = leftHand(2);
	datArr(10) = leftHandQ.x();
	datArr(11) = leftHandQ.y();
	datArr(12) = leftHandQ.z();
	datArr(13) = leftHandQ.w();
	return datArr;
}


double handEndpoint::getRosTime()
{
	return (ros::Time::now()).toSec();
}


void handEndpoint::gestureCallback(const hand_endpoint::gesture::ConstPtr &msg)
{
	gestTime_ = (msg->header.stamp).toSec();
	thisGestR_ = msg->gestureRight;
	thisGestL_ = msg->gestureLeft;
}


void handEndpoint::createPipeAndSpin(const int PORT)
{
	int server_fd, new_socket, valread; 
	struct sockaddr_in address, clientaddress; 
	int opt = 1; 
	int addrlen = sizeof(address); 
	float buffer[1024] = {0}; 
	int n, connfd;
	socklen_t clilen;

	// Creating socket file descriptor 
	if ((server_fd = socket(AF_INET, SOCK_STREAM, 0)) == 0) 
	{ 
		perror("socket failed"); 
		exit(EXIT_FAILURE); 
	} 
	
	// Forcefully attaching socket to the port PORT
	if (setsockopt(server_fd, SOL_SOCKET, SO_REUSEADDR | SO_REUSEPORT, 
												&opt, sizeof(opt))) 
	{ 
		perror("setsockopt"); 
		exit(EXIT_FAILURE); 
	} 
	address.sin_family = AF_INET; 
	address.sin_addr.s_addr = INADDR_ANY; 
	address.sin_port = htons( PORT ); 
	
	// Forcefully attaching socket to the port PORT
	if (bind(server_fd, (struct sockaddr *)&address, 
								sizeof(address))<0) 
	{ 
		perror("bind failed"); 
		exit(EXIT_FAILURE); 
	} 
	if (listen(server_fd, 3) < 0) 
	{ 
		perror("listen"); 
		exit(EXIT_FAILURE); 
	} 


	for ( ; ; )
	{
 		clilen = sizeof(clientaddress);
  		connfd = accept (server_fd, (struct sockaddr *) &clientaddress, &clilen);
  		printf("%s\n","Received request...");

  		float handFull[55];
  		Eigen14d handCen;

  		while ( (n = recv(connfd, buffer, MAXLINE,0)) > 0) 
  		{
  			if(hasHandCenter_[0] && hasHandCenter_[1])
			{
			//process buffer
  			for(int ij=0; ij<42; ij++)
			{handFull[ij]=buffer[ij];}

			//augment with hande odometry
  			handCen = getHandCenters;
  			for(int ij=0; ij<13; ij++)
			{handFull[ij+43]=handCen[ij];}

			if(gestureInput_==1)
			{
				thisGestR_ = buffer[42];
				thisGestL_ = buffer[43];
			}else if(gestureInput_==2)
			{
				//create and publish hand message; not implemented, pending testing
			}

			handFull[53] = thisGestR_;
			handFull[54] = thisGestL_;
  			handAction(handFull);
  			}
   		}
   		if (n < 0) {
  			perror("Read error");
  			exit(1);
 		}
 		close(connfd);
   	}
   	return;
}


void handEndpoint::handCallback(const hand_endpoint::hands::ConstPtr &msg)
{
	if(hasHandCenter_[0] && hasHandCenter_[1])
	{
	float Eigen::Vector55d handFull;
	//see endpoint.cs for matching convention, drawn from Unity
	handFull( 0) = (msg->header.stamp).toSec();
	handFull( 1) = 1;
	//quaternion describing each finger
	handFull( 2) = msg->right.thumb[2].x;
	handFull( 3) = msg->right.thumb[2].y;
	handFull( 4) = msg->right.thumb[2].z;
	handFull( 5) = msg->right.thumb[2].w;
	handFull( 6) = msg->right.point[2].x;
	handFull( 7) = msg->right.point[2].y;
	handFull( 8) = msg->right.point[2].z;
	handFull( 9) = msg->right.point[2].w;
	handFull(10) = msg->right.middle[2].x;
	handFull(11) = msg->right.middle[2].y;
	handFull(12) = msg->right.middle[2].z;
	handFull(13) = msg->right.middle[2].w;
	handFull(14) = msg->right.index[2].x;
	handFull(15) = msg->right.index[2].y;
	handFull(16) = msg->right.index[2].z;
	handFull(17) = msg->right.index[2].w;
	handFull(18) = msg->right.pinky[2].x;
	handFull(19) = msg->right.pinky[2].y;
	handFull(20) = msg->right.pinky[2].z;
	handFull(21) = msg->right.pinky[2].w;

	//left hand
	handFull(22) = 0;
	handFull(23) = msg->left.thumb[2].x;
	handFull(24) = msg->left.thumb[2].y;
	handFull(25) = msg->left.thumb[2].z;
	handFull(26) = msg->left.thumb[2].w;
	handFull(27) = msg->left.point[2].x;
	handFull(28) = msg->left.point[2].y;
	handFull(29) = msg->left.point[2].z;
	handFull(30) = msg->left.point[2].w;
	handFull(31) = msg->left.middle[2].x;
	handFull(32) = msg->left.middle[2].y;
	handFull(33) = msg->left.middle[2].z;
	handFull(34) = msg->left.middle[2].w;
	handFull(35) = msg->left.index[2].x;
	handFull(36) = msg->left.index[2].y;
	handFull(37) = msg->left.index[2].z;
	handFull(38) = msg->left.index[2].w;
	handFull(39) = msg->left.pinky[2].x;
	handFull(40) = msg->left.pinky[2].y;
	handFull(41) = msg->left.pinky[2].z;
	handFull(42) = msg->left.pinky[2].w;

	if(!gestureInput_)
	{
		thisGestR_ = msg->right.gesture;
		thisGestL_ = msg->left.gesture;
	}


	//augment with hand centers
	Eigen14d handCen = getHandCenters();
	for(int ij=0; ij<14; ij++)
	{
		handFull(ij+43) = handCen(ij);
	}
	handFull(53) = thisGestR_;
	handFull(54) = thisGestL_;

	handAction(handFull);
	} //end IF_INIT
	return;
}


} //ns
