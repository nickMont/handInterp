#include <unistd.h> 
#include <csignal>
#include <stdio.h> 
#include <sys/socket.h> 
#include <stdlib.h> 
#include <netinet/in.h> 
#include <string.h> 
#include <boost/program_options.hpp>
#include <ros/ros.h>
#include <vicon_hand/handMsg.h>
#include <Eigen/Geometry>

#define PORT 5055

const char strSIGTERM[] = "SIGTERM";
const char strSIGINT[] = "SIGINT";
const char strSIGHUP[] = "SIGHUP";
const char *ptrSigString = 0;
const int MAXLINE = 4096; //maximum line length
static volatile __sig_atomic_t sigterm_caught = 0;

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


int main(int argc, char *argv[]) 
{ 
	std::signal(SIGTERM, signalHandler);
  	std::signal(SIGINT, signalHandler);
 	std::signal(SIGHUP, signalHandler);
 	std::signal(SIGQUIT, signalHandler);

	int server_fd, new_socket, valread; 
	struct sockaddr_in address, clientaddress; 
	int opt = 1; 
	int addrlen = sizeof(address); 
	float buffer[1024] = {0}; 
	int n, connfd;
	socklen_t clilen;

	//Create ros node
	ros::init(argc, argv, "rosvrhand endpoint");
	ros::NodeHandle nh;
	ros::Publisher hand_pub = nh.advertise<vicon_hand::handMsg>("handPose",1);
	
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
	/*if ((new_socket = accept(server_fd, (struct sockaddr *)&address, 
					(socklen_t*)&addrlen))<0) 
	{ 
		perror("accept"); 
		exit(EXIT_FAILURE); 
	}*/
	//SPINNN HEEEERE

	/*while(!sigterm_caught)
	{
	    ::pause();
		send(new_socket, hello, strlen(hello), 0); 
		printf("Hello message sent\n");
		valread = read( new_socket , buffer, 1024); 
		printf("%s\n",buffer);
	//	usleep(1000);  //usleep takes mu-s
	}*/
	for ( ; ; )
	{
 		clilen = sizeof(clientaddress);
  		connfd = accept (server_fd, (struct sockaddr *) &clientaddress, &clilen);
  		printf("%s\n","Received request...");

  		while ( (n = recv(connfd, buffer, MAXLINE,0)) > 0) 
  		{

  			//Convert hand[] from buffer into ros message
  			vicon_hand::handMsg derp;
  			int nL=(sizeof(buffer)/sizeof(*buffer));
  			//convention: n=7*f, where f is the number of fingers.
  			//elements 0:2 of each of the f fingers are pose, 3:6 are quaternion as (xyzw)		
  			int nf=nL/7;
  			derp.poseArray.resize(nf);
  			derp.orientationArray.resize(nf);
  			derp.tLast.resize(nf);
  			float tcurr=(ros::Time::now()).toSec();
  			for(int ij=0; ij++; ij<nf)
  			{
  				derp.tLast[ij]=tcurr;
  				derp.poseArray[ij].x=buffer[0+ij*7];
  				derp.poseArray[ij].y=buffer[1+ij*7];
  				derp.poseArray[ij].z=buffer[2+ij*7];
  				derp.orientationArray[ij].x=buffer[3+ij*7];
  				derp.orientationArray[ij].y=buffer[4+ij*7];
  				derp.orientationArray[ij].z=buffer[5+ij*7];
  				derp.orientationArray[ij].w=buffer[6+ij*7];
  			}
  			hand_pub.publish(derp);
   		}
   		if (n < 0) {
  			perror("Read error");
  			exit(1);
 		}
 		close(connfd);
   	}
	

	return 0; 
} 
