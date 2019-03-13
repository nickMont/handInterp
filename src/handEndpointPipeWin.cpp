#include "handEndpoint.hpp"
#include <sys/socket.h> 
#include <stdlib.h> 
#include <netinet/in.h> 

namespace handIn
{

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

  		Vector55d handFull;
  		Vector14d handCen;

  		//arbitrarily set max buffer length at 5000
  		while ( (n = recv(connfd, buffer, 5000 ,0)) > 0) 
  		{
  			if(isGreen_)
			{
			//process buffer
  			for(int ij=0; ij<42; ij++)
			{handFull(ij)=buffer[ij];}

			//augment with hande odometry
  			handCen = getHandCenters();
  			for(int ij=0; ij<13; ij++)
			{handFull(ij+43)=handCen[ij];}

			if(gestureInput_==1)
			{
				thisGestR_ = buffer[42];
				thisGestL_ = buffer[43];
			}else if(gestureInput_==2)
			{
				//create and publish hand message; not implemented, pending testing
			}

			handFull(53) = thisGestR_;
			handFull(54) = thisGestL_;
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


}//ns