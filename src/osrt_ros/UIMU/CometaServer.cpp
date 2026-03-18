// Server side implementation of UDP client-server model

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <netinet/in.h>
#include "osrt_ros/UIMU/CometaServer.h"

#include <iostream>
#include <sstream>
#include <iterator>
#include <vector>

#include <ros/ros.h>
#include <tf/transform_listener.h>

// Driver code
CometaServer::CometaServer (int PORT, int MAXLINE) {

		ROS_INFO("Cometa server started");
		//socklen_t sockfd;
		buffer = new char[MAXLINE];
		hello = "Hello from CometaServer";
		buffersize = MAXLINE;
		//struct sockaddr_in servaddr, cliaddr;
		
		// Creating socket file descriptor
		if ( (sockfd = socket(AF_INET, SOCK_DGRAM, 0)) < 0 ) {
			perror("socket creation failed");
			exit(EXIT_FAILURE);
		}
		
		memset(&servaddr, 0, sizeof(servaddr));
		memset(&cliaddr, 0, sizeof(cliaddr));
		
		// Filling server information
		servaddr.sin_family = AF_INET; // IPv4
		servaddr.sin_addr.s_addr = INADDR_ANY;
		servaddr.sin_port = htons(PORT);
		
		// Bind the socket with the server address
		if ( bind(sockfd, (const struct sockaddr *)&servaddr,
				sizeof(servaddr)) < 0 )
		{
			perror("bind failed");
			exit(EXIT_FAILURE);
		}
		
		//socklen_t len, n;

		len = sizeof(cliaddr); //len is value/resuslt
}

CometaServer::~CometaServer(void)
{
}

//std::vector<double> CometaServer::receive()
bool CometaServer::receive()
{
		ROS_DEBUG_STREAM("READING CometaServer");
		n = recvfrom(sockfd, (char *)buffer, buffersize,
					MSG_WAITALL, ( struct sockaddr *) &cliaddr,
					&len);
		//now I need to find if there is the word BYE in it
		buffer[n] = '\0';
		//rosdebug?
		//printf("Client : %s\n", buffer);
		if (strcmp(buffer, "BYE!") == 0 )
		{
		       std::cout << "received goodbye SS OK." << std::endl;
		       return false;
		}

		std::stringstream ss;
		ss << buffer;
		std::istream_iterator<std::string> begin(ss), end;
		std::vector<std::string> vstrings(begin, end);
		//std::copy(vstrings.begin(), vstrings.end(), std::ostream_iterator<std::string>(std::cout, "\n"));
		std::vector<double> myvec;

		for (auto a_str : vstrings)
		{
			myvec.push_back(std::stod(a_str));
		}
		////THIS IS KINDA WORKING. I THINK
		//for (auto a_double : myvec )
		//	std::cout << a_double << "|";
		//std::cout << std::endl ;

		//printf("Client : %s\n", buffer);
		sendto(sockfd, (const char *)hello, strlen(hello),
			MSG_CONFIRM, (const struct sockaddr *) &cliaddr,
				len);
		//todo: rosdebug or somehting
		//printf("Hello message sent.\n");
		
		output = myvec;
		//return myvec;
		//for( auto i:myvec)
		//	ROS_INFO_STREAM("THIS THING" << i);
		//ROS_INFO_STREAM("THIS THING:" << myvec.size());
		return true;
}

