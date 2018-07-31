/*
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2018, Benjamin Schellenbeck
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *      * Redistributions of source code must retain the above copyright
 *      notice, this list of conditions and the following disclaimer.
 *      * Redistributions in binary form must reproduce the above copyright
 *      notice, this list of conditions and the following disclaimer in the
 *      documentation and/or other materials provided with the distribution.
 *      * Neither the name of the copyright holder nor the names 
 *      of its contributors may be used to endorse or promote products derived 
 *      from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <iostream>
#include <string.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <stdlib.h>
#include <unistd.h>
#include <netdb.h>
#include <vector>
#include "signal.h"

#include "ros/ros.h"
#include "adept_msgs/AdeptIO.h"

using namespace std;

//Sigint handler for shutting down this node with strg+c
void shutdown(int sig){

    ros::shutdown();
}

class IOService {
public:
	~IOService()
	{
		//Close the client if the object is destroyed
		close(client);
	}
	bool init(){
		ros::NodeHandle nh("");
		std::string nodeName = ros::this_node::getName();

		//If load_parameters is successful write it to the ros console
		if(load_parameters() )
		{
			ROS_INFO_STREAM(nodeName<<": Loaded parameters.");
		}
		//Else write an error to the ros console and return false
		else
		{
			ROS_ERROR_STREAM(nodeName<<": Did not find required ros parameters, exiting");
			return false;
		}

		// port number
		int portNum = 11001;

		struct sockaddr_in server_addr;

		ROS_INFO_STREAM(nodeName<<": Starting client ...");

		//Get Socket
		client = socket(AF_INET, SOCK_STREAM, 0);

		//If this fails return false
		if (client < 0) {
			ROS_ERROR_STREAM(nodeName<<": Error establishing socket...");
			return false;
		}

		ROS_INFO_STREAM(nodeName<<": Socket client has been created...");

		server_addr.sin_family = AF_INET;
		server_addr.sin_port = htons(portNum);

		inet_pton(AF_INET, ip.c_str(), &server_addr.sin_addr);

		//Connect to the Server
		//If the connection failed return false
		if (connect(client, (struct sockaddr *) &server_addr, sizeof(server_addr)) < 0){
			ROS_ERROR_STREAM(nodeName<<": Connection to the server port number: "<< portNum <<" failed");
			return false;
		}
		ROS_INFO_STREAM(nodeName<<": Connection to the server port number: "<< portNum);

		return true;
	}

	//Service request and response
	bool sendMessageOverSocket(adept_msgs::AdeptIO::Request  &req, adept_msgs::AdeptIO::Response &res){

		ros::Rate time_between_msg(socket_rate);
		vector<unsigned char> arrayOfByte(4);
		int bufsize = 1024;
		char buffer[bufsize];
		int zahl;

		//Convert int mode to big-endian bytes
		arrayOfByte = intToBytesBE(req.mode);
		//Send mode over the Socket to the adept controller

		send(client, &arrayOfByte[0], arrayOfByte.size(), 0);

		time_between_msg.sleep();

		//Convert int io_number to big-endian bytes
		arrayOfByte = intToBytesBE(req.io_number);
		//Send io_number over the Socket to the adept controller
		send(client, &arrayOfByte[0], arrayOfByte.size(), 0);

		time_between_msg.sleep();

		//Get response message from Socket Server
		recv(client, buffer, bufsize, 0);
		std::stringstream sstr(buffer);
		sstr >> zahl;
		//Write response message into the Service result
		res.result = (bool)zahl;

		//Service request successfully processed
		return true;
	}

private:
	//Declare client and Server-IP variables
	int client;

	std::string ip;

	int socket_rate;

	//Load parameter from the ROS-Parameter-Server
	bool load_parameters(){
		ros::NodeHandle nh;

		//Necessary parameters
		bool success = true;
		success =  nh.getParam("robot_ip_address", ip);
		success = success && nh.getParam("/adept_io/socket_rate", socket_rate);

		return success;
	}

	//Calculate an int to a 4 Byte big-endian vector of chars
	vector<unsigned char> intToBytesBE(int value){
		vector<unsigned char> arrayOfByte(4);

		for (int i = 0; i < 4; i++)
			arrayOfByte[3 - i] = (value >> (i * 8));
		return arrayOfByte;
	}

};

int main(int argc, char **argv) {

	//Initiate node "io_service_node"
	ros::init(argc, argv, "io_service_node", ros::init_options::NoSigintHandler);
	//Access point to the ROS system
	ros::NodeHandle nh;

	//Get the Node name for status messages
    std::string nodeName = ros::this_node::getName();

    //Instantiate ioService from class IOService
	IOService ioService;

	//If ioService initialization successful
	if (ioService.init()){
		//Start service io_service
		ros::ServiceServer service = nh.advertiseService("io_service", &IOService::sendMessageOverSocket, &ioService);

		ROS_INFO_STREAM(nodeName<<": Ready for service request");

		//Start a sigint handler for shutting down this node with strg+c
		signal(SIGINT, shutdown);

		ros::spin();
	}
	ROS_INFO_STREAM(nodeName<<": Initialization failed, exiting");

	return 0;
}
