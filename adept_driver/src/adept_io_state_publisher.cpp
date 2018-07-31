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
#include "signal.h"

#include "ros/ros.h"
#include "adept_msgs/AdeptIO.h"
#include "adept_msgs/IOStates.h"
#include "adept_msgs/IO.h"

using namespace std;

//Sigint handler for shutting down this node with strg+c
void shutdown(int sig)
{
  ros::shutdown();
}

int main(int argc, char **argv) {

	//Initiate node "io_publisher_node"
	ros::init(argc, argv, "io_publisher_node", ros::init_options::NoSigintHandler);
	//Access point to the ROS system
	ros::NodeHandle n;

	//Get the Node name for status messages
    std::string nodeName = ros::this_node::getName();

	//Parameter
	std::vector<int> ioNames;
	int rate;

	//If a their is a rate in the adept_io_publisher namespace it will be saved in the rate variable
	//otherwise the standard value 10
	ros::param::param<int>("/adept_io_publisher/rate", rate, 10);

	//If their is a Parameter io_names in the adept_io_publisher namespace
	//output info
	if (n.getParam("/adept_io_publisher/io_names", ioNames)){
		ROS_INFO_STREAM(nodeName<<": Loaded parameters.");
	}
	//Else output error and shutdown node
	else {
		ROS_ERROR_STREAM(nodeName<<": Did not find ioNames parameter, exiting");
		ros::shutdown();
	}

	//Publisher loop rate
	ros::Rate loop_rate(rate);
	ros::Rate at_start(2);

	//Wait 5s for the Service io_service
	if (!ros::service::waitForService("io_service", ros::Duration(5.0f))){
		//If their is no service output error and shutdown node
		ROS_ERROR_STREAM(nodeName<<": io_service not found, exiting");
		ros::shutdown();
	}
	at_start.sleep();

	//Start io_service Client and declare message io_service
	ros::ServiceClient client = n.serviceClient<adept_msgs::AdeptIO>("io_service");
	adept_msgs::AdeptIO io_service;
	ROS_INFO_STREAM(nodeName<<": io_service found.");

	//Start Publisher io_state_publisher and declare message io and ioStates
	ros::Publisher io_state_publisher = n.advertise<adept_msgs::IOStates>("io_state_publisher", 1000);
	adept_msgs::IO io;
	adept_msgs::IOStates ioStates;

	ROS_INFO_STREAM(nodeName<<": Ready to publish IO states.");

	//While ROS is ok
	while (ros::ok()) {
		//loop start
		int i = 1;
		//clear states in ioStates Variable
		ioStates.states.clear();
		//While i <= ioNames size
		while (i <= ioNames.size())
		{
			//Set io_service request mode on Read
			//Set io_service request io_number on the current io_number from the ioNames Parameter
			io_service.request.mode = io_service.request.READ;
			io_service.request.io_number = ioNames[i-1];

			//Send Service request
			//If successful set publisher message
			if (client.call(io_service))
			{

				io.pin = ioNames[i-1];
				io.state = (bool)io_service.response.result;
				ioStates.states.push_back(io);
			}
			//Else output error and return 1
			else
			{
				ROS_ERROR_STREAM(nodeName<<": Failed to call service io_service");
			}
			//increase i to publish the next IO from the ioNames Parameter
			i++;

		}

		//Publish ioStates
		io_state_publisher.publish(ioStates);

	    ros::spinOnce();

	    //Start a sigint handler for shutting down this node with strg+c
		signal(SIGINT, shutdown);

	}

	return 0;
}

