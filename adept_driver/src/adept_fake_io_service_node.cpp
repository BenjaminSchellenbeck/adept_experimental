/*
 *   Copyright (c) 2018 Benjamin Schellenbeck.  All rights reserved.
 *
 *   This program is free software; you can redistribute it and/or modify
 *   it under the terms of the GNU General Public License as published by
 *   the Free Software Foundation; either version 2 of the License, or
 *   (at your option) any later version.5
 *   
 *   This program is distributed in the hope that it will be useful,
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *   GNU General Public License for more details at:
 *
 *   http://www.gnu.org/licenses/gpl.html
 */

#include <iostream>
#include <string.h>
#include "signal.h"

#include "ros/ros.h"
#include "adept_msgs/AdeptIO.h"

using namespace std;
using std::map;

//Sigint handler for shutting down this node with strg+c
void shutdown(int sig){

    ros::shutdown();
}

class IOService {
public:

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

		//Dictionarys <int, bool> to simulate a gripper
		//Standard initialization is a open gripper
		grasp_output_nr_state[grasp_output_nr] = false;
		release_output_nr_state[release_output_nr] = true;
		grasp_input_nr_state[grasp_input_nr] = false;
		release_input_nr_state[release_input_nr] = true;

		return true;
	}

	//Service request and response
	//All requests are true except IOs for Gripper Simulation
	bool sendMessageOverSocket(adept_msgs::AdeptIO::Request  &req, adept_msgs::AdeptIO::Response &res){

		//If request mode is read
		if (req.mode == adept_msgs::AdeptIO::Request::READ){
			//If request io_number is the io_number to grasp
			if (!(grasp_input_nr_state.find(req.io_number)==grasp_input_nr_state.end())){
				//then result is current grasp input number state
				res.result = grasp_input_nr_state[grasp_input_nr];
			}
			//Else if request io_number is the io_number to release
			else if (!(release_input_nr_state.find(req.io_number)==release_input_nr_state.end())){
				//then result is current release input number state
				res.result = release_input_nr_state[release_input_nr];
			}
			//Else its another IO and the result must be true
			else {
				res.result = true;
			}
		}

		//If request mode is write
		if (req.mode == adept_msgs::AdeptIO::Request::WRITE){
			//If the io_number is the grasp output number
			if (abs(req.io_number) == grasp_output_nr){
				//and the io_number is smaller 0
				if (req.io_number < 0) {
					//then grasp output number state is false
					grasp_output_nr_state[grasp_output_nr] = false;
				}
				else {
					//Else its true
					grasp_output_nr_state[grasp_output_nr] = true;
				}
			}
			//If the io_number is the release output number
			if (abs(req.io_number) == release_output_nr){
				//and the io_number is smaller 0
				if (req.io_number < 0) {
					//then grasp output number state is false
					release_output_nr_state[release_output_nr] = false;
				}
				else {
					//Else its true
					release_output_nr_state[release_output_nr] = true;
				}
			}

			//If the grasp output state is true and the release output state is false
			if (grasp_output_nr_state[grasp_output_nr] == true and release_output_nr_state[release_output_nr] == false){
				//then the grasp input state must be true
				//and the release input state must be false
				grasp_input_nr_state[grasp_input_nr] = true;
				release_input_nr_state[release_input_nr] = false;
			}
			if (grasp_output_nr_state[grasp_output_nr] == false and release_output_nr_state[release_output_nr] == true){
				//then the grasp input state must be false
				//and the release input state must be true
				grasp_input_nr_state[grasp_input_nr] = false;
				release_input_nr_state[release_input_nr] = true;
			}
			res.result = true;
		}

		return true;
	}

private:

	//Declare Parameter variables
	int grasp_output_nr;
	int release_output_nr;
	int grasp_input_nr;
	int release_input_nr;

	//Declare dictionary to simulate Gripper states
	std::map<int, bool> grasp_input_nr_state;
	std::map<int, bool> release_input_nr_state;
	std::map<int, bool> grasp_output_nr_state;
	std::map<int, bool> release_output_nr_state;

	//Load parameter from the ROS-Parameter-Server
	bool load_parameters(){
		ros::NodeHandle nh;

		//Necessary parameters
		bool success = true;

		success = success && nh.getParam("/fake_io_service/grasp_output_nr", grasp_output_nr);
		success = success && nh.getParam("/fake_io_service/release_output_nr", release_output_nr);
		success = success && nh.getParam("/fake_io_service/grasp_input_nr", grasp_input_nr);
		success = success && nh.getParam("/fake_io_service/release_input_nr", release_input_nr);

		return success;
	}

};

int main(int argc, char **argv) {
	//Initiate node "fake_io_service_node"
	ros::init(argc, argv, "fake_io_service_node", ros::init_options::NoSigintHandler);
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



