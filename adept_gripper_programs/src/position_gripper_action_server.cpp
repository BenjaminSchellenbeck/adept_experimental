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

#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <adept_msgs/PositionGripperCommandAction.h>
#include <adept_msgs/PositionGripperCommandGoal.h>
#include "adept_msgs/AdeptIO.h"
#include "sensor_msgs/JointState.h"

using std::string;
using std::map;
using namespace std;

using namespace adept_msgs;
using namespace actionlib;

class GripperActionServer
{
protected:
	  PositionGripperCommandGoal goal_;
	  PositionGripperCommandFeedback feedback_;
	  PositionGripperCommandResult result_;

public:
	  GripperActionServer(ros::NodeHandle &n):
		  node_(n),
	      action_server_(node_, "gripper_action_server",
	                   boost::bind(&GripperActionServer::action_goal_callback, this, _1),
	                   boost::bind(&GripperActionServer::action_cancel_callback, this, _1),
	                   false)
	  {
	  }

	  bool init()
	  {
		    ros::NodeHandle nh("");
		    std::string nodeName = ros::this_node::getName();

			if(load_parameters() )
			{
				ROS_INFO_STREAM(nodeName<<": Loaded parameters.");
			}
			else
			{
				ROS_ERROR_STREAM(nodeName<<": Did not find required ros parameters, exiting");
				return false;
			}

		    // service client
		    service_client_ = nh.serviceClient<adept_msgs::AdeptIO>("io_service");
		    while(!service_client_.waitForExistence(ros::Duration(5.0f)))
		    {
		    	ROS_INFO_STREAM(nodeName<<": Waiting for Service (io_service) to start");
		    }

			int counter = 0;
		    for(int i=0; i < gripper_input_numbers.size(); i++){
		    	if (check_sensor_state(gripper_input_numbers[i])){
		    		current_gripper_position = gripper_positions[i];
		    		counter += 1;
		    	}
		    }

	    	if (counter > 1){
	    		ROS_INFO_STREAM(nodeName<<": Error: Two gripper inputs are true.");
	    		return false;
	    	}
		    return true;

	  }

	  void start()
	  {
		std::string nodeName = ros::this_node::getName();
		action_server_.start();
		ROS_INFO_STREAM(nodeName<<": Gripper action server started");
	  }

private:

	ros::NodeHandle node_;
	ActionServer<PositionGripperCommandAction> action_server_;
	ros::ServiceClient service_client_;

	std::vector<int> gripper_input_numbers;
	std::vector<float> gripper_positions;
	int grasp_output_nr;
	int release_output_nr;

	float min_position;
	float max_position;

	std::map<float, int> input_nr_position_table;
	float current_gripper_position;

	double goal_timeout;

	bool load_parameters(){
		ros::NodeHandle nh;

		//Necessary parameters
		bool success = true;

		success = success && nh.getParam("/gripper_action_server/grasp_output_nr", grasp_output_nr);
		success = success && nh.getParam("/gripper_action_server/release_output_nr", release_output_nr);

		success = success && nh.getParam("/gripper_action_server/min_position", min_position);
		success = success && nh.getParam("/gripper_action_server/max_position", max_position);

		success = success && nh.getParam("/gripper_action_server/gripper_input_numbers", gripper_input_numbers);
		success = success && nh.getParam("/gripper_action_server/gripper_positions", gripper_positions);

		success = success && nh.getParam("/gripper_action_server/goal_timeout", goal_timeout);

		for (int i=0; i < gripper_input_numbers.size();i++){
			input_nr_position_table[gripper_positions[i]] = int(gripper_input_numbers[i]);
		}

		if (gripper_input_numbers.size() != gripper_positions.size()){
			return false;
		}

		return success;
	}

	void action_goal_callback( ActionServer<PositionGripperCommandAction>::GoalHandle gh)
	{
		std::string nodeName = ros::this_node::getName();

		ROS_INFO("%s",(nodeName + ": Received gripper goal").c_str());

		adept_msgs::AdeptIO io_service;

		goal_.position = gh.getGoal()->position;

		if (set_gripper_state(gh)){

			get_current_gripper_position();
			ros::Time now;
			ros::Time start = ros::Time::now();
			ros::Duration timeout(goal_timeout);
			while ((float)current_gripper_position != (float)goal_.position) {

				feedback_.position = current_gripper_position;

				gh.publishFeedback(feedback_);
				get_current_gripper_position();
				now = ros::Time::now();

				if ((now-start) > timeout) {
					break;
				}
			}
			result_.position = current_gripper_position;

			gh.setSucceeded(result_);
			ROS_INFO_STREAM(nodeName + ": Gripper command succeeded");
		}

	}

	  void action_cancel_callback(ActionServer<PositionGripperCommandAction>::GoalHandle gh)
	    {
			std::string nodeName = ros::this_node::getName();

			ROS_INFO_STREAM(nodeName + ": Canceling current gripper action");

			gh.setCanceled();

			ROS_INFO_STREAM(nodeName + ": Current gripper action has been canceled");
	    }


	bool set_gripper_state(ActionServer<PositionGripperCommandAction>::GoalHandle gh){
		std::string nodeName = ros::this_node::getName();

		ROS_INFO("%s",(nodeName + ": Received gripper goal").c_str());

		adept_msgs::AdeptIO io_service_reset, io_service_set;

		goal_.position = gh.getGoal()->position;

		if (goal_.position >= min_position || goal_.position <= max_position){

			if ( input_nr_position_table.find(goal_.position) == input_nr_position_table.end()){
				gh.setRejected();
				ROS_INFO_STREAM(nodeName + ": Gripper goal command rejected. Position not available");
				return false;
			}
			else {
				gh.setAccepted();
				ROS_INFO_STREAM(nodeName + ": Goal command accepted");

				std::cout << goal_.position << std::endl;
				std::cout << current_gripper_position << std::endl;

				if ((float)goal_.position > (float)current_gripper_position){
					io_service_reset.request.mode = adept_msgs::AdeptIO::Request::WRITE;
					io_service_reset.request.io_number = grasp_output_nr*-1;

					io_service_set.request.mode = adept_msgs::AdeptIO::Request::WRITE;
					io_service_set.request.io_number = release_output_nr;
				}
				else if ((float)goal_.position < (float)current_gripper_position){
					io_service_reset.request.mode = adept_msgs::AdeptIO::Request::WRITE;
					io_service_reset.request.io_number = release_output_nr*-1;

					io_service_set.request.mode = adept_msgs::AdeptIO::Request::WRITE;
					io_service_set.request.io_number = grasp_output_nr;
				}
				else if ((float)goal_.position == (float)current_gripper_position){
					result_.position = current_gripper_position;

					gh.setSucceeded(result_);
					ROS_INFO_STREAM(nodeName + ": Gripper command succeeded");
					return false;
				}
				else {
					ROS_WARN_STREAM(nodeName + ": Unidentified gripper request, rejecting request");
					gh.setRejected();
					return false;
				}

				if(service_client_.call(io_service_reset))
				{
					ROS_INFO_STREAM(nodeName + ": Gripper goal command posted");
				}
				else {
						gh.setAborted();
						ROS_INFO_STREAM(nodeName + ": Gripper goal command aborted. Check IO-Service.");
				}

				if(service_client_.call(io_service_set))
				{
					ROS_INFO_STREAM(nodeName + ": Gripper goal command posted");
				}
				else {
						gh.setAborted();
						ROS_INFO_STREAM(nodeName + ": Gripper goal command aborted. Check IO-Service.");
				}

			}

		}
		else {
			gh.setRejected();
			ROS_INFO_STREAM(nodeName + ": Gripper goal command rejected. Position not in range");
			return false;
		}
		return true;
	}

	bool check_sensor_state(int input_nr){
		adept_msgs::AdeptIO io_service;
		io_service.request.mode = adept_msgs::AdeptIO::Request::READ;
		io_service.request.io_number = input_nr;
		if(service_client_.call(io_service) && (bool)io_service.response.result == true ){
			ROS_INFO_STREAM("check_sensor_state: true");
			return true;
		}
		else return false;
	}

	bool get_current_gripper_position(){

	    std::string nodeName = ros::this_node::getName();
		int counter = 0;
	    for(int i=0; i < gripper_input_numbers.size(); i++){
	    	if (check_sensor_state(gripper_input_numbers[i])){
	    		current_gripper_position = gripper_positions[i];
	    		counter += 1;
	    	}
	    }

    	if (counter > 1){
    		ROS_INFO_STREAM(nodeName<<": Error: Two gripper inputs are true.");
    		return false;
    	}

	    return true;
	}

};

int main(int argc, char** argv)
{
	ros::init(argc, argv, "gripper_action_server_node");

	ros::AsyncSpinner spinner(4);
	spinner.start();
	ros::NodeHandle n("");

	GripperActionServer gripper_action_server(n);
	if(gripper_action_server.init())
	{
		gripper_action_server.start();
	}

	ros::waitForShutdown();
  return 0;
}



