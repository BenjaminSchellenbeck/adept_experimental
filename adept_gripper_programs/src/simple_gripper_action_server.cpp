/*
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2018, Benjamin Schellenbeck
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *       * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *       * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *       * Neither the name of the copyright holder nor the names 
 *       of its contributors may be used to endorse or promote products derived 
 *       from this software without specific prior written permission.
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
 *
 * Inspired by https://github.com/ros-industrial/industrial_training/blob/indigo/training/work/demo_manipulation/src/robot_io/src/nodes/suction_gripper_action_server.cpp
 */

#include <iostream>
#include <string.h>

#include <ros/ros.h>
#include <actionlib/server/action_server.h>
#include "adept_msgs/AdeptIO.h"
#include <adept_msgs/SimpleGripperCommandAction.h>
#include <adept_msgs/SimpleGripperCommandGoal.h>

using std::string;

using namespace adept_msgs;
using namespace actionlib;

class GripperActionServer
{

public:
	  GripperActionServer(ros::NodeHandle &n):
		  node_(n),
		  action_server_(node_,
				  	  	 "grasp_execution_action",
						 boost::bind(&GripperActionServer::action_goal_callback, this, _1),
						 boost::bind(&GripperActionServer::action_cancel_callback, this, _1),
						 false)
	  {
	  }

	  bool init()
	  {
		    ros::NodeHandle nh;
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
	  ActionServer<SimpleGripperCommandAction> action_server_;
	  ros::ServiceClient service_client_;

	  bool suction_gripper;

	  int grasp_output_nr;
	  int release_output_nr;
	  int grasp_input_nr;
	  int release_input_nr;

	  void action_goal_callback( ActionServer<SimpleGripperCommandAction>::GoalHandle gh)
	    {
		  std::string nodeName = ros::this_node::getName();

		  ROS_INFO("%s",(nodeName + ": Received gripper goal").c_str());

		  adept_msgs::AdeptIO io_service;

		  switch(gh.getGoal()->goal)
		  	{
		  		case SimpleGripperCommandGoal::GRASP:

		  			gh.setAccepted();
		  			ROS_INFO_STREAM(nodeName + ": Grasp command accepted");

		  			io_service.request.mode = adept_msgs::AdeptIO::Request::WRITE;
		  			io_service.request.io_number = release_output_nr*-1;
		  			if (suction_gripper == false){
						if(!service_client_.call(io_service)){
							gh.setAborted();
							ROS_INFO_STREAM(nodeName + ": Set release_output to 0 failed");
							break;
						}

		  			io_service.request.io_number = grasp_output_nr;

		  			}

		  			if(service_client_.call(io_service))
					{
		  				if (!suction_gripper){
							if(!check_sensor_state(grasp_input_nr) )
							{
								gh.setAborted();
								ROS_INFO_STREAM(nodeName + ": Grasp check failed");
								break;
							}
		  				}
					}
					else
					{
						gh.setAborted();
						ROS_INFO_STREAM(nodeName + ": Grasp command aborted");
						break;
					}

					gh.setSucceeded();
					ROS_INFO_STREAM(nodeName + ": Grasp command succeeded");
					break;

		  		case SimpleGripperCommandGoal::RELEASE:

		  			gh.setAccepted();
		  			ROS_INFO_STREAM(nodeName + ": Release command accepted");

		  			io_service.request.mode = adept_msgs::AdeptIO::Request::WRITE;
		  			io_service.request.io_number = grasp_output_nr*-1;

		  			if (suction_gripper == false) {
						if(!service_client_.call(io_service)){
							gh.setAborted();
							ROS_INFO_STREAM(nodeName + ": Set grasp_output to 0 failed");
							break;
						}

						io_service.request.io_number = release_output_nr;
		  			}

		  			if(service_client_.call(io_service))
		  			{
		  				ROS_INFO_STREAM(suction_gripper);
		  				if (!suction_gripper){
							if(!check_sensor_state(release_input_nr))
							{
								gh.setAborted();
								ROS_INFO_STREAM(nodeName + ": Release check failed");
								break;
							}
		  				}
					}
					else
					{
						gh.setAborted();
						ROS_INFO_STREAM(nodeName + ": Release command aborted");
						break;
					}

					gh.setSucceeded();
					ROS_INFO_STREAM(nodeName + ": Release command succeeded");
					break;

		  		default:

		  			ROS_WARN_STREAM(nodeName + ": Unidentified grasp request, rejecting request");
		  			gh.setRejected();
		  			break;
		  	}

	    }
	  void action_cancel_callback(ActionServer<SimpleGripperCommandAction>::GoalHandle gh)
	    {
		    std::string nodeName = ros::this_node::getName();

			ROS_INFO_STREAM(nodeName + ": Canceling current gripper action");

		    gh.setCanceled();

		    ROS_INFO_STREAM(nodeName + ": Current gripper action has been canceled");
	    }

	  bool load_parameters(){
	  		ros::NodeHandle nh;

	  		//Necessary parameters
	  		bool success = true;

	  		success = success && nh.getParam("/gripper_action_server/suction_gripper", suction_gripper);
	  		success = success && nh.getParam("/gripper_action_server/grasp_output_nr", grasp_output_nr);
	  		if (!suction_gripper == true){
		  		success = success && nh.getParam("/gripper_action_server/release_output_nr", release_output_nr);
	  		}
	  		success = success && nh.getParam("/gripper_action_server/grasp_input_nr", grasp_input_nr);
	  		success = success && nh.getParam("/gripper_action_server/release_input_nr", release_input_nr);

	  		return success;
	  }

	  bool check_sensor_state(int input_nr)
	  {
		  adept_msgs::AdeptIO io_service;
	      io_service.request.mode = adept_msgs::AdeptIO::Request::READ;
	      io_service.request.io_number = input_nr;
		  if(service_client_.call(io_service) && (bool)io_service.response.result == true ){
			  ROS_INFO_STREAM("check_sensor_state: true");
			  return true;
		  }
		  return false;
	  }

};

int main(int argc, char** argv)
{
	ros::init(argc, argv, "gripper_action_server");

	ros::AsyncSpinner spinner(4);
	spinner.start();
	ros::NodeHandle nh;

	GripperActionServer gripper_action_server(nh);
	if(gripper_action_server.init())
	{
		gripper_action_server.start();
	}

	ros::waitForShutdown();

	return 0;
}
