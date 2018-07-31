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
 *   http://www.gnu.org/licenses/gpl.html
 */

#include <iostream>
#include <string.h>
#include <vector>
#include <map>
#include <signal.h>

#include "ros/ros.h"
#include "sensor_msgs/JointState.h"
#include "std_msgs/Header.h"

using namespace std;
using namespace sensor_msgs;


class JointStateAggregator
{
public:
	//Konstructor
	JointStateAggregator() {
		//Initiate private variables
		seq = 0;
		robotJointMessageAvailable = false;
		gripperJointMessageAvailable = false;
	}
	//Subscriber callback for the robot joint_state 
	void robotJointStatesMessageCallback(const sensor_msgs::JointState &publishedRobotJointStates)
	{
		robotJointState = publishedRobotJointStates;
		robotJointMessageAvailable = !robotJointState.name.empty();
	}
	//Subscriber callback for the gripper joint_state 
	void gripperJointStatesMessageCallback(const sensor_msgs::JointState &publishedGripperJointStates)
	{
		gripperJointState = publishedGripperJointStates;
		gripperJointMessageAvailable = !gripperJointState.name.empty();
	}
	//Generate message and publish this message
	void publishJointStatesMessage(ros::Publisher *pub_message)
	{
	    string nodeName = ros::this_node::getName();
		//If their is a gripper joint state message and a robot joint state message then generate
		//message and publish it 
		if (gripperJointMessageAvailable == true && robotJointMessageAvailable == true){
			jointStates = robotJointState;
			jointStates.header.seq = seq;
			jointStates.header.stamp = ros::Time::now();
			jointStates.name.resize(jointStates.name.size()+1,gripperJointState.name[0]);
			jointStates.name.resize(jointStates.name.size()+1,gripperJointState.name[1]);
			jointStates.position.resize(jointStates.position.size()+1,gripperJointState.position[0]);
			jointStates.position.resize(jointStates.position.size()+1,gripperJointState.position[1]);

			pub_message->publish(jointStates);

			seq += 1;
		}
		//Else
		else {
			if (gripperJointMessageAvailable == false) {
				ROS_INFO_STREAM(nodeName<<": waiting for gripper joint states");
			}
			if (robotJointMessageAvailable == false) {
				ROS_INFO_STREAM(nodeName<<": waiting for robot joint states");
			}
		}
	}

private:
	//Declare private variables
	bool robotJointMessageAvailable;
	bool gripperJointMessageAvailable;
	JointState robotJointState;
	JointState gripperJointState;
	JointState jointStates;
	int seq;

};

//Sigint handler for shutting down this node with strg+c
void shutdown(int sig)
{
  ros::shutdown();
}


int main(int argc, char **argv) {
	//Initiate node "joint_state_aggregator"
	ros::init(argc, argv, "joint_state_aggregator", ros::init_options::NoSigintHandler);
	//Access point to the ROS system
	ros::NodeHandle n;

	//Load parameter
    	string nodeName = ros::this_node::getName();
    	string robot_joint_state_topic_name;
    	string gripper_joint_state_topic_name;

    	int rate;
	ros::param::param<int>("/joint_state_aggregator/rate", rate, 10);
	ros::Rate loop_rate(rate);

	bool success;
	success = n.getParam("/joint_state_aggregator/robot_joint_state_topic_name", robot_joint_state_topic_name);
	success = success && n.getParam("/joint_state_aggregator/gripper_joint_state_topic_name", gripper_joint_state_topic_name);

	if (success){
		ROS_INFO_STREAM(nodeName<<": Loaded parameters.");
	}
	//Else print error into the Terminal and shutting down the programm
	else {
		ROS_ERROR_STREAM(nodeName<<": Did not find parameters, exiting");
		ros::shutdown();
	}

	//Instantiate jointStateAggregator from the class JointStateAggregator
    JointStateAggregator *jointStateAggregator = new JointStateAggregator();

	//Initiate Subsciber
	ros::Subscriber robot_joint_state_subscriber = n.subscribe(robot_joint_state_topic_name, 1000, &JointStateAggregator::robotJointStatesMessageCallback, jointStateAggregator);
	ros::Subscriber gripper_joint_state_subscriber = n.subscribe(gripper_joint_state_topic_name, 1000, &JointStateAggregator::gripperJointStatesMessageCallback, jointStateAggregator);

	//Initiate Publisher
	ros::Publisher joint_state_pub = n.advertise<sensor_msgs::JointState>("joint_states", 1000);

	//While the programm is running
	while (ros::ok()){
		//Generate Message and publish it on the joint_states Topic
		jointStateAggregator->publishJointStatesMessage(&joint_state_pub);

		ros::spinOnce();
		signal(SIGINT, shutdown);
		loop_rate.sleep();
	}
	delete jointStateAggregator;
	return 0;
}
