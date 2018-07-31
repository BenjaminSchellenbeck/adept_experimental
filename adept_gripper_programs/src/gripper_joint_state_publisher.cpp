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
#include <signal.h>

#include "ros/ros.h"

#include "adept_msgs/IOStates.h"
#include "adept_msgs/IO.h"
#include "sensor_msgs/JointState.h"
#include "std_msgs/Header.h"

using std::string;

class GripperJointStatePublisher
{
public:
  //! Constructor.
	GripperJointStatePublisher(std::vector<string> gripperNames, std::vector<int> gripperInputs, std::vector<double> gripperPositions){
		this->gripperNames = gripperNames;
		this->gripperInputs = gripperInputs;
		this->gripperPositions = gripperPositions;
		this->seq = 0;
	}

  //! Destructor.
  ~GripperJointStatePublisher(){
  }

  //! Publish the message.
  void publishMessage(ros::Publisher *pub_message){
		sensor_msgs::JointState jointState;
		std_msgs::Header header;
		for(int i = 0; i < gripperInputs.size(); i++){
			for (int k = 0; k < ioStates.states.size(); k++){
				if (ioStates.states[k].pin == gripperInputs[i]){
						if (ioStates.states[k].state == true){
							jointState.header.seq = seq;
							jointState.header.stamp = ros::Time::now();
							jointState.name.push_back(gripperNames[0]);
							jointState.name.push_back(gripperNames[1]);
							jointState.position.push_back(gripperPositions[i]);
							jointState.position.push_back(gripperPositions[i]*-1);

						}
				}
			}
		}

		jointState_to_publish = jointState;
		pub_message->publish(jointState_to_publish);

		this->seq += 1;
  }

  //! Callback function for subscriber.
  void messageCallback(const adept_msgs::IOStates &publishedIOStates){
	  ioStates = publishedIOStates;
  }

private:
  adept_msgs::IOStates ioStates;

  std::vector<string> gripperNames;
  std::vector<int> gripperInputs;
  std::vector<double> gripperPositions;
  sensor_msgs::JointState jointState_to_publish;

  sensor_msgs::JointState lastJointState;


  int seq;
};

void shutdown(int sig)
{
  ros::shutdown();
}

int main(int argc, char **argv) {

	ros::init(argc, argv, "gripper_joint_state_publisher", ros::init_options::NoSigintHandler);
	ros::NodeHandle n;

    std::string nodeName = ros::this_node::getName();

    int rate;
	ros::param::param<int>("/gripper_joint_state_publisher/rate", rate, 10);
	ros::Rate loop_rate(rate);

	std::vector<string> gripperNames;
	std::vector<int> gripperInputs;
	std::vector<double> gripperPositions;

	bool success;
	success =n.getParam("/gripper_joint_state_publisher/gripper_names", gripperNames);
	success = success && n.getParam("/gripper_joint_state_publisher/gripper_inputs", gripperInputs);
	success = success && n.getParam("/gripper_joint_state_publisher/gripper_positions", gripperPositions);

	if (success){
		ROS_INFO_STREAM(nodeName<<": Loaded parameters.");
	}
	else {
		ROS_ERROR_STREAM(nodeName<<": Did not find parameters, exiting");
		ros::shutdown();
	}

    GripperJointStatePublisher *gripperJointStatePublisher = new GripperJointStatePublisher(gripperNames, gripperInputs, gripperPositions);

	ros::Publisher joint_state_pub = n.advertise<sensor_msgs::JointState>("joint_states", 1000);

	ros::Subscriber io_state_subscriber = n.subscribe("io_state_publisher", 1000, &GripperJointStatePublisher::messageCallback, gripperJointStatePublisher );

	ROS_INFO_STREAM(nodeName<<": Ready to publish IO states.");
	while (ros::ok()){
		gripperJointStatePublisher->publishMessage(&joint_state_pub);

		ros::spinOnce();
		signal(SIGINT, shutdown);
		loop_rate.sleep();
	}
	delete gripperJointStatePublisher;

	return 0;
}


