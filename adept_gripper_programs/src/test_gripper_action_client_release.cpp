/*
 *   Copyright (c) 2018 Benjamin Schellenbeck.  All rights reserved.
 *
*/

#include <iostream>
#include <string.h>

#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <adept_msgs/SimpleGripperCommandAction.h>

using namespace adept_msgs;

int main (int argc, char **argv)
{
  ros::init(argc, argv, "test_gripper_action_client");

  // create the action client
  // true causes the client to spin its own thread
  actionlib::SimpleActionClient<SimpleGripperCommandAction> GraspActionClient("grasp_execution_action", true);

  ROS_INFO("Waiting for action server to start.");
  // wait for the action server to start
  GraspActionClient.waitForServer(); //will wait for infinite time

  ROS_INFO("Action server started, sending goal.");
  // send a goal to the action
  SimpleGripperCommandGoal grasp_goal;
  grasp_goal.goal = SimpleGripperCommandGoal::RELEASE;
  GraspActionClient.sendGoal(grasp_goal);

  //wait for the action to return
  bool finished_before_timeout = GraspActionClient.waitForResult(ros::Duration(30.0));

  if (finished_before_timeout)
  {
    actionlib::SimpleClientGoalState state = GraspActionClient.getState();
    ROS_INFO("Action finished: %s",state.toString().c_str());
  }
  else
    ROS_INFO("Action did not finish before the time out.");

  //exit
  return 0;
}

