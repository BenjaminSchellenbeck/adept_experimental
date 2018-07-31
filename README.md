# adept_experimental
Experimental package for Adept manipulators within ROS-Industrial

All packages have been completely rebuilt. The reference package was abb_experimental and the ROS-Industrial Tutorials.
The V + programs are a further development of the original V + program from the package swri-ros-pkg (https://github.com/ros-industrial/swri-ros-pkg/tree/master/adept/adept_common).
The programs were tested under ROS Indigo and Kinetic

## Contents:

### adept_driver:
Includes programs for communication between the Adept Controller and ROS-Industrial.

#### ROS-Industrial:

* Starts the industrial-robot client in Trajectory Downloading or Trajectory Streaming mode.
* Starts the node robot_state for receiving robot status messages.
* Includes an I / O interface.
* Includes the program adept_fake_io_service_node.cpp that simulates the I / O interface. A pneumatic parallel gripper can be simulated, all other inputs and outputs directly return true.

#### V+ programs:

* The V + programs differ only in the different conversion of the axis values.
* It automatically detects if the trajectory downloading or trajectory streaming mode is used.
* In streaming mode, stop messages cause the robot to stop immediately.
* Contains an I/O interface via a socket server
* Published continuously robot status messages

### adept_msgs:

Message definition of the I/O interface.

### adept_viper_s650_support:

Support package for the Adept Viper s650 (6-Axis).

### adept_viper_s650_moveit_plugins:

IKFast Plugin for the Adept Viper s650.

### adept_viper_s650_moveit_config:

MoveIt! package for the Adept Viper s650.

### adept_cobra_s600_support:

Support package for the Adept Cobra s600 (SCARA).

### adept_cobra_s600_moveit_config:

MoveIt! package for the Adept Cobra s600 (SCARA)

### adept_gripper_programs:

Contains programs for controlling grippers on Adept industrial robots via ROS.

#### Program adept_simple-gripper_action_server.cpp:
	
* Allows the control of a suction gripper or a pneumatic parallel gripper with only 2 states which are interrogated via sensors.
* Example action clients are in the  programs test_gripper_action_client_release.cpp and test_gripper_action_client_grasp.cpp.

#### Program adept_position_gripper_action_server.cpp:
		
* Allows the control of pneumatic parallel grippers with several gripping positions. The respective gripping position is determined by the distance of a gripper jaw from the center between both gripper jaws.
* Example action clients are in the programs test_position_gripper_action_client_release.cpp and test_position_gripper_action_client_grasp.cpp.

#### Program gripper_joint_state_publisher.cpp:

* Queries the status of the gripper continuously via the I/O interface and publishes it with the message type JointState by default on the topic joint_states.

### test_gripper_description:
	
Includes sample 3D models and URDF files from a gripper on the Adept Viper s650

### adept_viper_s650_with_test_gripper_moveit_config:

This MoveIT! package is an example of how to deal with the adept_gripper_program. It uses 	the example gripper mounted on the Viper s650 from the package 	test_gripper_description.


### joint_state_aggregator

Combines JointState messages from the gripper_joint_state_publisher and the ros-industrial programs and publish the results on the topic joint_states.

## Start the programs:

### Start Viper s650 without gripper:

1. Load the V + file adept_ros.v2 into Adept ACE and start the program a.ros () in task 0.
2. Start MoveIT in trajectory downloading mode:

```
roslaunch adept_viper_s650_moveit_config moveit_planning_execution_downloader.launch sim:=false robot_ip:=xxx.xxx.xxx.xxx

```
or start in trajectory streaming mode:

```
roslaunch adept_viper_s650_moveit_config moveit_planning_execution_streamer.launch sim:=false 	robot_ip:=xxx.xxx.xxx.xxx
```

### Start Viper s650 with gripper:

Note: For this we need the joint_state_aggregator.

1. 1. Load the V + file adept_ros.v2 in Adept ACE and start the program a.ros () in task 0. To simulate a gripper, the program gripper_test () can be started in task 3 and the output 104 must manually set to true.
2. Start MoveIT in trajectory downloading mode:
```
roslaunch adept_viper_s650_test_gripper_moveit_config moveit_planning_execution_downloader.launch sim:=false robot_ip:=xxx.xxx.xxx.xxx
```
or start in trajectory streaming mode:
```
roslaunch adept_viper_s650_test_gripper_moveit_config moveit_planning_execution_streamer.launch sim:=false robot_ip:=xxx.xxx.xxx.xxx
```

### Start Cobra s600:

1. Load the V + file adept_ros_scara.v2 into Adept ACE and start the program a.ros () in task 0.
2. Start MoveIT in trajectory downloading mode:
```
roslaunch adept_cobra_s600_moveit_config moveit_planning_execution_downloader.launch sim:=false robot_ip:=xxx.xxx.xxx.xxx
```
or start in trajectory streaming mode:
```
roslaunch adept_cobra_s600_moveit_config moveit_planning_execution_streamer.launch sim:=false 	robot_ip:=xxx.xxx.xxx.xxx
```	
