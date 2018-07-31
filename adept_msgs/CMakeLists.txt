cmake_minimum_required(VERSION 2.8.3)
project(adept_msgs)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  std_msgs
  message_generation
  actionlib_msgs
)

## Generate messages in the 'msg' folder
 add_message_files(
   FILES
   IO.msg
   IOStates.msg
)

## Generate services in the 'srv' folder
add_service_files(
   FILES
   AdeptIO.srv
)

## Generate actions in the 'action' folder
 add_action_files(
   DIRECTORY action
   FILES
   SimpleGripperCommand.action
   PositionGripperCommand.action
 )

## Generate added messages and services with any dependencies listed here
 generate_messages(
   DEPENDENCIES
   std_msgs
   actionlib_msgs
)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES adept_msgs
  CATKIN_DEPENDS std_msgs message_runtime actionlib_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

