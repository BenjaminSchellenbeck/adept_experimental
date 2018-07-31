cmake_minimum_required(VERSION 2.8.3)
project(joint_state_aggregator)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  std_msgs
)
###################################
## catkin specific configuration ##
###################################

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES joint_state_aggregator
   CATKIN_DEPENDS roscpp sensor_msgs std_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)
add_executable(${PROJECT_NAME}_joint_state_aggregator src/joint_state_aggregator.cpp)
target_link_libraries(${PROJECT_NAME}_joint_state_aggregator ${catkin_LIBRARIES})
add_dependencies(${PROJECT_NAME}_joint_state_aggregator ${catkin_EXPORTED_TARGETS})
set_target_properties(${PROJECT_NAME}_joint_state_aggregator PROPERTIES OUTPUT_NAME joint_state_aggregator PREFIX "")

#############
## Install ##
#############

#install(
#  TARGETS
#   ${PROJECT_NAME}_joint_state_aggregator
#    RUNTIME DESTINATION
#    ${CATKIN_PACKAGE_BIN_DESTINATION}
#)
