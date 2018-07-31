cmake_minimum_required(VERSION 2.8.3)
project(adept_gripper_programs)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  actionlib
  actionlib_msgs 
  sensor_msgs
  std_msgs
  adept_msgs
)

###################################
## catkin specific configuration ##
###################################

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES adept_gripper_programs
  CATKIN_DEPENDS roscpp sensor_msgs std_msgs adept_msgs actionlib actionlib_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

include_directories(include
                    ${catkin_INCLUDE_DIRS})


add_executable(${PROJECT_NAME}_gripper_joint_state_publisher src/gripper_joint_state_publisher.cpp)
target_link_libraries(${PROJECT_NAME}_gripper_joint_state_publisher ${catkin_LIBRARIES})
add_dependencies(${PROJECT_NAME}_gripper_joint_state_publisher ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_gripper_joint_state_publisher ${catkin_LIBRARIES})
set_target_properties(${PROJECT_NAME}_gripper_joint_state_publisher PROPERTIES OUTPUT_NAME gripper_joint_state_publisher PREFIX "")

add_executable(${PROJECT_NAME}_simple_gripper_action_server src/simple_gripper_action_server.cpp )
target_link_libraries(${PROJECT_NAME}_simple_gripper_action_server ${catkin_LIBRARIES})
add_dependencies(${PROJECT_NAME}_simple_gripper_action_server ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_simple_gripper_action_server ${catkin_LIBRARIES})
set_target_properties(${PROJECT_NAME}_simple_gripper_action_server PROPERTIES OUTPUT_NAME simple_gripper_action_server PREFIX "")

add_executable(${PROJECT_NAME}_position_gripper_action_server src/position_gripper_action_server.cpp )
target_link_libraries(${PROJECT_NAME}_position_gripper_action_server ${catkin_LIBRARIES})
add_dependencies(${PROJECT_NAME}_position_gripper_action_server ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_position_gripper_action_server ${catkin_LIBRARIES})
set_target_properties(${PROJECT_NAME}_position_gripper_action_server PROPERTIES OUTPUT_NAME position_gripper_action_server PREFIX "")

add_executable(${PROJECT_NAME}_test_gripper_action_client_grasp src/test_gripper_action_client_grasp.cpp )
target_link_libraries(${PROJECT_NAME}_test_gripper_action_client_grasp ${catkin_LIBRARIES})
add_dependencies(${PROJECT_NAME}_test_gripper_action_client_grasp ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_test_gripper_action_client_grasp ${catkin_LIBRARIES})
set_target_properties(${PROJECT_NAME}_test_gripper_action_client_grasp PROPERTIES OUTPUT_NAME test_gripper_action_client_grasp PREFIX "")

add_executable(${PROJECT_NAME}_test_gripper_action_client_release src/test_gripper_action_client_release.cpp )
target_link_libraries(${PROJECT_NAME}_test_gripper_action_client_release ${catkin_LIBRARIES})
add_dependencies(${PROJECT_NAME}_test_gripper_action_client_release ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_test_gripper_action_client_release ${catkin_LIBRARIES})
set_target_properties(${PROJECT_NAME}_test_gripper_action_client_release PROPERTIES OUTPUT_NAME test_gripper_action_client_release PREFIX "")

add_executable(${PROJECT_NAME}_test_position_gripper_action_client_grasp src/test_position_gripper_action_client_grasp.cpp )
target_link_libraries(${PROJECT_NAME}_test_position_gripper_action_client_grasp ${catkin_LIBRARIES})
add_dependencies(${PROJECT_NAME}_test_position_gripper_action_client_grasp ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_test_position_gripper_action_client_grasp ${catkin_LIBRARIES})
set_target_properties(${PROJECT_NAME}_test_position_gripper_action_client_grasp PROPERTIES OUTPUT_NAME test_position_gripper_action_client_grasp PREFIX "")

add_executable(${PROJECT_NAME}_test_position_gripper_action_client_release src/test_position_gripper_action_client_release.cpp )
target_link_libraries(${PROJECT_NAME}_test_position_gripper_action_client_release ${catkin_LIBRARIES})
add_dependencies(${PROJECT_NAME}_test_position_gripper_action_client_release ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_test_position_gripper_action_client_release ${catkin_LIBRARIES})
set_target_properties(${PROJECT_NAME}_test_position_gripper_action_client_release PROPERTIES OUTPUT_NAME test_position_gripper_action_client_release PREFIX "")


#############
## Install ##
#############

install(
  TARGETS
    ${PROJECT_NAME}_gripper_joint_state_publisher
    ${PROJECT_NAME}_simple_gripper_action_server
    ${PROJECT_NAME}_position_gripper_action_server
    ${PROJECT_NAME}_test_gripper_action_client_grasp
    ${PROJECT_NAME}_test_gripper_action_client_release
    ${PROJECT_NAME}_test_position_gripper_action_client_grasp
    ${PROJECT_NAME}_test_position_gripper_action_client_release
    RUNTIME DESTINATION
    ${CATKIN_PACKAGE_BIN_DESTINATION}
)

