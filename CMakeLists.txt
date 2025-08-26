cmake_minimum_required(VERSION 3.0.2)
project(miniarm_vel)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  serial
  message_generation
)

## 服務檔
add_service_files(
  FILES
  MoveArm.srv
  ServoControl.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

## catkin package
catkin_package(
  CATKIN_DEPENDS roscpp rospy std_msgs message_runtime
  # LIBRARIES manipulator  # 不用寫也可以
  # INCLUDE_DIRS include
)

## include
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  /usr/include/eigen3
  ${PROJECT_SOURCE_DIR}/MotorUnion
)

add_subdirectory(MotorUnion)

## manipulator library
add_library(manipulator
  src/Manipulator.cpp
)

## test_trajectory
add_executable(test_trajectory
  src/test_trajectory.cpp
)
target_link_libraries(test_trajectory
  manipulator
  ${catkin_LIBRARIES}
  MotorUnion
)

## arm_server
add_executable(arm_server src/arm_server.cpp)
target_link_libraries(arm_server
  manipulator
  ${catkin_LIBRARIES}
  MotorUnion
)
add_dependencies(arm_server ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## arm_client
add_executable(arm_client src/arm_client.cpp)
target_link_libraries(arm_client
  manipulator
  ${catkin_LIBRARIES}
  MotorUnion
)
add_dependencies(arm_client ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## gripper_control_server
add_executable(gripper_control_server src/gripper_control_server.cpp)
target_link_libraries(gripper_control_server
  manipulator
  ${catkin_LIBRARIES}
  MotorUnion
)
add_dependencies(gripper_control_server ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

