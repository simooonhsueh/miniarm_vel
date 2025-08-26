cmake_minimum_required(VERSION 3.5)
project(Motor)


if (NOT TARGET DynamixelSdk)
add_subdirectory(dynamixel)
endif()
if (NOT TARGET motor)
add_subdirectory(motor)
endif()

add_library(DMotor SHARED Dmotor.cpp DmotorXm.cpp DmotorMx.cpp DmotorPro.cpp DmotorProPlus.cpp)
target_link_libraries(DMotor DynamixelSdk motor)
