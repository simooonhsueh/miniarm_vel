cmake_minimum_required(VERSION 3.5)
project(MotorUnion)

add_subdirectory(Motor)

add_library(${PROJECT_NAME} SHARED MotorUnion.cpp)
target_link_libraries(${PROJECT_NAME} DMotor)

target_include_directories(${PROJECT_NAME} PUBLIC ${PROJECT_SOURCE_DIR})