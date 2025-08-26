cmake_minimum_required(VERSION 3.5)
project(motor)

if (NOT TARGET motor)
    add_library(motor SHARED motor.cpp)
endif()