cmake_minimum_required(VERSION 3.15)
project(OPWKinematicsConsumer LANGUAGES CXX)

find_package(OPWKinematics 0.1 CONFIG REQUIRED)
add_executable(opw_consumer main.cpp)
target_link_libraries(opw_consumer PRIVATE OPWKinematics::OPWKinematics)
