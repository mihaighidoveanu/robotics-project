##
# Copyright (c) 2011 Aldebaran Robotics. All Rights Reserved.

cmake_minimum_required(VERSION 2.6.4 FATAL_ERROR)
set(CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}" ${CMAKE_MODULE_PATH})

# needed for ABI compatibility
set(CMAKE_CXX_FLAGS "-D_GLIBCXX_USE_CXX11_ABI=0")

# Give a name to the project.
project(nao-imitator)
# This include enable you to use our CMake framework
find_package(qibuild)

# Create an executable named proj,
# with the source file : main.cpp
qi_create_bin(proj main.cpp kinect.cpp nao.cpp imitator.cpp humanSkeleton.cpp ${CMAKE_SOURCE_DIR}/../NAOH25/NAOKinematics.cpp evaluate.cpp)

# Tell CMake that proj depends on ALCOMMON and ALPROXIES
# This will set the libraries to link proj with,
# the include paths, and so on
qi_use_lib(proj ALCOMMON ALPROXIES)

include_directories(".")
include_directories(${CMAKE_SOURCE_DIR}/../eigen)
include_directories(${CMAKE_SOURCE_DIR}/../NAOH25)

