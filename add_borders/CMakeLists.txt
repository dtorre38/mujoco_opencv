cmake_minimum_required(VERSION 3.10)

# Set the project name and version
project(add_borders VERSION 1.0)

# Specify the C++ standard
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED True)

# Find OpenCV package
find_package(OpenCV REQUIRED)

# Add the executable
add_executable(add_borders add_borders.cpp)

# Link OpenCV libraries
target_link_libraries(add_borders ${OpenCV_LIBS})
