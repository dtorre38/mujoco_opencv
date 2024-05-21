cmake_minimum_required(VERSION 3.16)
project(opencv)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Append the project-specific cmake module path
list(APPEND CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/cmake")

# Set the C compiler flags to suppress warnings
set(CMAKE_C_FLAGS "-w -Wno-implicit-function-declaration -target x86_64-apple-darwin -mavx")

# Find the Mujoco and GLFW3 packages
find_package(mujoco REQUIRED)
find_package(glfw3 REQUIRED)
find_package(GSL REQUIRED)
find_package(OpenCV REQUIRED)

# Create the main executable
add_executable(main main.cpp)

# Link libraries to the main executable
target_link_libraries(main mujoco::mujoco glfw GSL::gsl GSL::gslcblas m ${OpenCV_LIBS})
