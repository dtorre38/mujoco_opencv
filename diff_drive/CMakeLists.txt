cmake_minimum_required(VERSION 3.16)
project(diff_drive)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Append the project-specific cmake module path
list(APPEND CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/cmake")

# Set the C compiler flags to suppress warnings
set(CMAKE_C_FLAGS "-w -Wno-implicit-function-declaration -target x86_64-apple-darwin -mavx")

# Find the required packages
find_package(mujoco REQUIRED)
find_package(glfw3 REQUIRED)
find_package(OpenGL REQUIRED)
find_package(GSL REQUIRED)
find_package(OpenCV REQUIRED)

# Create the main executable
add_executable(main main.cpp) # must use cpp to run opencv

# Link libraries to the main executable
target_link_libraries(main PRIVATE mujoco::mujoco glfw OpenGL::GL GSL::gsl GSL::gslcblas m ${OpenCV_LIBS})

# Set the output directory for the executable to the parent directory of the build folder
set_target_properties(main PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/.."
)