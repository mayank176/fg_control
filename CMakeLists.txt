cmake_minimum_required(VERSION 3.10)

# Project name
project(fg_control)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(Boost REQUIRED COMPONENTS system)
include_directories(${Boost_INCLUDE_DIRS})

# Add the directory to search for header files
include_directories(include)

# Add executable
add_executable(fg_control src/main.cpp
                            src/udp_io.cpp
                            src/flight_controller.cpp
)

target_link_libraries(fg_control ${Boost_LIBRARIES})