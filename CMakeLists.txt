cmake_minimum_required(VERSION 3.16.3)
project(test_control)

SET(CMAKE_VERBOSE_MAKEFILE ON)
#include_directories(${CMAKE_SOURCE_DIR}/lib/nlohmann)
message("OS: ${CMAKE_SYSTEM_NAME}")

add_subdirectory(base)

set_target_properties(test_every_joint PROPERTIES
        CXX_STANDARD 17
        CXX_STANDARD_REQUIRED ON)
