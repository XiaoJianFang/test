cmake_minimum_required(VERSION 3.6)
project(SendMap)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
find_package(lcm)
include_directories(${Lcm_INCLUDE_DIRS})

set(SOURCE_FILES main.cpp SendMap.cpp map.cpp)
add_executable(SendMap ${SOURCE_FILES})

TARGET_LINK_LIBRARIES(SendMap   lcm -lpthread)