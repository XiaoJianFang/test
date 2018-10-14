cmake_minimum_required(VERSION 2.8)
project(ibeosdk_demos)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}  -std=c++0x")

#SET(CMAKE_BUILD_TYPE "Debug")
SET(CMAKE_BUILD_TYPE "Release")
SET(CMAKE_CXX_FLAGS_DEBUG "$ENV{CXXFLAGS} -O0 -Wall -g -ggdb")
SET(CMAKE_CXX_FLAGS_RELEASE "$ENV{CXXFLAGS} -O3 -Wall")
include_directories(${CMAKE_SOURCE_DIR}/include )
LINK_DIRECTORIES(${CMAKE_SOURCE_DIR}/lib)

#lcm通信
find_package(lcm)
include_directories(${Lcm_INCLUDE_DIRS})

find_package(Boost REQUIRED COMPONENTS ${NeededBoostComponents})
include_directories(${Boost_INCLUDE_DIRS})
link_directories(${Boost_LIBRARY_DIRS})


#set(SDKLINKLIB -Wl,--whole-archive ibeosdk -Wl,--no-whole-archive)

SET(DIR_SRCS IbeoSdkLuxLiveDemonew.cpp)
ADD_EXECUTABLE(LidarDriver_Release ${DIR_SRCS})

target_link_libraries(LidarDriver_Release  lcm ibeosdk  -lpthread  libVelodyne.a libibeosdk.a libboost_system.a libboost_thread.a libboost_date_time.a libboost_program_options.a )
		


#######################################################  ${Boost_LIBRARIES}
