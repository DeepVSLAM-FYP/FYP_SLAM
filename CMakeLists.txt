cmake_minimum_required(VERSION 3.10)
project(SLAM_Systems)

# Set default build type to Release
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Debug)
endif()
message("Build type: " ${CMAKE_BUILD_TYPE})

option(DEBUG_PRINT "Enable debug print" ON)

if(DEBUG_PRINT)
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -DDEBUG_PRINT")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DDEBUG_PRINT")
endif()

option(O3_DEBUG "Enable O3 optimization on debug" ON)

if(O3_DEBUG)
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -O3")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3")
else()
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -O0")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O0")
    set(CMAKE_C_FLAGS_RELEASE "${CMAKE_C_FLAGS_RELEASE} -O3")
    set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -O3")
endif()

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-deprecated-declarations -Wconversion-null -Wdeprecated" )
set(CMAKE_C_FLAGS_RELEASE "${CMAKE_C_FLAGS_RELEASE} -march=native")
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -march=native")

# Define PROJECT_ROOT_DIR for both SLAM systems
set(PROJECT_ROOT_DIR ${CMAKE_CURRENT_SOURCE_DIR})

# Option to build specific SLAM system
option(BUILD_ORBSLAM3 "Build ORB-SLAM3" ON)

# Add ORBSLAM3 if enabled
if(BUILD_ORBSLAM3)
  # ORB-SLAM3 uses C++14
  add_subdirectory(ORBSLAM3)
endif()

# Message about what's being built
message(STATUS "Building:")
if(BUILD_ORBSLAM3)
  message(STATUS "  - ORB-SLAM3 (C++17)")
endif()

# Print a warning if nothing is being built
if(NOT BUILD_ORBSLAM3)
  message(WARNING "ORB-SLAM3 is not selected for building!")
endif()
