cmake_minimum_required(VERSION 3.10)
project(SLAM_Systems)

# Set default build type to Release
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()
message("Build type: " ${CMAKE_BUILD_TYPE})

# Global compiler flags
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -O3")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3")
set(CMAKE_C_FLAGS_RELEASE "${CMAKE_C_FLAGS_RELEASE} -march=native")
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -march=native")

# Define PROJECT_ROOT_DIR for both SLAM systems
set(PROJECT_ROOT_DIR ${CMAKE_CURRENT_SOURCE_DIR})

# Option to build specific SLAM system
option(BUILD_ORBSLAM3 "Build ORB-SLAM3" ON)
option(BUILD_XFEATSLAM "Build XFeatSLAM" ON)

# Add ORBSLAM3 if enabled
if(BUILD_ORBSLAM3)
  # ORB-SLAM3 uses C++14
  add_subdirectory(ORBSLAM3)
endif()

# Add XFeatSLAM if enabled
if(BUILD_XFEATSLAM)
  # XFeatSLAM uses C++17
  add_subdirectory(XFeatSLAM)
endif()

# Message about what's being built
message(STATUS "Building:")
if(BUILD_ORBSLAM3)
  message(STATUS "  - ORB-SLAM3 (C++14)")
endif()
if(BUILD_XFEATSLAM)
  message(STATUS "  - XFeatSLAM (C++17)")
endif()

# Print a warning if nothing is being built
if(NOT BUILD_ORBSLAM3 AND NOT BUILD_XFEATSLAM)
  message(WARNING "Neither ORB-SLAM3 nor XFeatSLAM is selected for building!")
endif()