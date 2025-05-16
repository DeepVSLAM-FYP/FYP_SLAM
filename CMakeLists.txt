cmake_minimum_required(VERSION 3.10)
project(SLAM_Systems)

# Set default build type to Release
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()
message("Build type: " ${CMAKE_BUILD_TYPE})

# Define PROJECT_ROOT_DIR for both SLAM systems
set(PROJECT_ROOT_DIR ${CMAKE_CURRENT_SOURCE_DIR})

option(DEBUG_PRINT "Enable debug print" ON)
option(O3_DEBUG "Enable O3 optimization on debug" ON)
option(USE_TOOLCHAIN "Use toolchain" OFF)
option(BUILD_SP_DPU "Build SuperPoint DPU libs and examples" OFF)

if(USE_TOOLCHAIN)
    set(CMAKE_TOOLCHAIN_FILE "${PROJECT_ROOT_DIR}/toolchain-kr260.cmake")
endif()


if(DEBUG_PRINT)
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -DDEBUG_PRINT")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DDEBUG_PRINT")
endif()


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


# Option to build specific SLAM system
option(BUILD_ORBSLAM3 "Build ORB-SLAM3" ON)

# Add ORBSLAM3 if enabled
if(BUILD_ORBSLAM3)
  # Pass BUILD_SP_DPU option to ORBSLAM3
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

# Print status about SuperPoint DPU support
if(BUILD_SP_DPU)
  message(STATUS "SuperPoint DPU support is enabled")
else()
  message(STATUS "SuperPoint DPU support is disabled")
endif()

# Use toolchain file if specified
if(DEFINED CMAKE_TOOLCHAIN_FILE)
  message(STATUS "Using toolchain file: ${CMAKE_TOOLCHAIN_FILE}")
else()
  message(STATUS "No toolchain file specified. Using default system root.")
endif()
