cmake_minimum_required(VERSION 3.10)
project(xfeatSLAM)

IF(NOT CMAKE_BUILD_TYPE)
    SET(CMAKE_BUILD_TYPE Release)
ENDIF()

MESSAGE("Build type: " ${CMAKE_BUILD_TYPE})

# set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}  -Wall   -O3")
# set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall   -O3")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}  -w  -O3")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -w   -O3 -Wconversion-null -Wdeprecated")
set(CMAKE_C_FLAGS_RELEASE "${CMAKE_C_FLAGS_RELEASE} -march=native")
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -march=native")

set(PROJECT_ROOT_DIR "${PROJECT_SOURCE_DIR}/..")

# Check C++17 support
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++17" COMPILER_SUPPORTS_CXX17)

if(COMPILER_SUPPORTS_CXX17)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17")
    add_definitions(-DCOMPILEDWITHC11)
    message(STATUS "Using flag -std=c++17.")
else()
    message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++17 support. Please use a different C++ compiler.")
endif()

# enable folder organization in IDE
set_property(GLOBAL PROPERTY USE_FOLDERS ON)

# for libtorch
set(CMAKE_PREFIX_PATH "${PROJECT_ROOT_DIR}/Thirdparty/pytorch/torch")
find_package(Torch REQUIRED)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${TORCH_CXX_FLAGS}")
message("-- Torch Version: " ${Torch_VERSION})

# find libraries
set(OpenCV_DIR "${PROJECT_ROOT_DIR}/Thirdparty/libs/lib/cmake/opencv4")
find_package(OpenCV 4.1)
   if(NOT OpenCV_FOUND)
      message(FATAL_ERROR "OpenCV > 4.4 not found.")
   endif()
MESSAGE("OPENCV VERSION: " ${OpenCV_VERSION} " found in " ${OpenCV_INCLUDE_DIRS})

set(Eigen3_DIR "${PROJECT_ROOT_DIR}/Thirdparty/libs/share/eigen3/cmake")
find_package(Eigen3 3.1.0 REQUIRED)
message("EIGEN VERSION:  " ${EIGEN3_VERSION} " found in " ${EIGEN3_INCLUDE_DIR})

set(Pangolin_DIR "${PROJECT_ROOT_DIR}/Thirdparty/libs/lib/cmake/Pangolin")
find_package(Pangolin 0.6 REQUIRED)
message("PANGOLIN VERSION: " ${Pangolin_VERSION} " found in " ${Pangolin_INCLUDE_DIRS})

# find_package(realsense2)
include_directories(
    ${EIGEN_INCLUDE_DIR}
    ${OpenCV_INCLUDE_DIR}
    ${Pangolin_INCLUDE_DIRS}

    ${PROJECT_ROOT_DIR}
    ${PROJECT_SOURCE_DIR}
    ${PROJECT_SOURCE_DIR}/include
    ${PROJECT_SOURCE_DIR}/include/CameraModels

    ${PROJECT_ROOT_DIR}/Thirdparty
    ${PROJECT_ROOT_DIR}/Thirdparty/g2o/
    ${PROJECT_ROOT_DIR}/Thirdparty/DBoW2/
    ${PROJECT_ROOT_DIR}/Thirdparty/Sophus/
)

# Set output directory for libraries
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_ROOT_DIR}/lib/XF)

# Define common libraries that every module will link against
set(COMMON_LIBRARIES
    ${OpenCV_LIBS}
    ${Pangolin_LIBRARIES}
    ${TORCH_LIBRARIES}
    ${EIGEN3_LIBS}
    ${PROJECT_ROOT_DIR}/Thirdparty/libs/lib/libDBoW2.so
    ${PROJECT_ROOT_DIR}/Thirdparty/libs/lib/libg2o.so
    -lboost_serialization
    -lcrypto
)

# Combine all modules into a single library
add_library(${PROJECT_NAME} SHARED
    # Core Module
    src/System.cc
    src/Config.cc
    src/Settings.cc
    
    # Tracking Module
    src/Tracking.cc
    src/Frame.cc
    
    # Mapping Module
    src/LocalMapping.cc
    src/LoopClosing.cc
    
    # Map Data Module
    src/Map.cc
    src/Atlas.cc
    src/MapPoint.cc
    src/KeyFrame.cc
    src/KeyFrameDatabase.cc
    
    # Feature Module
    src/ORBextractor.cc
    src/ORBmatcher.cc
    src/XFeat.cc
    src/XFextractor.cc
    
    # Optimization Module
    src/Optimizer.cc
    src/G2oTypes.cc
    src/OptimizableTypes.cpp
    src/Sim3Solver.cc
    src/MLPnPsolver.cpp
    
    # Camera Models Module
    src/CameraModels/Pinhole.cpp
    src/CameraModels/KannalaBrandt8.cpp
    
    # Visualization Module
    src/FrameDrawer.cc
    src/MapDrawer.cc
    src/Viewer.cc
    
    # Utilities Module
    src/Converter.cc
    src/GeometricTools.cc
    src/TwoViewReconstruction.cc
    src/ImuTypes.cc
)

target_link_libraries(${PROJECT_NAME} ${COMMON_LIBRARIES})

# ########## Build Examples ###########

# RGB-D example
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_ROOT_DIR}/Examples/XF)

add_executable(rgbd_tum examples/RGB-D/rgbd_tum.cc)
target_link_libraries(rgbd_tum ${PROJECT_NAME})

# Monocular examples
add_executable(mono_tum
    examples/Monocular/mono_tum.cc)
target_link_libraries(mono_tum ${PROJECT_NAME})

add_executable(mono_kitti
    examples/Monocular/mono_kitti.cc)
target_link_libraries(mono_kitti ${PROJECT_NAME})

add_executable(mono_euroc
    examples/Monocular/mono_euroc.cc)
target_link_libraries(mono_euroc ${PROJECT_NAME})

add_executable(mono_tum_vi
    examples/Monocular/mono_tum_vi.cc)
target_link_libraries(mono_tum_vi ${PROJECT_NAME})

message(STATUS "XFeatSLAM has been compiled into a single shared library")
message(STATUS "Library will be built in: ${CMAKE_LIBRARY_OUTPUT_DIRECTORY}")
