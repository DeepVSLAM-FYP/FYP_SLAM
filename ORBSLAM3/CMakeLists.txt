cmake_minimum_required(VERSION 3.10)
project(ORB_SLAM3)

IF(NOT CMAKE_BUILD_TYPE)
  SET(CMAKE_BUILD_TYPE Debug)
ENDIF()

MESSAGE("Build type: " ${CMAKE_BUILD_TYPE})

# set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}  -Wall   -O3")
# set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall   -O3")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}  -O3")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -Wconversion-null -Wdeprecated")
set(CMAKE_C_FLAGS_RELEASE "${CMAKE_C_FLAGS_RELEASE} -march=native")
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -march=native")

set(PROJECT_ROOT_DIR "${PROJECT_SOURCE_DIR}/..")

# Check C++11 or C++0x support
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++14" COMPILER_SUPPORTS_CXX14)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
if(COMPILER_SUPPORTS_CXX14)
   set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14")
   add_definitions(-DCOMPILEDWITHC14)
   message(STATUS "Using flag -std=c++14.")
elseif(COMPILER_SUPPORTS_CXX11)
   set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
   add_definitions(-DCOMPILEDWITHC11)
   message(STATUS "Using flag -std=c++11.")
else()
   message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++14 support. Please use a different C++ compiler.")
endif()


# TODO: make the dep libraries to install .cmake files in the same directory and uncomment the following line
# LIST(APPEND CMAKE_MODULE_PATH ${PROJECT_ROOT_DIR}/cmake_modules)

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

include_directories(
        ${PROJECT_ROOT_DIR}
        ${PROJECT_SOURCE_DIR}
        ${PROJECT_SOURCE_DIR}/include
        ${PROJECT_SOURCE_DIR}/include/CameraModels
        ${PROJECT_SOURCE_DIR}/include/FeatureExtractors
        ${PROJECT_ROOT_DIR}/Thirdparty/Sophus
        ${EIGEN3_INCLUDE_DIR}
        ${Pangolin_INCLUDE_DIRS}
)

# Set output directory for libraries
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_ROOT_DIR}/lib/ORB)

# Define common libraries that every module will link against
set(COMMON_LIBRARIES
    ${OpenCV_LIBS}
    ${EIGEN3_LIBS}
    ${Pangolin_LIBRARIES}
#     ${PROJECT_ROOT_DIR}/Thirdparty/libs/lib/libDBoW2.so
    ${PROJECT_ROOT_DIR}/Thirdparty/libs/lib/libfbow.so
    ${PROJECT_ROOT_DIR}/Thirdparty/libs/lib/libg2o.so
    -lboost_serialization
    -lcrypto
)

add_library(Feature_Extractors SHARED
    src/FeatureExtractors/ORBextractor.cc
)

target_link_libraries( Feature_Extractors
    ${OpenCV_LIBS}
    ${EIGEN3_LIBS}
)

# Combine all source files into a single library
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
#     src/ORBextractor.cc
    src/ORBmatcher.cc
    
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

target_link_libraries(${PROJECT_NAME} ${COMMON_LIBRARIES} Feature_Extractors)

# Build examples - these link to the main library which includes all modules
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_ROOT_DIR}/Examples/ORB)

add_executable(rgbdORB_tum
        Examples/RGB-D/rgbd_tum.cc)
target_link_libraries(rgbdORB_tum ${PROJECT_NAME})

add_executable(monoORB_tum
        Examples/Monocular/mono_tum.cc)
target_link_libraries(monoORB_tum ${PROJECT_NAME})

add_executable(monoORB_kitti
        Examples/Monocular/mono_kitti.cc)
target_link_libraries(monoORB_kitti ${PROJECT_NAME})

add_executable(monoORB_euroc
        Examples/Monocular/mono_euroc.cc)
target_link_libraries(monoORB_euroc ${PROJECT_NAME})

add_executable(monoORB_tum_vi
        Examples/Monocular/mono_tum_vi.cc)
target_link_libraries(monoORB_tum_vi ${PROJECT_NAME})

add_executable(monoORB_inertial_euroc
        Examples/Monocular-Inertial/mono_inertial_euroc.cc)
target_link_libraries(monoORB_inertial_euroc ${PROJECT_NAME})

add_executable(monoORB_inertial_tum_vi
        Examples/Monocular-Inertial/mono_inertial_tum_vi.cc)
target_link_libraries(monoORB_inertial_tum_vi ${PROJECT_NAME})
message(STATUS "Libraries will be built in: ${CMAKE_LIBRARY_OUTPUT_DIRECTORY}")