cmake_minimum_required(VERSION 3.10)
project(ORB_SLAM3)

IF(NOT CMAKE_BUILD_TYPE)
  SET(CMAKE_BUILD_TYPE Debug)
ENDIF()

MESSAGE("Build type: " ${CMAKE_BUILD_TYPE})

# set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}  -Wall   -O3")
# set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall   -O3")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}  -O3")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3")
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
    ${PROJECT_ROOT_DIR}/Thirdparty/libs/lib/libDBoW2.so
    ${PROJECT_ROOT_DIR}/Thirdparty/libs/lib/libg2o.so
    -lboost_serialization
    -lcrypto
)

# 1. Core Module
add_library(orbslam3_core SHARED
    src/System.cc
    src/Config.cc
    src/Settings.cc
    include/System.h
    include/Config.h
    include/Settings.h
)
target_link_libraries(orbslam3_core ${COMMON_LIBRARIES})

# 2. Tracking Module
add_library(orbslam3_tracking SHARED
    src/Tracking.cc
    src/Frame.cc
    include/Tracking.h
    include/Frame.h
)
target_link_libraries(orbslam3_tracking 
    orbslam3_core
    orbslam3_features
    ${COMMON_LIBRARIES}
)

# 3. Mapping Module
add_library(orbslam3_mapping SHARED
    src/LocalMapping.cc
    src/LoopClosing.cc
    include/LocalMapping.h
    include/LoopClosing.h
)
target_link_libraries(orbslam3_mapping 
    orbslam3_core
    orbslam3_map
    orbslam3_optimizer
    ${COMMON_LIBRARIES}
)

# 4. Map Data Module
add_library(orbslam3_map SHARED
    src/Map.cc
    src/Atlas.cc
    src/MapPoint.cc
    src/KeyFrame.cc
    src/KeyFrameDatabase.cc
    include/Map.h
    include/Atlas.h
    include/MapPoint.h
    include/KeyFrame.h
    include/KeyFrameDatabase.h
)
target_link_libraries(orbslam3_map 
    orbslam3_core
    ${COMMON_LIBRARIES}
)

# 5. Feature Module
add_library(orbslam3_features SHARED
    src/ORBextractor.cc
    src/ORBmatcher.cc
    include/ORBextractor.h
    include/ORBmatcher.h
    include/ORBVocabulary.h
)
target_link_libraries(orbslam3_features 
    orbslam3_core
    ${COMMON_LIBRARIES}
)

# 6. Optimization Module
add_library(orbslam3_optimizer SHARED
    src/Optimizer.cc
    src/G2oTypes.cc
    src/OptimizableTypes.cpp
    src/Sim3Solver.cc
    src/MLPnPsolver.cpp
    include/Optimizer.h
    include/G2oTypes.h
    include/OptimizableTypes.h
    include/Sim3Solver.h
    include/MLPnPsolver.h
)
target_link_libraries(orbslam3_optimizer 
    orbslam3_core
    orbslam3_map
    ${COMMON_LIBRARIES}
)

# 7. Camera Models Module
add_library(orbslam3_cameras SHARED
    src/CameraModels/Pinhole.cpp
    src/CameraModels/KannalaBrandt8.cpp
    include/CameraModels/GeometricCamera.h
    include/CameraModels/Pinhole.h
    include/CameraModels/KannalaBrandt8.h
)
target_link_libraries(orbslam3_cameras 
    orbslam3_core
    ${COMMON_LIBRARIES}
)

# 8. Visualization Module
add_library(orbslam3_visualization SHARED
    src/FrameDrawer.cc
    src/MapDrawer.cc
    src/Viewer.cc
    include/FrameDrawer.h
    include/MapDrawer.h
    include/Viewer.h
)
target_link_libraries(orbslam3_visualization 
    orbslam3_core
    orbslam3_map
    ${COMMON_LIBRARIES}
)

# 9. Utilities Module
add_library(orbslam3_utils SHARED
    src/Converter.cc
    src/GeometricTools.cc
    src/TwoViewReconstruction.cc
    src/ImuTypes.cc
    include/Converter.h
    include/GeometricTools.h
    include/TwoViewReconstruction.h
    include/ImuTypes.h
    include/SerializationUtils.h
)
target_link_libraries(orbslam3_utils 
    orbslam3_core
    ${COMMON_LIBRARIES}
)

# Main ORB_SLAM3 meta-library that links all modules together
add_library(${PROJECT_NAME} SHARED
    # Just a thin wrapper to link all modules
    src/System.cc
)
target_link_libraries(${PROJECT_NAME}
    orbslam3_core
    orbslam3_tracking
    orbslam3_mapping
    orbslam3_map
    orbslam3_features
    orbslam3_optimizer
    orbslam3_cameras
    orbslam3_visualization
    orbslam3_utils
)

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