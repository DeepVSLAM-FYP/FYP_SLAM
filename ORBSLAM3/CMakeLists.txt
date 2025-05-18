cmake_minimum_required(VERSION 3.10)
project(ORB_SLAM3)

IF(NOT CMAKE_BUILD_TYPE)
  SET(CMAKE_BUILD_TYPE Release)
ENDIF()

MESSAGE("Build type: " ${CMAKE_BUILD_TYPE})

if(O3_DEBUG)
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -O3")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3")
else()
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -O0")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O0")
    set(CMAKE_C_FLAGS_RELEASE "${CMAKE_C_FLAGS_RELEASE} -O3")
    set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -O3")
endif()

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wunused-result")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wconversion-null -Wdeprecated -Wunused-result")
set(CMAKE_C_FLAGS_RELEASE "${CMAKE_C_FLAGS_RELEASE} -march=native")
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -march=native")
set(PROJECT_ROOT_DIR "${PROJECT_SOURCE_DIR}/..")

# Check C++17 support
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++17" COMPILER_SUPPORTS_CXX17)

if(COMPILER_SUPPORTS_CXX17)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17")
    add_definitions(-DCOMPILEDWITHC17)
    message(STATUS "Using flag -std=c++17.")
else()
    message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++17 support. Please use a different C++ compiler.")
endif()

# TODO: make the dep libraries to install .cmake files in the same directory and uncomment the following line
# LIST(APPEND CMAKE_MODULE_PATH ${PROJECT_ROOT_DIR}/cmake_modules)

# set(OpenCV_DIR "${PROJECT_ROOT_DIR}/Thirdparty/libs/lib/cmake/opencv4")
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
        ${PROJECT_ROOT_DIR}/Thirdparty/FBOW/include
        ${EIGEN3_INCLUDE_DIR}
        ${Pangolin_INCLUDE_DIRS}
        ${PROJECT_ROOT_DIR}/Thirdparty/super_point_vitis/include
)

# Add SuperPoint include directories if enabled
if(BUILD_SP_DPU)
    # include_directories(
    #     ${PROJECT_ROOT_DIR}/Thirdparty/super_point_vitis/include
    # )
    # Define a macro to enable conditional compilation
    add_definitions(-DBUILD_SP_DPU)
endif()

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

if(BUILD_SP_DPU)
    set(COMMON_LIBRARIES ${COMMON_LIBRARIES}
        ${PROJECT_ROOT_DIR}/Thirdparty/super_point_vitis/lib/libSuperPointLib.so
    )
endif()

add_library(Match_Visualizer SHARED
    src/MatchVisualizer.cc
    )
target_link_libraries(Match_Visualizer
    ${OpenCV_LIBS}
    ${EIGEN3_LIBS}
)

add_library(Feature_Extractors SHARED
    src/FeatureExtractors/ORBextractor.cc
    src/FeatureExtractors/SIFTextractor.cc
    src/FeatureExtractors/Dummy/FeatureIO.cpp
    src/FeatureExtractors/Dummy/Dummyextractor.cc
    # src/FeatureExtractors/Dummy/DummyAdapter.cpp
)

target_link_libraries( Feature_Extractors
    ${OpenCV_LIBS}
    ${EIGEN3_LIBS}
)

# Pipelined Feature Extraction
add_library(Pipelined_Feature_Extraction SHARED
    src/PipelinedFE/DummyPipelinedProcess.cc
    src/PipelinedFE/ORBPipelinedProcess.cc
    src/PipelinedFE/PipelinedProcessFactory.cc
)

# Conditionally add SuperPoint implementation if enabled
if(BUILD_SP_DPU)
    target_sources(Pipelined_Feature_Extraction PRIVATE
        src/PipelinedFE/SuperPointPipelinedProcess.cc
    )
    target_link_libraries(Pipelined_Feature_Extraction
        ${PROJECT_ROOT_DIR}/Thirdparty/super_point_vitis/lib/libSuperPointLib.so
    )
endif()

target_link_libraries(Pipelined_Feature_Extraction
    ${OpenCV_LIBS}
    ${EIGEN3_LIBS}
    Feature_Extractors
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
    src/GlobalFeatureExtractorType.cc
    
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

target_link_libraries(${PROJECT_NAME} 
    ${COMMON_LIBRARIES} 
    Feature_Extractors 
    Match_Visualizer
    Pipelined_Feature_Extraction
)

# Build examples - these link to the main library which includes all modules
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_ROOT_DIR}/Examples/ORB)



add_executable(monoORB_euroc
        Examples/Monocular/mono_euroc.cc)
target_link_libraries(monoORB_euroc ${PROJECT_NAME})


add_executable(mono_euroc_pipelined
        Examples/Monocular/mono_euroc_pipelined.cc)
target_link_libraries(mono_euroc_pipelined ${PROJECT_NAME})

add_executable(mono_euroc_pipelined_dummy
        Examples/Monocular/mono_euroc_pipelined_dummy.cc)
target_link_libraries(mono_euroc_pipelined_dummy ${PROJECT_NAME})

# Add SuperPoint real-time feature extraction executable only if BUILD_SP_DPU is enabled
if(BUILD_SP_DPU)
    add_executable(mono_euroc_superpoint
            Examples/Monocular/mono_euroc_superpoint.cc)
    target_link_libraries(mono_euroc_superpoint ${PROJECT_NAME})
    
    add_executable(ip_cam_superpoint
            Examples/Monocular/ip_cam_superpoint.cpp)
    target_link_libraries(ip_cam_superpoint ${PROJECT_NAME} -lvart-util -lvart-runner -lglog)
    
    add_executable(webcam_superpoint
            Examples/Monocular/webcam_superpoint.cpp)
    target_link_libraries(webcam_superpoint ${PROJECT_NAME} -lvart-util -lvart-runner -lglog)
    
    message(STATUS "Building SuperPoint DPU real-time feature extraction executable")
endif()

#TODO: make these examples work
# add_executable(monoORB_tum
#         Examples/Monocular/mono_tum.cc)
# target_link_libraries(monoORB_tum ${PROJECT_NAME})

# add_executable(monoORB_kitti
#         Examples/Monocular/mono_kitti.cc)
# target_link_libraries(monoORB_kitti ${PROJECT_NAME})

#note: the following examples are still not working 
# add_executable(rgbdORB_tum
#         Examples/RGB-D/rgbd_tum.cc)
# target_link_libraries(rgbdORB_tum ${PROJECT_NAME})

# add_executable(monoORB_tum_vi
#         Examples/Monocular/mono_tum_vi.cc)
# target_link_libraries(monoORB_tum_vi ${PROJECT_NAME})

# add_executable(monoORB_inertial_euroc
#         Examples/Monocular-Inertial/mono_inertial_euroc.cc)
# target_link_libraries(monoORB_inertial_euroc ${PROJECT_NAME})

# add_executable(monoORB_inertial_tum_vi
#         Examples/Monocular-Inertial/mono_inertial_tum_vi.cc)
# target_link_libraries(monoORB_inertial_tum_vi ${PROJECT_NAME})
message(STATUS "Libraries will be built in: ${CMAKE_LIBRARY_OUTPUT_DIRECTORY}")
message(STATUS "Executables will be built in: ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}")
# message(STATUS "Dummy Pipelined features will be loaded from: datasets/feature_outputs/SP_H")