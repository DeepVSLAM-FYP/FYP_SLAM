cmake_minimum_required(VERSION 2.8)
project(DBoW3)

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}  -Wall  -O3 -march=native ")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Wall  -O3 -march=native")

set(HDRS_DBOW3
  src/BowVector.h
  src/Database.h
  src/DBoW3.h
  src/DescManip.h
  src/exports.h
  src/QueryResults.h
  src/quicklz.h
  src/timers.h
  src/FeatureVector.h
  src/ScoringObject.h   
  src/Vocabulary.h)
set(SRCS_DBOW3
  src/BowVector.cpp
  src/Database.cpp
  src/DescManip.cpp
  src/QueryResults.cpp
  src/quicklz.c
  src/FeatureVector.cpp
  src/ScoringObject.cpp   
  src/Vocabulary.cpp)

# Define custom install prefix to Thirdparty/libs
if(NOT DBOW3_INSTALL_PREFIX)
  set(DBOW3_INSTALL_PREFIX ${PROJECT_SOURCE_DIR}/../libs)
endif()
message(STATUS "DBOW3_INSTALL_PREFIX: ${DBOW3_INSTALL_PREFIX}")

# Look for OpenCV in standard system locations first, then try custom location
find_package(OpenCV 3.0 QUIET)
if(NOT OpenCV_FOUND)
  # Try custom location
  list(APPEND CMAKE_PREFIX_PATH ${PROJECT_SOURCE_DIR}/../libs)
  find_package(OpenCV 3.0 QUIET)
  if(NOT OpenCV_FOUND)
    # Try system-wide OpenCV 4 (backward compatible with 3)
    find_package(OpenCV 4.0 QUIET)
    if(NOT OpenCV_FOUND)
      message(FATAL_ERROR "OpenCV >= 3.0 not found. Please install OpenCV.")
    endif()
  endif()
endif()

message(STATUS "OpenCV directory: ${OpenCV_DIR}")
message(STATUS "OpenCV version: ${OpenCV_VERSION}")

include_directories(${OpenCV_INCLUDE_DIRS})
add_library(DBoW3 SHARED ${SRCS_DBOW3})
target_link_libraries(DBoW3 ${OpenCV_LIBS})

# ------------------------------------------------------------------
# INSTALLATION SECTION
# ------------------------------------------------------------------

# Install the shared library
install(TARGETS DBoW3
  LIBRARY DESTINATION ${DBOW3_INSTALL_PREFIX}/lib
  ARCHIVE DESTINATION ${DBOW3_INSTALL_PREFIX}/lib
  RUNTIME DESTINATION ${DBOW3_INSTALL_PREFIX}/bin
)

# Install the header files for DBoW3
install(FILES ${HDRS_DBOW3}
  DESTINATION ${DBOW3_INSTALL_PREFIX}/include/DBoW3
)
