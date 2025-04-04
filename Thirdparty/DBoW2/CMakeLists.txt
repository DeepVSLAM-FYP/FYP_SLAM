cmake_minimum_required(VERSION 2.8)
project(DBoW2)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall -O3 -march=native")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -O3 -march=native")

set(HDRS_DBOW2
  DBoW2/BowVector.h
  DBoW2/FORB.h 
  DBoW2/FClass.h       
  DBoW2/FeatureVector.h
  DBoW2/ScoringObject.h   
  DBoW2/TemplatedVocabulary.h
)
set(SRCS_DBOW2
  DBoW2/BowVector.cpp
  DBoW2/FORB.cpp      
  DBoW2/FeatureVector.cpp
  DBoW2/ScoringObject.cpp
)

set(HDRS_DUTILS
  DUtils/Random.h
  DUtils/Timestamp.h
)
set(SRCS_DUTILS
  DUtils/Random.cpp
  DUtils/Timestamp.cpp
)

# Define custom install prefix (default to ../libs relative to the project)
if(NOT DBOW2_INSTALL_PREFIX)
  set(DBOW2_INSTALL_PREFIX ${PROJECT_SOURCE_DIR}/../libs)
endif()
message(STATUS "DBOW2_INSTALL_PREFIX: ${DBOW2_INSTALL_PREFIX}")

# Force CMake to search for OpenCV in the custom location
list(APPEND CMAKE_PREFIX_PATH ${PROJECT_SOURCE_DIR}/../libs)

find_package(OpenCV 4 QUIET)
message(STATUS "OpenCV directory: ${OpenCV_DIR}")
if(NOT OpenCV_FOUND)
  find_package(OpenCV 3.0 QUIET)
  if(NOT OpenCV_FOUND)
    message(FATAL_ERROR "OpenCV > 3.0 not found.")
  endif()
endif()

# Set the library output directory to custom install location
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

include_directories(${OpenCV_INCLUDE_DIRS})
add_library(DBoW2 SHARED ${SRCS_DBOW2} ${SRCS_DUTILS})
target_link_libraries(DBoW2 ${OpenCV_LIBS})

# ------------------------------------------------------------------
# INSTALLATION SECTION
# ------------------------------------------------------------------

# Install the shared library
install(TARGETS DBoW2
  LIBRARY DESTINATION ${DBOW2_INSTALL_PREFIX}/lib
  ARCHIVE DESTINATION ${DBOW2_INSTALL_PREFIX}/lib
  RUNTIME DESTINATION ${DBOW2_INSTALL_PREFIX}/bin
)

# Install the header files for DBoW2
install(FILES ${HDRS_DBOW2}
  DESTINATION ${DBOW2_INSTALL_PREFIX}/include/DBoW2
)

# Install the header files for DUtils
install(FILES ${HDRS_DUTILS}
  DESTINATION ${DBOW2_INSTALL_PREFIX}/include/DUtils
)
