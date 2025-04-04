cmake_minimum_required(VERSION 3.4)
project(Sophus VERSION 1.1.0)

include(CMakePackageConfigHelpers)
include(GNUInstallDirs)

# Set custom install prefix for Sophus if not provided
if(NOT SOPHUS_INSTALL_PREFIX)
  set(SOPHUS_INSTALL_PREFIX ${CMAKE_CURRENT_SOURCE_DIR}/../libs)
endif()
message(STATUS "SOPHUS_INSTALL_PREFIX: ${SOPHUS_INSTALL_PREFIX}")

# Optionally override CMAKE_INSTALL_PREFIX so that GNUInstallDirs variables match our custom location
set(CMAKE_INSTALL_PREFIX ${SOPHUS_INSTALL_PREFIX} CACHE INTERNAL "Installation Prefix" FORCE)

# Release by default (use -DCMAKE_BUILD_TYPE=Debug to override)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

set(CMAKE_CXX_STANDARD 11)

# Compiler settings for different compilers
if("${CMAKE_CXX_COMPILER_ID}" STREQUAL "Clang")
   set(CMAKE_CXX_FLAGS_DEBUG  "-O0 -g")
   set(CMAKE_CXX_FLAGS_RELEASE "-O3")
   set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Werror -Wextra -Wno-deprecated-register -Qunused-arguments -fcolor-diagnostics")
elseif("${CMAKE_CXX_COMPILER_ID}" STREQUAL "GNU")
   set(CMAKE_CXX_FLAGS_DEBUG  "-O0 -g")
   set(CMAKE_CXX_FLAGS_RELEASE "-O3")
   set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Werror -Wextra -std=c++11 -Wno-deprecated-declarations -ftemplate-backtrace-limit=0")
   set(CMAKE_CXX_FLAGS_COVERAGE "${CMAKE_CXX_FLAGS_DEBUG} --coverage -fno-inline -fno-inline-small-functions -fno-default-inline")
   set(CMAKE_EXE_LINKER_FLAGS_COVERAGE "${CMAKE_EXE_LINKER_FLAGS_DEBUG} --coverage")
   set(CMAKE_SHARED_LINKER_FLAGS_COVERAGE "${CMAKE_SHARED_LINKER_FLAGS_DEBUG} --coverage")
elseif(CMAKE_CXX_COMPILER_ID MATCHES "^MSVC$")
   add_definitions("-D _USE_MATH_DEFINES /bigobj /wd4305 /wd4244 /MP")
endif()

# Add local path for finding packages
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake_modules")

# Find Eigen3 from the custom location (../libs)
list(APPEND CMAKE_PREFIX_PATH "${CMAKE_CURRENT_SOURCE_DIR}/../libs")
find_package(Eigen3 3.3.0 REQUIRED)
message(STATUS "Eigen3_INCLUDE_DIR: ${Eigen3_INCLUDE_DIR}")

# Define the interface library target
add_library(sophus INTERFACE)

set(SOPHUS_HEADER_FILES
  sophus/average.hpp
  sophus/common.hpp
  sophus/geometry.hpp
  sophus/interpolate.hpp
  sophus/interpolate_details.hpp
  sophus/num_diff.hpp
  sophus/rotation_matrix.hpp
  sophus/rxso2.hpp
  sophus/rxso3.hpp
  sophus/se2.hpp
  sophus/se3.hpp
  sophus/sim2.hpp
  sophus/sim3.hpp
  sophus/sim_details.hpp
  sophus/so2.hpp
  sophus/so3.hpp
  sophus/types.hpp
  sophus/velocities.hpp
  sophus/formatstring.hpp
)

set(SOPHUS_OTHER_FILES
  sophus/test_macros.hpp
  sophus/example_ensure_handler.cpp
)

if(MSVC)
  target_compile_definitions(sophus INTERFACE _USE_MATH_DEFINES)
endif()

# Add Eigen dependency to our interface target
if(TARGET Eigen3::Eigen)
  target_link_libraries(sophus INTERFACE Eigen3::Eigen)
  set(Eigen3_DEPENDENCY "find_dependency (Eigen3 ${Eigen3_VERSION})")
else()
  target_include_directories(sophus SYSTEM INTERFACE ${Eigen3_INCLUDE_DIR})
endif()

# Specify include directories for the target: for building and for install interface.
target_include_directories(sophus INTERFACE
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>"
    "$<INSTALL_INTERFACE:include>"
)

# Declare that Sophus uses C++11 features
target_compile_features(sophus INTERFACE
  cxx_auto_type
  cxx_decltype
  cxx_nullptr
  cxx_right_angle_brackets
  cxx_variadic_macros
  cxx_variadic_templates
)

# Add sources as a custom target (to show up in IDEs)
add_custom_target(other SOURCES ${SOPHUS_OTHER_FILES})

# Option to build tests and examples (off by default)
option(BUILD_TESTS "Build tests." OFF)
if(BUILD_TESTS)
    enable_testing()
    add_subdirectory(test)
endif()

option(BUILD_EXAMPLES "Build examples." OFF)
if(BUILD_EXAMPLES)
    add_subdirectory(examples)
endif()

# --- Package export configuration ---

# Place CMake export files in our custom install location
set(SOPHUS_CMAKE_EXPORT_DIR ${SOPHUS_INSTALL_PREFIX}/cmake/sophus)
set_target_properties(sophus PROPERTIES EXPORT_NAME Sophus)

install(TARGETS sophus EXPORT SophusTargets)
install(EXPORT SophusTargets
  NAMESPACE Sophus::
  DESTINATION ${SOPHUS_CMAKE_EXPORT_DIR}
)

export(TARGETS sophus NAMESPACE Sophus:: FILE SophusTargets.cmake)
export(PACKAGE Sophus)

configure_package_config_file(
  SophusConfig.cmake.in
  ${CMAKE_CURRENT_BINARY_DIR}/SophusConfig.cmake
  INSTALL_DESTINATION ${SOPHUS_CMAKE_EXPORT_DIR}
  NO_CHECK_REQUIRED_COMPONENTS_MACRO
)

write_basic_package_version_file(
  ${CMAKE_CURRENT_BINARY_DIR}/SophusConfigVersion.cmake
  VERSION ${PROJECT_VERSION}
  COMPATIBILITY SameMajorVersion
)

install(
  FILES ${CMAKE_CURRENT_BINARY_DIR}/SophusConfig.cmake
        ${CMAKE_CURRENT_BINARY_DIR}/SophusConfigVersion.cmake
  DESTINATION ${SOPHUS_CMAKE_EXPORT_DIR}
)

# --- Install header files ---
install(
  FILES ${SOPHUS_HEADER_FILES}
  DESTINATION ${SOPHUS_INSTALL_PREFIX}/include/sophus
)
