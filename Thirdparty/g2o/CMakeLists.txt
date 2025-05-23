cmake_minimum_required(VERSION 2.6)
set(CMAKE_LEGACY_CYGWIN_WIN32 0)

project(g2o)

set(g2o_C_FLAGS)
set(g2o_CXX_FLAGS)

# Default build type
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()
message(STATUS "BUILD TYPE:" ${CMAKE_BUILD_TYPE})

set(G2O_LIB_TYPE SHARED)

# Workaround for MSVC8 issues with Eigen
if(MSVC90)
  add_definitions(-DEIGEN_DONT_ALIGN_STATICALLY=1)
  message(STATUS "Disabling memory alignment for MSVC8")
endif(MSVC90)

# Define our custom install prefix
if(NOT G2O_INSTALL_PREFIX)
  set(G2O_INSTALL_PREFIX ${CMAKE_CURRENT_SOURCE_DIR}/../libs)
endif()
message(STATUS "G2O_INSTALL_PREFIX: ${G2O_INSTALL_PREFIX}")

# Set the output directories for libraries and executables
set(g2o_LIBRARY_OUTPUT_DIRECTORY "${G2O_INSTALL_PREFIX}/lib")
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${g2o_LIBRARY_OUTPUT_DIRECTORY})
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${g2o_LIBRARY_OUTPUT_DIRECTORY})
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${G2O_INSTALL_PREFIX}/bin")

set(G2O_SOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR})
MESSAGE(STATUS "G2O_SOURCE_DIR: ${G2O_SOURCE_DIR}")
# Append custom module path (for SuiteSparse, QGLViewer, Eigen3, etc.)
list(APPEND CMAKE_MODULE_PATH ${g2o_SOURCE_DIR}/cmake_modules)

# Detect OS and add appropriate definitions
if(UNIX)
  add_definitions(-DUNIX)
  message(STATUS "Compiling on Unix")
endif(UNIX)

# Optional OpenMP support (currently turned off by default)
find_package(OpenMP)
set(G2O_USE_OPENMP OFF CACHE BOOL "Build g2o with OpenMP support (EXPERIMENTAL)")
if(OPENMP_FOUND AND G2O_USE_OPENMP)
  set(G2O_OPENMP 1)
  set(g2o_C_FLAGS "${g2o_C_FLAGS} ${OpenMP_C_FLAGS}")
  set(g2o_CXX_FLAGS "${g2o_CXX_FLAGS} -DEIGEN_DONT_PARALLELIZE ${OpenMP_CXX_FLAGS}")
  message(STATUS "Compiling with OpenMP support")
endif()

# Compiler flags for release builds
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -O3 -march=native")
set(CMAKE_C_FLAGS_RELEASE "${CMAKE_C_FLAGS_RELEASE} -O3 -march=native")

# Activate warnings
set(g2o_C_FLAGS "${g2o_C_FLAGS}")
set(g2o_CXX_FLAGS "${g2o_CXX_FLAGS}")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${g2o_CXX_FLAGS}")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${g2o_C_FLAGS}")

# Find Eigen3 (ensure CMake looks in ../libs first)
list(APPEND CMAKE_PREFIX_PATH "${CMAKE_CURRENT_SOURCE_DIR}/../libs")
set(EIGEN3_INCLUDE_DIR ${G2O_EIGEN3_INCLUDE})
find_package(Eigen3 3.1.0 REQUIRED)
if(Eigen3_FOUND)
  set(G2O_EIGEN3_INCLUDE ${EIGEN3_INCLUDE_DIR} CACHE PATH "Directory of Eigen3")
else()
  set(G2O_EIGEN3_INCLUDE "" CACHE PATH "Directory of Eigen3")
endif()
message(STATUS "Eigen3 include directory: ${G2O_EIGEN3_INCLUDE}")

# Generate config.h from config.h.in
set(G2O_CXX_COMPILER "${CMAKE_CXX_COMPILER_ID} ${CMAKE_CXX_COMPILER}")
configure_file(config.h.in ${g2o_SOURCE_DIR}/config.h)

# Set up top-level include directories
include_directories(
  ${g2o_SOURCE_DIR}/g2o/core
  ${g2o_SOURCE_DIR}/g2o/types
  ${g2o_SOURCE_DIR}/g2o/stuff 
  ${G2O_EIGEN3_INCLUDE}
)

# Build the g2o library
add_library(g2o ${G2O_LIB_TYPE}
  # types
  g2o/types/types_sba.h
  g2o/types/types_six_dof_expmap.h
  g2o/types/types_sba.cpp
  g2o/types/types_six_dof_expmap.cpp
  g2o/types/types_seven_dof_expmap.cpp
  g2o/types/types_seven_dof_expmap.h
  g2o/types/se3quat.h
  g2o/types/se3_ops.h
  g2o/types/se3_ops.hpp
  # core
  g2o/core/base_edge.h
  g2o/core/base_binary_edge.h
  g2o/core/hyper_graph_action.cpp
  g2o/core/base_binary_edge.hpp
  g2o/core/hyper_graph_action.h
  g2o/core/base_multi_edge.h           
  g2o/core/hyper_graph.cpp
  g2o/core/base_multi_edge.hpp         
  g2o/core/hyper_graph.h
  g2o/core/base_unary_edge.h          
  g2o/core/linear_solver.h
  g2o/core/base_unary_edge.hpp         
  g2o/core/marginal_covariance_cholesky.cpp
  g2o/core/base_vertex.h               
  g2o/core/marginal_covariance_cholesky.h
  g2o/core/base_vertex.hpp             
  g2o/core/matrix_structure.cpp
  g2o/core/batch_stats.cpp             
  g2o/core/matrix_structure.h
  g2o/core/batch_stats.h               
  g2o/core/openmp_mutex.h
  g2o/core/block_solver.h              
  g2o/core/block_solver.hpp            
  g2o/core/parameter.cpp               
  g2o/core/parameter.h                 
  g2o/core/cache.cpp                    
  g2o/core/cache.h
  g2o/core/optimizable_graph.cpp       
  g2o/core/optimizable_graph.h         
  g2o/core/solver.cpp                  
  g2o/core/solver.h
  g2o/core/creators.h                 
  g2o/core/optimization_algorithm_factory.cpp
  g2o/core/estimate_propagator.cpp     
  g2o/core/optimization_algorithm_factory.h
  g2o/core/estimate_propagator.h       
  g2o/core/factory.cpp                 
  g2o/core/optimization_algorithm_property.h
  g2o/core/factory.h                   
  g2o/core/sparse_block_matrix.h
  g2o/core/sparse_optimizer.cpp  
  g2o/core/sparse_block_matrix.hpp
  g2o/core/sparse_optimizer.h
  g2o/core/hyper_dijkstra.cpp 
  g2o/core/hyper_dijkstra.h
  g2o/core/parameter_container.cpp     
  g2o/core/parameter_container.h
  g2o/core/optimization_algorithm.cpp 
  g2o/core/optimization_algorithm.h
  g2o/core/optimization_algorithm_with_hessian.cpp 
  g2o/core/optimization_algorithm_with_hessian.h
  g2o/core/optimization_algorithm_levenberg.cpp 
  g2o/core/optimization_algorithm_levenberg.h
  g2o/core/optimization_algorithm_gauss_newton.cpp 
  g2o/core/optimization_algorithm_gauss_newton.h
  g2o/core/jacobian_workspace.cpp 
  g2o/core/jacobian_workspace.h
  g2o/core/robust_kernel.cpp 
  g2o/core/robust_kernel.h
  g2o/core/robust_kernel_factory.cpp
  g2o/core/robust_kernel_factory.h
  g2o/core/robust_kernel_impl.cpp 
  g2o/core/robust_kernel_impl.h
  # stuff
  g2o/stuff/string_tools.h
  g2o/stuff/color_macros.h 
  g2o/stuff/macros.h
  g2o/stuff/timeutil.cpp
  g2o/stuff/misc.h
  g2o/stuff/timeutil.h
  g2o/stuff/os_specific.c    
  g2o/stuff/os_specific.h
  g2o/stuff/string_tools.cpp
  g2o/stuff/property.cpp       
  g2o/stuff/property.h       
)

# -----------------------------
# INSTALLATION SECTION
# -----------------------------

# Install the g2o shared library
install(TARGETS g2o
  LIBRARY DESTINATION ${G2O_INSTALL_PREFIX}/lib
  ARCHIVE DESTINATION ${G2O_INSTALL_PREFIX}/lib
  RUNTIME DESTINATION ${G2O_INSTALL_PREFIX}/bin
)

# Install header files from the core, types, and stuff directories
install(DIRECTORY ${g2o_SOURCE_DIR}/g2o/core/ DESTINATION ${G2O_INSTALL_PREFIX}/include/g2o/core
  FILES_MATCHING PATTERN "*.h" PATTERN "*.hpp"
)
install(DIRECTORY ${g2o_SOURCE_DIR}/g2o/types/ DESTINATION ${G2O_INSTALL_PREFIX}/include/g2o/types
  FILES_MATCHING PATTERN "*.h" PATTERN "*.hpp"
)
install(DIRECTORY ${g2o_SOURCE_DIR}/g2o/stuff/ DESTINATION ${G2O_INSTALL_PREFIX}/include/g2o/stuff
  FILES_MATCHING PATTERN "*.h" PATTERN "*.hpp"
)

# Optionally install the generated config.h file if needed by users
install(FILES ${g2o_SOURCE_DIR}/config.h DESTINATION ${G2O_INSTALL_PREFIX}/include/g2o)
