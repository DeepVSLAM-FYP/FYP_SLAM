# cmake/kr260-toolchain.cmake
set(CMAKE_SYSTEM_NAME      Linux)
set(CMAKE_SYSTEM_PROCESSOR aarch64)

# Set compilers (adjust if using cross-compilers)
set(CMAKE_C_COMPILER   /mnt/kr260/usr/bin/aarch64-linux-gnu-gcc)
set(CMAKE_CXX_COMPILER /mnt/kr260/usr/bin/aarch64-linux-gnu-g++)

# Toolchain file for cross-compiling with /mnt/kr260 as sysroot
set(CMAKE_SYSROOT /mnt/kr260)
set(CMAKE_FIND_ROOT_PATH /mnt/kr260)

# Add sysroot flags
# set(CMAKE_C_FLAGS   "--sysroot=/mnt/kr260 ${CMAKE_C_FLAGS}")
# set(CMAKE_CXX_FLAGS "--sysroot=/mnt/kr260 ${CMAKE_CXX_FLAGS}")

# Search for programs in the build host directories
set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
# Search for libraries, headers, and packages only in the sysroot
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_PACKAGE ONLY) 