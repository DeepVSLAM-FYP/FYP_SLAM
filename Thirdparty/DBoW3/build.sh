#!/bin/bash
set -e # Exit on error


#store the current working directory in temporary variable
temp_dir=$(pwd)
script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$script_dir"

mkdir -p build
cd build
echo "Building DBOW3"
echo "Configuring DBOW3"
cmake ..
cmake --build . --target install --parallel
cd $temp_dir
echo "DBOW3 installed successfully to {SLAM_ROOT}/ThirdParty/libs"
