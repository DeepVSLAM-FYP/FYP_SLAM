#!/bin/bash

#store the current working directory in temporary variable
temp_dir=$(pwd)
script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$script_dir"

mkdir -p build
cd build
echo "Building Eigen 3.4.0"
echo "Configuring Eigen 3.4.0"
cmake ..
cmake --build . --target install --parallel
cd $temp_dir
echo "Eigen 3.4.0 installed successfully to {SLAM_ROOT}/ThirdParty/libs"
