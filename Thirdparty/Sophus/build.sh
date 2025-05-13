#!/bin/bash

#store the current working directory in temporary variable
temp_dir=$(pwd)
script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$script_dir"

mkdir -p build
cd build
echo "Building Sophus"
echo "Configuring Sophus"
cmake ..
cmake --build . --target install --parallel
cd $temp_dir
echo "Sophus installed successfully to {SLAM_ROOT}/ThirdParty/libs"
