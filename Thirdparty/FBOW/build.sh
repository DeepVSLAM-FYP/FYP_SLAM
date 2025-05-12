#!/bin/bash
set -e # Exit on error

# Store the current working directory in temporary variable
temp_dir=$(pwd)
script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$script_dir"

mkdir -p build
cd build
echo "Building FBOW"
echo "Configuring FBOW"
cmake .. -DCMAKE_BUILD_TYPE=RELEASE
cmake --build . --target install --parallel
cd $temp_dir
echo "FBOW installed successfully to {SLAM_ROOT}/ThirdParty/libs"