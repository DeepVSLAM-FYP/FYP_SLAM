#!/bin/bash

#store the current working directory in temporary variable
temp_dir=$(pwd)
script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$script_dir"

mkdir -p build
cd build
echo "Building G2O"
echo "Configuring G2O"
rm -rf ./*
cmake ..
cmake --build . --target install --parallel
cd $temp_dir
echo "G2O installed successfully to {SLAM_ROOT}/ThirdParty/libs"
