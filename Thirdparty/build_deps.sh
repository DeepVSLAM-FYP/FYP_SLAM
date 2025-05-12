#!/bin/bash

# Script to build all third-party dependencies for FYP_SLAM
# Builds: DBOW2, DBoW3, FBOW, Eigen, g2o, Pangolin, OpenCV, Sophus

set -e  # Exit immediately if a command exits with a non-zero status

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# Log function for better output
log() {
    echo -e "\033[1;32m[BUILD]\033[0m $1"
}

error() {
    echo -e "\033[1;31m[ERROR]\033[0m $1"
    exit 1
}

# Function to build a dependency
build_dependency() {
    local name="$1"
    local dir="$SCRIPT_DIR/$2"
    local build_script="$dir/build.sh"
    
    log "Building $name..."
    
    if [ ! -d "$dir" ]; then
        error "Directory $dir does not exist!"
    fi
    
    if [ ! -f "$build_script" ]; then
        error "Build script not found: $build_script"
    fi
    
    cd "$dir" || error "Failed to change directory to $dir"
    source ./build.sh || error "Failed to build $name"
    
    log "$name built successfully"
}

log "Starting build process for all dependencies"

# Build each dependency
build_dependency "DBoW2" "DBoW2"
build_dependency "DBoW3" "DBoW3" 
build_dependency "FBOW" "FBOW"
build_dependency "Eigen" "eigen-3.4.0"
build_dependency "g2o" "g2o"
build_dependency "Pangolin" "Pangolin-0.6"
build_dependency "OpenCV" "opencv-4.1.0"
build_dependency "Sophus" "Sophus"

log "All dependencies built successfully!"