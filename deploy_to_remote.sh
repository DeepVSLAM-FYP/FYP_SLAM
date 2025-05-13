#!/bin/bash

# Usage: ./deploy_to_remote.sh user@remote_host:/remote/path

set -e

if [ $# -ne 1 ]; then
    echo "Usage: $0 user@remote_host:/remote/path"
    exit 1
fi



REMOTE_URL="root@192.248.10.70"
REMOTE_PATH_1="/mnt/kr260/root/jupyter_notebooks/Fyp/FYP_SLAM"
REMOTE_PATH_2="/mnt/sda1/FYP_2024/Ruchith/FYP_SLAM"

REMOTE_TARGET_1="$REMOTE_URL:$REMOTE_PATH_1"
REMOTE_TARGET_2="$REMOTE_URL:$REMOTE_PATH_2"
# Paths to sync (relative to project root)
ORBSLAM3_LIBS="lib/ORB" # ORBSLAM3 libs path
THIRDPARTY_LIBS="Thirdparty/libs/lib" # Thirdparty libs path
ORBSLAM3_EXAMPLES="Examples/ORB" # ORBSLAM3 executable path
BUILD_DIR="build"  # Build directory
VOCABULARY_PATH="Vocabulary"

# Rsync options
RSYNC_OPTS="-avzP --partial --inplace \
      -e \"ssh -T -c aes128-gcm@openssh.com \
          -o Compression=no \
          -o ControlMaster=auto \
          -o ControlPath=~/.ssh/cm-%r@%h:%p \
          -o ControlPersist=600\" \
      --delete"

# Sync ORBSLAM3 built libraries
if [ -d "$ORBSLAM3_LIBS" ]; then
    echo "Syncing ORBSLAM3 built libraries..."
    rsync $RSYNC_OPTS "$REMOTE_TARGET_1/$ORBSLAM3_LIBS/" "$ORBSLAM3_LIBS/"
fi

# Sync Thirdparty built libraries
if [ -d "$THIRDPARTY_LIBS" ]; then
    echo "Syncing Thirdparty built libraries..."
    rsync $RSYNC_OPTS "$REMOTE_TARGET_1/$THIRDPARTY_LIBS/" "$THIRDPARTY_LIBS/"
fi

# Sync ORBSLAM3 executables
if [ -d "$ORBSLAM3_EXAMPLES" ]; then
    echo "Syncing ORBSLAM3 executables..."
    rsync $RSYNC_OPTS "$REMOTE_TARGET_1/$ORBSLAM3_EXAMPLES/" "$ORBSLAM3_EXAMPLES/"
fi

# Sync build folder
if [ -d "$BUILD_DIR" ]; then
    echo "Syncing build folder..."
    rsync $RSYNC_OPTS "$REMOTE_TARGET_1/$BUILD_DIR/" "$BUILD_DIR/"
fi

echo "Deployment complete!" 