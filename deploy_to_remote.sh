#!/bin/bash

# Usage: ./deploy_to_remote.sh user@remote_host:/remote/path

set -e

if [ $# -ne 1 ]; then
    echo "Usage: $0 user@remote_host:/remote/path"
    exit 1
fi

REMOTE_TARGET="root@192.248.10.70:/mnt/kr260/root/jupyter_notebooks/Fyp/FYP_SLAM"

# Paths to sync (relative to project root)
ORBSLAM3_LIBS="lib/ORB"
THIRDPARTY_LIBS="Thirdparty/libs/lib"
ORBSLAM3_EXAMPLES="Examples/ORB"
BUILD_DIR="build"  # Change if your build dir is named differently

# Rsync options
RSYNC_OPTS="-avz --progress --delete"

# Sync ORBSLAM3 built libraries
if [ -d "$ORBSLAM3_LIBS" ]; then
    echo "Syncing ORBSLAM3 built libraries..."
    rsync $RSYNC_OPTS "$REMOTE_TARGET/$ORBSLAM3_LIBS/" "$ORBSLAM3_LIBS/"
fi

# Sync Thirdparty built libraries
if [ -d "$THIRDPARTY_LIBS" ]; then
    echo "Syncing Thirdparty built libraries..."
    rsync $RSYNC_OPTS "$REMOTE_TARGET/$THIRDPARTY_LIBS/" "$THIRDPARTY_LIBS/"
fi

# Sync ORBSLAM3 executables
if [ -d "$ORBSLAM3_EXAMPLES" ]; then
    echo "Syncing ORBSLAM3 executables..."
    rsync $RSYNC_OPTS "$REMOTE_TARGET/$ORBSLAM3_EXAMPLES/" "$ORBSLAM3_EXAMPLES/"
fi

# Sync build folder
if [ -d "$BUILD_DIR" ]; then
    echo "Syncing build folder..."
    rsync $RSYNC_OPTS "$REMOTE_TARGET/$BUILD_DIR/" "$BUILD_DIR/"
fi

echo "Deployment complete!" 