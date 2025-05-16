#!/bin/bash

set -e

git pull

REMOTE_URL="root@192.248.10.70"
REMOTE_PATH_1="/mnt/kr260/root/jupyter_notebooks/Fyp/FYP_SLAM"
REMOTE_PATH_2="/mnt/sda1/FYP_2024/Ruchith/FYP_SLAM"

REMOTE_TARGET_1="$REMOTE_URL:$REMOTE_PATH_1"
REMOTE_TARGET_2="$REMOTE_URL:$REMOTE_PATH_2"
# Paths to sync (relative to project root)
# Define paths
ORBSLAM3_LIBS="lib/ORB" # ORBSLAM3 libs path
THIRDPARTY_LIBS="Thirdparty/libs/" # Thirdparty libs path
ORBSLAM3_EXAMPLES="Examples/ORB" # ORBSLAM3 executable path
BUILD_DIR="build"  # Build directory
VOCABULARY_PATH="Vocabulary"
SUPERPOINT_LIBS="Thirdparty/super_point_vitis/lib/"

# Create directories if they don't exist
mkdir -p "$ORBSLAM3_LIBS"
mkdir -p "$THIRDPARTY_LIBS"
mkdir -p "$ORBSLAM3_EXAMPLES"
mkdir -p "$BUILD_DIR"
mkdir -p "$VOCABULARY_PATH"

# Rsync and SSH options
RSYNC_OPTS="-avzP --partial --inplace"
SSH_OPTS="ssh -T -c aes128-gcm@openssh.com \
    -o Compression=no \
    -o ControlMaster=auto \
    -o ControlPath=~/.ssh/cm-%r@%h:%p \
    -o ControlPersist=600"

# Sync ORBSLAM3 built libraries
echo "Syncing ORBSLAM3 built libraries..."
rsync $RSYNC_OPTS -e "$SSH_OPTS" --delete "$REMOTE_TARGET_1/$ORBSLAM3_LIBS/" "$ORBSLAM3_LIBS/"

# Sync Thirdparty built libraries
echo "Syncing Thirdparty built libraries..."
rsync $RSYNC_OPTS -e "$SSH_OPTS" --delete "$REMOTE_TARGET_1/$THIRDPARTY_LIBS/" "$THIRDPARTY_LIBS/"

# Sync ORBSLAM3 executables
echo "Syncing ORBSLAM3 executables..."
rsync $RSYNC_OPTS -e "$SSH_OPTS" --delete "$REMOTE_TARGET_1/$ORBSLAM3_EXAMPLES/" "$ORBSLAM3_EXAMPLES/"

# Sync build folder
echo "Syncing build folder..."
rsync $RSYNC_OPTS -e "$SSH_OPTS" --delete "$REMOTE_TARGET_1/$BUILD_DIR/" "$BUILD_DIR/"

# Sync SuperPoint libs
echo "Syncing SuperPoint libs..."
rsync $RSYNC_OPTS -e "$SSH_OPTS" --delete "$REMOTE_TARGET_1/$SUPERPOINT_LIBS/" "$SUPERPOINT_LIBS/"

# Sync Vocabulary folder
# echo "Syncing Vocabulary folder..."
# rsync $RSYNC_OPTS -e "$SSH_OPTS" "$REMOTE_TARGET_2/$VOCABULARY_PATH/" "$VOCABULARY_PATH/"



echo "Deployment complete!" 