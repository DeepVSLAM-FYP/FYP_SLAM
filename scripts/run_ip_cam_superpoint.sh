#!/bin/bash

# Run ORB-SLAM3 in Monocular mode with real-time SuperPoint DPU feature extractor and IP camera

# Get the directory where the script is located
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"
PROJECT_ROOT="$(dirname "$SCRIPT_DIR")"

# Default parameters
MODEL_PATH="$PROJECT_ROOT/Thirdparty/super_point_vitis/compiled_SP_by_H.xmodel"
IP_ADDRESS="192.168.1.145"  # Default IP address
PORT=8080                   # Default port (changed from 8554 to 8080)
FPS=10                      # Default target FPS
CAMERA_FPS=20               # Default camera FPS
PROTOCOL="rtsp"             # Default protocol
NUM_THREADS=3               # Default number of threads (changed from 4 to 2 to match cpp default)
OUTPUT_DIR="$PROJECT_ROOT/Trajectories/ORB_SuperPoint_IP/"
SKIP_SLAM=0                 # By default, don't skip SLAM

# Parse command line arguments
while [[ $# -gt 0 ]]; do
  case $1 in
    -i|--ip)
      IP_ADDRESS="$2"
      shift 2
      ;;
    -p|--port)
      PORT="$2"
      shift 2
      ;;
    -f|--fps)
      FPS="$2"
      shift 2
      ;;
    -c|--camera-fps)
      CAMERA_FPS="$2"
      shift 2
      ;;
    -t|--threads)
      NUM_THREADS="$2"
      shift 2
      ;;
    -m|--model)
      MODEL_PATH="$2"
      shift 2
      ;;
    -o|--output)
      OUTPUT_DIR="$2"
      shift 2
      ;;
    --protocol)
      PROTOCOL="$2"
      shift 2
      ;;
    --skip-slam)
      SKIP_SLAM=1
      shift 1
      ;;
    --show-keypoints)
      export DEBUG_KeypointVisualization=1
      shift 1
      ;;
    --debug-features)
      export DEBUG_FEAT=1
      shift 1
      ;;
    -h|--help)
      echo "Usage: $0 [options]"
      echo "Options:"
      echo "  -i, --ip IP         IP address of the camera (default: 192.168.1.100)"
      echo "  -p, --port PORT     Port number (default: 8080)"
      echo "  -f, --fps FPS       Target FPS for capture and processing (default: 30)"
      echo "  -c, --camera-fps FPS Camera's internal FPS (default: 30)"
      echo "  -t, --threads NUM   Number of threads (default: 2)"
      echo "  -m, --model PATH    Path to SuperPoint model (default: compiled_SP_by_H.xmodel)"
      echo "  -o, --output DIR    Output directory for trajectories"
      echo "  --protocol PROTO    Protocol (rtsp, http) (default: rtsp)"
      echo "  --skip-slam         Skip SLAM initialization and processing (for feature extraction only)"
      echo "  --show-keypoints    Enable keypoint visualization"
      echo "  --debug-features    Enable detailed feature extraction debug output"
      echo "  -h, --help          Show this help message"
      exit 0
      ;;
    *)
      echo "Unknown option: $1"
      exit 1
      ;;
  esac
done

# Create output directory if it doesn't exist
mkdir -p "$OUTPUT_DIR"

# Create debug_output directory if it doesn't exist
mkdir -p "$PROJECT_ROOT/debug_output"

# Print current configuration
echo "Running with configuration:"
echo "- IP address: $IP_ADDRESS"
echo "- Port: $PORT"
echo "- Target FPS: $FPS"
echo "- Camera FPS: $CAMERA_FPS"
echo "- Protocol: $PROTOCOL"
echo "- Threads: $NUM_THREADS"
echo "- SuperPoint model: $MODEL_PATH"
echo "- Output directory: $OUTPUT_DIR"
echo "- Skip SLAM: $SKIP_SLAM"

# Set SKIP_SLAM environment variable if enabled
if [ "$SKIP_SLAM" -eq 1 ]; then
    export SKIP_SLAM=1
    echo "SLAM initialization and processing will be skipped"
fi

# Enable debug mode if needed
# export DEBUG_SLAM=1

# Source debug.sh if exists
if [ -f "$SCRIPT_DIR/debug.sh" ]; then
    source "$SCRIPT_DIR/debug.sh"
fi

# Check if DEBUG_KeypointVisualization is set
if [ -n "$DEBUG_KeypointVisualization" ]; then
    echo "Keypoint visualization is enabled"
fi

# Check if DEBUG_FEAT is set
if [ -n "$DEBUG_FEAT" ]; then
    echo "Feature extraction debugging is enabled"
fi

# Run the ORB-SLAM3 with SuperPoint and IP camera
"$PROJECT_ROOT/Examples/ORB/ip_cam_superpoint" \
    -t "$NUM_THREADS" \
    -o "$OUTPUT_DIR" \
    "$PROJECT_ROOT/Vocabulary/dpu_bovisa_sp_h.fbow" \
    "$PROJECT_ROOT/ORBSLAM3/Examples/Monocular/EuRoC_Dummy.yaml" \
    "$MODEL_PATH" \
    "$IP_ADDRESS" \
    "$PORT" \
    "$FPS" \
    "$PROTOCOL" \
    "$CAMERA_FPS" 2>&1 | tee "$PROJECT_ROOT/debug_output/ip_cam_sp_debug_log_$(date +%Y%m%d_%H%M%S).txt"

echo "ORB-SLAM3 Monocular IP Camera SuperPoint run completed" 