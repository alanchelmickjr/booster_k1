#!/bin/bash

echo "============================================"
echo "   K-1 Wave Detection Launcher"
echo "============================================"

# Kill any existing wave detector processes
echo "Cleaning up old processes..."
pkill -f "src/wave_detector.py" 2>/dev/null
pkill -f "SimpleHTTPServer 8080" 2>/dev/null

# Kill anything using port 8080
lsof -ti:8080 | xargs kill -9 2>/dev/null

# Wait for cleanup
sleep 2

# Change to project directory
cd /home/booster/booster_k1

# Source ROS2
echo "Setting up ROS2 environment..."
if [ -f "/opt/ros/humble/setup.bash" ]; then
    source /opt/ros/humble/setup.bash
else
    echo "❌ ROS2 setup file not found at /opt/ros/humble/setup.bash"
    echo "Please install ROS2 Humble."
    exit 1
fi

# Check if ROS2 is actually working
if ! command -v ros2 &> /dev/null; then
    echo "❌ ROS2 command not found even after sourcing."
    exit 1
fi

# Auto-detect camera topic
echo "Checking camera topics..."
CAMERA_TOPIC=""

if ros2 topic list | grep -q "/booster_camera_bridge/image_left_raw"; then
    CAMERA_TOPIC="/booster_camera_bridge/image_left_raw"
    echo "✓ Found Bridge Topic: $CAMERA_TOPIC"
elif ros2 topic list | grep -q "/image_left_raw"; then
    CAMERA_TOPIC="/image_left_raw"
    echo "⚠ Bridge not found, using Raw Topic: $CAMERA_TOPIC"
elif ros2 topic list | grep -q "/image_raw"; then
    CAMERA_TOPIC="/image_raw"
    echo "⚠ Using fallback topic: $CAMERA_TOPIC"
else
    echo "❌ No known camera topics found!"
    echo "Available topics:"
    ros2 topic list | grep image
    echo "Trying default bridge topic anyway..."
    CAMERA_TOPIC="/booster_camera_bridge/image_left_raw"
fi

# Launch the detector
echo ""
echo "Starting Wave Detector..."
echo "- Topic: $CAMERA_TOPIC"
echo "- Web interface: http://192.168.88.153:8080"
echo "- Wave at the camera to interact!"
echo "- Press Ctrl+C to stop"
echo ""

# Run the consolidated detector with the detected topic
python3 src/wave_detector.py --topic "$CAMERA_TOPIC"