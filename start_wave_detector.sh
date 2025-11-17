#!/bin/bash

echo "============================================"
echo "   K-1 Wave Detection Launcher"
echo "============================================"

# Kill any existing wave detector processes
echo "Cleaning up old processes..."
pkill -f "wave_detector" 2>/dev/null
pkill -f "port 8080" 2>/dev/null

# Kill anything using port 8080
lsof -ti:8080 | xargs kill -9 2>/dev/null

# Wait for cleanup
sleep 2

# Change to project directory
cd /home/booster/booster_k1

# Source ROS2
echo "Setting up ROS2 environment..."
source /opt/ros/humble/setup.bash

# Check camera topic
echo "Checking camera..."
if ros2 topic list | grep -q "/image_left_raw"; then
    echo "✓ Camera topic found"
else
    echo "⚠ Camera topic not found"
fi

# Launch the detector
echo ""
echo "Starting Wave Detector..."
echo "- Web interface: http://192.168.88.153:8080"
echo "- Wave at the camera to interact!"
echo "- Press Ctrl+C to stop"
echo ""

# Run the detector
python3 src/working_wave_detector.py