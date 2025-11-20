#!/bin/bash

echo "============================================"
echo "   K-1 Camera Diagnostic Tool"
echo "============================================"
echo "Timestamp: $(date)"
echo ""

# Source ROS2
source /opt/ros/humble/setup.bash

echo "--- 1. Checking Active Processes ---"
if pgrep -f "booster-camera-bridge" > /dev/null; then
    echo "✅ booster-camera-bridge is RUNNING (PID: $(pgrep -f booster-camera-bridge))"
else
    echo "❌ booster-camera-bridge is NOT RUNNING"
fi

if pgrep -f "booster-video-stream" > /dev/null; then
    echo "✅ booster-video-stream is RUNNING"
else
    echo "⚠️ booster-video-stream is NOT RUNNING"
fi

echo ""
echo "--- 2. Listing All ROS2 Topics ---"
ros2 topic list

echo ""
echo "--- 3. Checking Topic Activity (5s sample) ---"
echo "Checking /booster_camera_bridge/image_left_raw..."
timeout 5s ros2 topic hz /booster_camera_bridge/image_left_raw || echo "❌ No data on /booster_camera_bridge/image_left_raw"

echo "Checking /image_left_raw..."
timeout 5s ros2 topic hz /image_left_raw || echo "❌ No data on /image_left_raw"

echo "Checking /image_raw..."
timeout 5s ros2 topic hz /image_raw || echo "❌ No data on /image_raw"

echo "Checking /camera/image_raw..."
timeout 5s ros2 topic hz /camera/image_raw || echo "❌ No data on /camera/image_raw"

echo ""
echo "--- 4. Checking USB Devices ---"
lsusb

echo ""
echo "--- 5. Checking Video Devices ---"
ls -l /dev/video*

echo ""
echo "--- 6. Recent System Logs (Camera) ---"
dmesg | grep -i -E 'camera|video|uvc' | tail -n 20

echo ""
echo "============================================"
echo "Diagnostic Complete"
echo "============================================"