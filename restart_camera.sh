#!/bin/bash

echo "============================================"
echo "   K-1 Camera Restart Utility"
echo "============================================"
echo "Timestamp: $(date)"

# Source ROS2
source /opt/ros/humble/setup.bash

echo "1. Stopping existing camera services..."
# Kill potential camera processes
echo "123456" | sudo -S pkill -f booster-camera-bridge
echo "123456" | sudo -S pkill -f booster-video-stream
sleep 3

echo "2. Restarting ROS2 Daemon..."
ros2 daemon stop
ros2 daemon start
sleep 2

echo "3. Restarting Camera Bridge..."
BRIDGE_PATH="/opt/booster/DaemonPerception/bin/booster-camera-bridge"

if [ -f "$BRIDGE_PATH" ]; then
    echo "   Found bridge at: $BRIDGE_PATH"
    echo "   Starting bridge..."
    # Start in background with nohup, using sudo
    # We use 'sh -c' to run the nohup command under sudo
    echo "123456" | sudo -S nohup "$BRIDGE_PATH" > /dev/null 2>&1 &
    echo "   Bridge started."
else
    echo "❌ Could not find booster-camera-bridge binary at $BRIDGE_PATH"
    exit 1
fi

echo "4. Waiting for initialization (10s)..."
sleep 10

echo "5. Verifying data stream..."
if timeout 5s ros2 topic hz /booster_camera_bridge/image_left_raw > /dev/null; then
    echo "✅ SUCCESS: Data detected on /booster_camera_bridge/image_left_raw"
else
    echo "❌ FAILURE: Still no data on camera topic."
    echo "   - Check hardware connection."
    echo "   - Check if another process is hogging the camera."
fi

echo "============================================"