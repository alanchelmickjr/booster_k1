#!/bin/bash

echo "==========================================="
echo "Testing ROS2 Camera Topics on K-1"
echo "==========================================="

# Source ROS2
source /opt/ros/humble/setup.bash

echo -e "\n1. Checking available ROS2 topics:"
ros2 topic list | grep -E "(camera|image)" || echo "No camera topics found"

echo -e "\n2. Checking booster_camera_bridge topics specifically:"
ros2 topic list | grep booster_camera_bridge

echo -e "\n3. Checking if left camera is publishing:"
timeout 2 ros2 topic hz /booster_camera_bridge/image_left_raw 2>/dev/null || echo "Left camera not publishing"

echo -e "\n4. Checking if right camera is publishing:"
timeout 2 ros2 topic hz /booster_camera_bridge/image_right_raw 2>/dev/null || echo "Right camera not publishing"

echo -e "\n5. Checking ALL topics with data rates:"
ros2 topic list -v | grep -E "(camera|image)"

echo -e "\n6. Checking camera bridge node status:"
ros2 node list | grep -i camera || echo "No camera nodes running"

echo -e "\n==========================================="