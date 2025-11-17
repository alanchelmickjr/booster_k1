#!/bin/bash

# Camera Feed Diagnostic Suite for K1 Robot
# This script deploys diagnostic tools and runs comprehensive tests

ROBOT_IP="192.168.88.153"
ROBOT_USER="booster"
ROBOT_PASS="123456"
ROBOT_PATH="/home/booster/camera_diagnostics"

# Colors
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m'

echo -e "${BLUE}================================${NC}"
echo -e "${BLUE}   K1 Camera Diagnostics Suite  ${NC}"
echo -e "${BLUE}================================${NC}"

# Step 1: Deploy diagnostic files
echo -e "\n${YELLOW}[1/5] Deploying diagnostic files to robot...${NC}"

# Create directory on robot
sshpass -p ${ROBOT_PASS} ssh -o StrictHostKeyChecking=no ${ROBOT_USER}@${ROBOT_IP} "mkdir -p ${ROBOT_PATH}"

# Copy files
sshpass -p ${ROBOT_PASS} scp -o StrictHostKeyChecking=no src/came_yolo.py ${ROBOT_USER}@${ROBOT_IP}:${ROBOT_PATH}/
sshpass -p ${ROBOT_PASS} scp -o StrictHostKeyChecking=no test_camera_diagnostics.py ${ROBOT_USER}@${ROBOT_IP}:${ROBOT_PATH}/
sshpass -p ${ROBOT_PASS} scp -o StrictHostKeyChecking=no test_ros2_camera.sh ${ROBOT_USER}@${ROBOT_IP}:${ROBOT_PATH}/

if [ $? -eq 0 ]; then
    echo -e "${GREEN}✓ Files deployed successfully${NC}"
else
    echo -e "${RED}✗ Failed to deploy files${NC}"
    exit 1
fi

# Step 2: Run ROS2 topic diagnostics
echo -e "\n${YELLOW}[2/5] Running ROS2 topic diagnostics...${NC}"
echo -e "${BLUE}----------------------------------------${NC}"

sshpass -p ${ROBOT_PASS} ssh -o StrictHostKeyChecking=no ${ROBOT_USER}@${ROBOT_IP} << 'EOF'
source /opt/ros/humble/setup.bash 2>/dev/null || source /opt/ros/foxy/setup.bash 2>/dev/null
cd /home/booster/camera_diagnostics

echo "=== ROS2 Topic List ==="
ros2 topic list | grep -E 'camera|image' || echo "No camera topics found"

echo -e "\n=== Checking Booster Camera Bridge ==="
if ros2 topic list | grep -q "booster_camera_bridge"; then
    echo "✓ Booster camera bridge topics found:"
    ros2 topic list | grep booster_camera_bridge
    
    # Check topic info
    echo -e "\n=== Topic Info ==="
    ros2 topic info /booster_camera_bridge/image_left_raw --verbose 2>/dev/null || echo "Could not get topic info"
    
    # Check publication rate
    echo -e "\n=== Publication Rate ==="
    timeout 2 ros2 topic hz /booster_camera_bridge/image_left_raw 2>/dev/null || echo "No messages being published"
else
    echo "✗ Booster camera bridge topics NOT found"
fi

echo -e "\n=== Camera Nodes ==="
ros2 node list | grep -E 'camera|booster' || echo "No camera nodes found"
EOF

# Step 3: Run Python diagnostic tool
echo -e "\n${YELLOW}[3/5] Running Python camera diagnostics...${NC}"
echo -e "${BLUE}----------------------------------------${NC}"

sshpass -p ${ROBOT_PASS} ssh -o StrictHostKeyChecking=no ${ROBOT_USER}@${ROBOT_IP} << 'EOF'
source /opt/ros/humble/setup.bash 2>/dev/null || source /opt/ros/foxy/setup.bash 2>/dev/null
cd /home/booster/camera_diagnostics

echo "Starting diagnostic node (10 seconds)..."
timeout 10 python3 test_camera_diagnostics.py 2>&1 | grep -E "\[DIAG\]|\[SUMMARY\]"
EOF

# Step 4: Test camera feed with our script
echo -e "\n${YELLOW}[4/5] Testing camera feed with YOLO script...${NC}"
echo -e "${BLUE}----------------------------------------${NC}"

sshpass -p ${ROBOT_PASS} ssh -o StrictHostKeyChecking=no ${ROBOT_USER}@${ROBOT_IP} << 'EOF'
source /opt/ros/humble/setup.bash 2>/dev/null || source /opt/ros/foxy/setup.bash 2>/dev/null
cd /home/booster/camera_diagnostics

echo "Starting came_yolo.py (10 seconds)..."
timeout 10 python3 came_yolo.py 2>&1 | grep -E "\[INIT\]|\[MAIN\]|\[DIAGNOSTICS\]|\[CALLBACK\]|ERROR|WARNING"
EOF

# Step 5: Check system services
echo -e "\n${YELLOW}[5/5] Checking camera-related services...${NC}"
echo -e "${BLUE}----------------------------------------${NC}"

sshpass -p ${ROBOT_PASS} ssh -o StrictHostKeyChecking=no ${ROBOT_USER}@${ROBOT_IP} << 'EOF'
echo "=== Booster Services ==="
systemctl status booster_bridge_service 2>/dev/null | head -10 || echo "booster_bridge_service not found"

echo -e "\n=== Camera Processes ==="
ps aux | grep -E 'camera|booster|ros2' | grep -v grep | head -5

echo -e "\n=== Check for camera device ==="
ls -la /dev/video* 2>/dev/null || echo "No video devices found"

echo -e "\n=== V4L2 devices ==="
v4l2-ctl --list-devices 2>/dev/null || echo "v4l2-ctl not available"
EOF

# Generate diagnosis report
echo -e "\n${BLUE}================================${NC}"
echo -e "${BLUE}      DIAGNOSIS REPORT          ${NC}"
echo -e "${BLUE}================================${NC}"

sshpass -p ${ROBOT_PASS} ssh -o StrictHostKeyChecking=no ${ROBOT_USER}@${ROBOT_IP} << 'EOF'
source /opt/ros/humble/setup.bash 2>/dev/null || source /opt/ros/foxy/setup.bash 2>/dev/null

echo -e "\n🔍 DIAGNOSTIC FINDINGS:\n"

# Check 1: ROS2 topics
if ros2 topic list | grep -q "booster_camera_bridge"; then
    echo "✅ Camera topics exist"
    
    # Check if publishing
    MSG_COUNT=$(timeout 2 ros2 topic echo /booster_camera_bridge/image_left_raw 2>/dev/null | grep -c "header:" || echo "0")
    if [ "$MSG_COUNT" != "0" ]; then
        echo "✅ Camera is publishing data"
    else
        echo "❌ Camera topics exist but NOT publishing data"
        echo "   → Issue: Camera driver may not be running"
    fi
else
    echo "❌ Camera topics do NOT exist"
    echo "   → Issue: Booster camera bridge not running"
fi

# Check 2: Services
if systemctl is-active booster_bridge_service >/dev/null 2>&1; then
    echo "✅ Booster bridge service is running"
else
    echo "❌ Booster bridge service is NOT running"
    echo "   → Fix: sudo systemctl start booster_bridge_service"
fi

# Check 3: Camera device
if ls /dev/video* >/dev/null 2>&1; then
    echo "✅ Camera devices found in /dev/"
else
    echo "❌ No camera devices in /dev/"
    echo "   → Issue: Camera hardware not detected"
fi

echo -e "\n📋 RECOMMENDED ACTIONS:\n"

# Provide specific recommendations
if ! ros2 topic list | grep -q "booster_camera_bridge"; then
    echo "1. Start the camera bridge:"
    echo "   sudo systemctl restart booster_bridge_service"
    echo "   OR"
    echo "   ros2 run booster_camera_bridge camera_bridge_node"
elif ! timeout 2 ros2 topic echo /booster_camera_bridge/image_left_raw 2>/dev/null | grep -q "header:"; then
    echo "1. Check QoS settings - mismatch between publisher and subscriber"
    echo "2. Restart camera services:"
    echo "   sudo systemctl restart booster_bridge_service"
    echo "3. Check camera hardware connection"
else
    echo "Camera infrastructure appears to be working."
    echo "The issue may be in the application code (YOLO processing, etc.)"
fi
EOF

echo -e "\n${BLUE}================================${NC}"
echo -e "${GREEN}Diagnostics complete!${NC}"
echo -e "${BLUE}================================${NC}"

echo -e "\n${YELLOW}To view camera feed manually, SSH to robot and run:${NC}"
echo "source /opt/ros/humble/setup.bash"
echo "ros2 topic echo /booster_camera_bridge/image_left_raw --once"