#!/bin/bash

echo "================================"
echo "ROS2 Camera Topic Diagnostics"
echo "================================"

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

# Source ROS2
echo -e "${YELLOW}[1/5] Sourcing ROS2...${NC}"
source /opt/ros/humble/setup.bash 2>/dev/null || source /opt/ros/foxy/setup.bash 2>/dev/null

if [ $? -ne 0 ]; then
    echo -e "${RED}Failed to source ROS2!${NC}"
    exit 1
fi
echo -e "${GREEN}ROS2 sourced successfully${NC}"

# List all topics
echo -e "\n${YELLOW}[2/5] Listing all ROS2 topics...${NC}"
ros2 topic list

# Find camera topics
echo -e "\n${YELLOW}[3/5] Finding camera-related topics...${NC}"
CAMERA_TOPICS=$(ros2 topic list | grep -E 'camera|image|video')

if [ -z "$CAMERA_TOPICS" ]; then
    echo -e "${RED}No camera topics found!${NC}"
    echo "Checking for any sensor_msgs topics..."
    ros2 topic list -t | grep sensor_msgs
else
    echo -e "${GREEN}Found camera topics:${NC}"
    echo "$CAMERA_TOPICS"
fi

# Check specific expected topics
echo -e "\n${YELLOW}[4/5] Checking expected booster camera topics...${NC}"
EXPECTED_TOPICS=(
    "/booster_camera_bridge/image_left_raw"
    "/booster_camera_bridge/image_right_raw"
)

for topic in "${EXPECTED_TOPICS[@]}"; do
    if ros2 topic list | grep -q "$topic"; then
        echo -e "${GREEN}✓ Found: $topic${NC}"
        
        # Get topic info
        echo "  Getting topic info..."
        ros2 topic info "$topic" --verbose
        
        # Check if topic is publishing
        echo "  Checking publication rate..."
        timeout 2 ros2 topic hz "$topic" 2>/dev/null | head -5
        
        # Echo one message
        echo "  Sample message header:"
        timeout 1 ros2 topic echo "$topic" --once 2>/dev/null | head -20
    else
        echo -e "${RED}✗ Missing: $topic${NC}"
    fi
done

# Check for any image transport topics
echo -e "\n${YELLOW}[5/5] Checking for image_transport topics...${NC}"
IMAGE_TRANSPORT_TOPICS=$(ros2 topic list | grep -E 'compressed|theora|raw')

if [ -n "$IMAGE_TRANSPORT_TOPICS" ]; then
    echo -e "${GREEN}Found image_transport topics:${NC}"
    echo "$IMAGE_TRANSPORT_TOPICS"
else
    echo -e "${YELLOW}No image_transport topics found${NC}"
fi

# Check node list for camera-related nodes
echo -e "\n${YELLOW}[BONUS] Checking for camera-related nodes...${NC}"
CAMERA_NODES=$(ros2 node list | grep -E 'camera|vision|image')

if [ -n "$CAMERA_NODES" ]; then
    echo -e "${GREEN}Found camera nodes:${NC}"
    echo "$CAMERA_NODES"
    
    # Get info on camera nodes
    for node in $CAMERA_NODES; do
        echo -e "\n${YELLOW}Node info for: $node${NC}"
        ros2 node info "$node" 2>/dev/null | head -20
    done
else
    echo -e "${RED}No camera nodes found!${NC}"
fi

# Summary
echo -e "\n================================"
echo -e "${YELLOW}DIAGNOSTIC SUMMARY${NC}"
echo -e "================================"

if [ -n "$CAMERA_TOPICS" ]; then
    echo -e "${GREEN}✓ Camera topics exist${NC}"
    
    # Test if we can get data
    echo -e "\n${YELLOW}Testing data flow from first camera topic...${NC}"
    FIRST_TOPIC=$(echo "$CAMERA_TOPICS" | head -1)
    
    echo "Testing topic: $FIRST_TOPIC"
    echo "Waiting for 3 seconds to check message rate..."
    
    MSG_COUNT=$(timeout 3 ros2 topic echo "$FIRST_TOPIC" 2>/dev/null | grep -c "header:")
    
    if [ "$MSG_COUNT" -gt 0 ]; then
        echo -e "${GREEN}✓ Receiving data! ($MSG_COUNT messages in 3 seconds)${NC}"
    else
        echo -e "${RED}✗ No data received from topic!${NC}"
        echo "Possible issues:"
        echo "1. Camera driver not publishing"
        echo "2. QoS mismatch"
        echo "3. Network issues"
    fi
else
    echo -e "${RED}✗ No camera topics found${NC}"
    echo "Possible issues:"
    echo "1. Camera driver not running"
    echo "2. ROS2 environment not properly configured"
    echo "3. Camera bridge not started"
fi

echo -e "\n================================"
echo "Diagnostics complete!"
echo "================================"