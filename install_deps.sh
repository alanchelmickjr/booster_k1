#!/bin/bash

echo "============================================"
echo "   K-1 Dependency Installer"
echo "============================================"

# Source ROS2 to check what we have
if [ -f "/opt/ros/humble/setup.bash" ]; then
    source /opt/ros/humble/setup.bash
else
    echo "⚠️ ROS2 Humble not found. Attempting to add repository..."
    echo "123456" | sudo -S apt install software-properties-common -y
    echo "123456" | sudo -S add-apt-repository universe -y
    echo "123456" | sudo -S apt update && sudo apt install curl -y
    echo "123456" | sudo -S curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
    echo "123456" | sudo -S apt update
    echo "123456" | sudo -S apt install ros-humble-ros-base -y
    source /opt/ros/humble/setup.bash
fi

echo "1. Installing System Dependencies..."
# Install critical ROS2 packages for camera and bridging
echo "123456" | sudo -S apt-get update
echo "123456" | sudo -S apt-get install -y \
    ros-humble-cv-bridge \
    ros-humble-image-transport \
    ros-humble-image-transport-plugins \
    ros-humble-v4l2-camera \
    ros-humble-camera-info-manager \
    python3-pip \
    v4l-utils \
    usbutils

echo "2. Installing Python Dependencies..."
# Install Python libraries required by our scripts
pip3 install --upgrade pip
pip3 install \
    opencv-python-headless \
    numpy \
    ultralytics \
    pyaudio \
    SpeechRecognition

echo "3. Verifying Installation..."
if python3 -c "import cv_bridge; import ultralytics; print('✅ Python deps OK')" 2>/dev/null; then
    echo "✅ Python dependencies verified"
else
    echo "❌ Python dependency check failed"
fi

if ros2 pkg list | grep -q cv_bridge; then
    echo "✅ ROS2 cv_bridge found"
else
    echo "❌ ROS2 cv_bridge NOT found"
fi

echo "============================================"
echo "Installation Complete"
echo "============================================"