#!/bin/bash

# K-1 YOLO Wave Detection Launcher
# Automatically handles ROS2 setup and dependency checks

echo "============================================"
echo "   K-1 YOLO Wave Detection System"
echo "============================================"
echo ""

# Set working directory
cd /home/booster/booster_k1

# Setup ROS2 environment
echo "Setting up ROS2 environment..."
source /opt/ros/humble/setup.bash

# Check if camera topic exists
echo "Checking camera topics..."
if ros2 topic list | grep -q "/image_left_raw"; then
    echo "✓ Camera topic /image_left_raw found"
else
    echo "⚠ Warning: Camera topic /image_left_raw not found"
    echo "Available topics:"
    ros2 topic list | grep image
fi

# Check Python dependencies
echo "Checking Python dependencies..."
python3 -c "import cv2, numpy, ultralytics" 2>/dev/null
if [ $? -eq 0 ]; then
    echo "✓ All Python dependencies available"
else
    echo "⚠ Missing Python dependencies, installing..."
    pip3 install opencv-python ultralytics --quiet
fi

# Test TTS availability
echo "Testing Text-to-Speech..."
if command -v espeak &> /dev/null; then
    echo "✓ Using espeak for TTS"
    # Test with a quiet voice
    espeak "System ready" 2>/dev/null
else
    if command -v piper &> /dev/null; then
        echo "✓ Using piper for TTS"
    else
        echo "⚠ No TTS engine found, voice disabled"
    fi
fi

# Check for TTS module availability
if python3 -c "from src.tts_module import TextToSpeech" 2>/dev/null; then
    echo "✓ TTS module available"
else
    echo "⚠ TTS module not found, running without voice"
fi

# Check YOLO model
if [ -f "/home/booster/yolov8n.pt" ]; then
    echo "✓ YOLO model found"
else
    echo "⚠ YOLO model not found at /home/booster/yolov8n.pt"
    echo "Downloading YOLOv8n model..."
    python3 -c "from ultralytics import YOLO; YOLO('yolov8n.pt')" 2>/dev/null
fi

echo ""
echo "============================================"
echo "Starting YOLO Wave Detector..."
echo "============================================"
echo ""
echo "Instructions:"
echo "- Wave at the camera to interact"
echo "- The robot will greet you when it sees you"
echo "- Press Ctrl+C to stop"
echo ""
echo "Launching..."
echo ""

# Function to run the detector
run_detector() {
    # Use the fixed version
    python3 /home/booster/booster_k1/src/yolo_wave_detector_fixed.py
    return $?
}

# Main loop with automatic restart
RESTART_COUNT=0
MAX_RESTARTS=5

while [ $RESTART_COUNT -lt $MAX_RESTARTS ]; do
    run_detector
    EXIT_CODE=$?
    
    if [ $EXIT_CODE -eq 0 ]; then
        echo "Detector exited normally"
        break
    elif [ $EXIT_CODE -eq 130 ]; then
        echo "Stopped by user (Ctrl+C)"
        break
    else
        RESTART_COUNT=$((RESTART_COUNT + 1))
        echo "⚠ Detector crashed (exit code: $EXIT_CODE)"
        
        if [ $RESTART_COUNT -lt $MAX_RESTARTS ]; then
            echo "Restarting in 3 seconds... (attempt $RESTART_COUNT/$MAX_RESTARTS)"
            sleep 3
        else
            echo "❌ Maximum restart attempts reached"
            exit 1
        fi
    fi
done

echo ""
echo "============================================"
echo "YOLO Wave Detector stopped"
echo "============================================"