# K-1 YOLO Wave Detection System - User Guide

## 🚀 System Status: SOFTWARE READY (Hardware Check Required)

The YOLO Wave Detection system has been consolidated and optimized. However, recent diagnostics indicate a potential hardware issue with the camera connection.

## ⚠️ Critical Hardware Check

Before running the system, please verify that your camera is detected:

```bash
ls -l /dev/video*
```

**If this returns "No such file or directory":**
1.  **Check Connections:** Ensure the camera USB/CSI cable is firmly connected to the Jetson.
2.  **Reboot Robot:** A full system reboot often fixes missing video devices.
    ```bash
    sudo reboot
    ```
3.  **Check Kernel Logs:**
    ```bash
    dmesg | grep -i -E 'camera|video|uvc'
    ```

## 🎮 Quick Start (Once Camera is Detected)

To start the wave detector:

```bash
./start_wave_detector.sh
```

## 📺 Access the Web Interface

Open your browser and go to:
**http://192.168.88.153:8080**

You'll see:
- Live camera feed with YOLO annotations
- Green boxes around detected people
- Yellow boxes when wave is detected
- FPS and detection statistics
- Visual indicators for wave recognition

## 🔧 Troubleshooting & Tools

We have included new diagnostic tools to help you:

1.  **`./diagnose_camera.sh`**: Checks ROS2 topics, running processes, and video devices.
2.  **`./restart_camera.sh`**: Attempts to forcefully restart the camera bridge service.

## 📂 System Architecture

The system has been simplified to:

1.  **`src/wave_detector.py`** - The core logic (YOLO + Wave Detection + Web Server + TTS)
2.  **`start_wave_detector.sh`** - The entry point script that auto-detects camera topics.
3.  **`src/tts_module.py`** - Shared Text-to-Speech module.

## 🎯 Detection Settings

- **Confidence Threshold**: 0.3 (30%)
- **Wave Movement Threshold**: 25 pixels horizontal movement
- **TTS Cooldown**: 5 seconds between wave greetings
- **Greeting Cooldown**: 15 seconds for general person detection

## 🛑 To Stop the System

From terminal where it's running:
Press **Ctrl+C**

Or if running in background:
```bash
pkill -f wave_detector