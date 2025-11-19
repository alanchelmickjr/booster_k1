# K1 Robot Technical Information

## Vision System
- **Camera**: Depth camera located in the head
- **Topics**: 
  - `/booster_camera_bridge/image_left_raw` (from came_yolo.py)
  - `/image_left_raw` (direct from mipi_cam)
- **Bridge Binary**: `/opt/booster/DaemonPerception/bin/booster-camera-bridge`

## Audio System  
- **Input**: Microphone array in head
- **Output**: Speaker in torso
- **TTS**: espeak or piper

## Controller
- **Board**: Jetson Orin NX 8GB
- **Location**: Torso

## Working Files (from arminforoughi/booster_k1)
1. **basic_cam.py** - Basic camera viewer
2. **basic_controls.py** - Robot movement controls  
3. **came_yolo.py** - YOLO detection with web interface

## Critical Finding
The came_yolo.py REQUIRES:
1. The booster-camera-bridge to be running
2. The bridge publishes to `/booster_camera_bridge/image_left_raw`
3. The direct mipi_cam publishes to `/image_left_raw` but crashes

## The Problem
The camera system has crashed. The bridge was killed during our debugging and needs to be properly restarted.