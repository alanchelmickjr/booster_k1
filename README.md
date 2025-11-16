# Booster K1 - Smart Robot System

## Quick Start

### SSH into robot
```bash
ssh booster@192.168.88.153
```

## Available Programs

### 1. Basic Robot Controls
```bash
python src/basic_controls.py eth0
```
- Keyboard control (WASD + QE for rotation)
- Hand gestures (rock, paper, scissors, grasp, OK)
- Head control (up, down, left, right)
- Voice commands (optional)

### 2. Basic Camera Feed
```bash
python src/basic_cam.py
```
- Web-based camera viewer
- View at: http://192.168.88.153:8080
- Supports stereo cameras with `--stereo` flag

### 3. YOLO Object Detection
```bash
python src/came_yolo.py
```
- Real-time object/face detection
- Web interface at: http://192.168.88.153:8080
- Options: `--model yolov8n`, `--detection face/object`

### 4. Smart Recognition System (NEW! 🎯)
```bash
python src/smart_recognition.py
```

**Features:**
- YOLO object detection
- Face recognition with DeepFace
- Text-to-Speech (asks "Who is this?" for unknown people)
- Object naming database
- Web interface at: http://192.168.88.153:8080

**Usage:**
1. When K1 sees an unknown person, it asks "Who is this?"
2. Use web interface to teach names
3. Name objects through the web UI
4. K1 remembers everyone and everything!

**Installation:**
```bash
# Install dependencies
pip install -r requirements.txt

# Install espeak for TTS
sudo apt-get install espeak
```

## Features

- ✅ Robot locomotion control
- ✅ Camera feed streaming
- ✅ YOLO object detection
- ✅ Face recognition with DeepFace
- ✅ Text-to-Speech interaction
- ✅ Person/object database
- ✅ Auto-learning mode

See [ROADMAP.md](ROADMAP.md) for planned features!
