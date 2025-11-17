# K-1 YOLO Wave Detection System - User Guide

## 🚀 System Status: RUNNING

The YOLO Wave Detection system is now active on your K-1 robot with a web interface for monitoring.

## 📺 Access the Web Interface

Open your browser and go to:
**http://192.168.88.153:8080**

You'll see:
- Live camera feed with YOLO annotations
- Green boxes around detected people
- Yellow boxes when wave is detected
- FPS and detection statistics
- Visual indicators for wave recognition

## 👋 How to Test Wave Detection

1. **Stand in front of the camera** - Make sure you're visible
2. **Wave your hand** - Move it side to side above shoulder level
3. **Look for visual feedback**:
   - Box color changes from green to yellow
   - "WAVE DETECTED!" appears on screen
   - Robot should say hello via TTS

## 🎯 Current Detection Settings

- **Confidence Threshold**: 0.4 (40%)
- **Wave Movement Threshold**: 20 pixels horizontal movement
- **Frame Skip**: Processing every 2nd frame for performance
- **Max Resolution**: 640px width
- **TTS Cooldown**: 5 seconds between wave greetings

## 🔍 Debugging Tips

If wave detection isn't working:
1. **Check the web interface** - Are you being detected as a person?
2. **Wave more dramatically** - Bigger side-to-side motion
3. **Move closer to camera** - Need minimum 100px height
4. **Check lighting** - Better lighting helps detection

## 📊 What You Should See

In the web interface:
- **FPS counter** - Should be around 10-15 FPS
- **People count** - Number of people detected
- **Bounding boxes** - Around each detected person
- **Center dots** - Blue dots tracking person center (for wave detection)
- **Status messages** - "WAVE DETECTED!" when recognized

## 🛑 To Stop the System

From terminal:
```bash
# Find the process
ps aux | grep yolo_wave_detector_web
# Kill it
kill [PID]
```

Or press Ctrl+C in the terminal where it's running.

## 🔧 Performance Notes

The system is optimized for Jetson Orin NX 8GB:
- Frame skipping reduces load
- Resolution limiting prevents memory issues  
- Error recovery handles camera crashes
- Auto-restart on failures

## 📝 Known Issues

1. **Camera may crash after extended use** - System will auto-restart
2. **Wave detection needs clear arm movement** - Side-to-side works best
3. **Multiple people may confuse wave detection** - Works best with 1-2 people

## 🎉 Success Indicators

You know it's working when:
✅ Web interface shows live video
✅ Green boxes appear around people
✅ Robot says hello when you wave
✅ Yellow box appears during wave
✅ FPS counter shows 10+ FPS

---

**Current deployment**: Running on port 8080
**Process**: Running in background
**Auto-restart**: Enabled (up to 5 attempts)