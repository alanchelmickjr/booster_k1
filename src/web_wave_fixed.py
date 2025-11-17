#!/usr/bin/env python3
"""
Fixed wave detector with working web interface on port 8080
"""

import os
import sys
import time
import threading
import signal
import cv2
import numpy as np
from flask import Flask, Response
from ultralytics import YOLO

# Kill anything on port 8080 first
os.system("lsof -ti:8080 | xargs kill -9 2>/dev/null")
time.sleep(1)

# ROS2 imports
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

# Flask app
app = Flask(__name__)
output_frame = None
frame_lock = threading.Lock()

HTML = '''
<!DOCTYPE html>
<html>
<head>
    <title>K-1 Wave Detection</title>
    <meta http-equiv="refresh" content="30">
    <style>
        body { 
            background: #1a1a1a; 
            color: #0f0; 
            font-family: monospace; 
            text-align: center;
            margin: 0;
            padding: 20px;
        }
        h1 { 
            color: #0f0; 
            text-shadow: 0 0 10px #0f0;
        }
        .container {
            max-width: 800px;
            margin: 0 auto;
        }
        img { 
            border: 3px solid #0f0;
            box-shadow: 0 0 20px #0f0;
            max-width: 100%;
            height: auto;
        }
        .status {
            background: #000;
            border: 1px solid #0f0;
            padding: 10px;
            margin: 20px 0;
            font-size: 18px;
        }
        .wave-detected {
            color: #ff0;
            font-size: 24px;
            animation: blink 0.5s infinite;
        }
        @keyframes blink {
            50% { opacity: 0.5; }
        }
    </style>
</head>
<body>
    <div class="container">
        <h1>🤖 K-1 Robot Vision - YOLO Wave Detection</h1>
        <img src="/video_feed" alt="Live Camera Feed">
        <div class="status">
            <p>👁️ Camera: ACTIVE | 🎯 YOLO: RUNNING | 👋 Wave to interact!</p>
            <p>Green box = Person | Yellow box = Wave detected</p>
        </div>
    </div>
</body>
</html>
'''

class WebWaveDetector(Node):
    def __init__(self):
        super().__init__('web_wave_detector')
        
        self.bridge = CvBridge()
        print("Loading YOLO model...")
        self.yolo_model = YOLO('/home/booster/yolov8n.pt')
        print("YOLO loaded!")
        
        self.frame_count = 0
        self.last_positions = []
        self.last_wave_time = 0
        self.last_greeting_time = 0
        
        self.subscription = self.create_subscription(
            Image,
            '/image_left_raw',
            self.camera_callback,
            10
        )
        
        print("Wave detector initialized!")
        os.system('espeak "Wave detector online! Check port 8080!" 2>/dev/null &')
    
    def camera_callback(self, msg):
        global output_frame
        
        try:
            self.frame_count += 1
            if self.frame_count % 2 != 0:  # Skip frames
                return
            
            # Convert image
            frame = None
            if msg.encoding == 'nv12':
                height = msg.height
                width = msg.width
                img_data = np.frombuffer(msg.data, dtype=np.uint8)
                expected_size = int(height * 1.5 * width)
                if len(img_data) == expected_size:
                    yuv = img_data.reshape((int(height * 1.5), width))
                    frame = cv2.cvtColor(yuv, cv2.COLOR_YUV2BGR_NV12)
            else:
                frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            
            if frame is None:
                return
            
            # Resize
            if frame.shape[1] > 640:
                scale = 640 / frame.shape[1]
                frame = cv2.resize(frame, (int(frame.shape[1] * scale), int(frame.shape[0] * scale)))
            
            # Copy for display
            display = frame.copy()
            
            # Add frame counter
            cv2.putText(display, f"Frame: {self.frame_count}", (10, 30),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
            
            # Run YOLO
            results = self.yolo_model(frame, conf=0.25, verbose=False)
            
            # Process detections
            wave_detected = False
            person_count = 0
            
            for result in results:
                if result.boxes is None:
                    continue
                
                for box in result.boxes:
                    cls = int(box.cls[0])
                    if cls == 0:  # Person
                        person_count += 1
                        x1, y1, x2, y2 = map(int, box.xyxy[0].cpu().numpy())
                        center_x = (x1 + x2) // 2
                        
                        # Track positions
                        self.last_positions.append(center_x)
                        if len(self.last_positions) > 10:
                            self.last_positions.pop(0)
                        
                        # Wave detection
                        movement = 0
                        if len(self.last_positions) >= 5:
                            movement = max(self.last_positions) - min(self.last_positions)
                            if movement > 20:  # Wave threshold
                                wave_detected = True
                        
                        # Draw box
                        color = (0, 255, 255) if wave_detected else (0, 255, 0)
                        thickness = 3 if wave_detected else 2
                        cv2.rectangle(display, (x1, y1), (x2, y2), color, thickness)
                        
                        # Add label
                        label = f"Person {person_count}"
                        if wave_detected:
                            label = "WAVING! " + label
                        cv2.putText(display, label, (x1, y1-10),
                                   cv2.FONT_HERSHEY_SIMPLEX, 0.9, color, 2)
                        
                        # Draw center dot
                        cv2.circle(display, (center_x, (y1+y2)//2), 5, (255, 0, 0), -1)
                        
                        # Show movement value
                        if movement > 0:
                            cv2.putText(display, f"Move: {movement}", (x1, y2+20),
                                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 1)
            
            # Handle TTS
            current_time = time.time()
            if person_count > 0 and current_time - self.last_greeting_time > 10:
                print(f"Person detected at frame {self.frame_count}")
                os.system('espeak "Hello! Wave at me!" 2>/dev/null &')
                self.last_greeting_time = current_time
            
            if wave_detected and current_time - self.last_wave_time > 5:
                print(f"*** WAVE DETECTED at frame {self.frame_count}! ***")
                os.system('espeak "Nice wave! Hello friend!" 2>/dev/null &')
                self.last_wave_time = current_time
            
            # Add status overlay
            status_text = f"People: {person_count}"
            if wave_detected:
                status_text += " | WAVE DETECTED!"
            cv2.putText(display, status_text, (10, 60),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
            
            # Update global frame
            with frame_lock:
                ret, buffer = cv2.imencode('.jpg', display, [cv2.IMWRITE_JPEG_QUALITY, 80])
                if ret:
                    output_frame = buffer.tobytes()
            
            # Log periodically
            if self.frame_count % 60 == 0:
                print(f"[{time.strftime('%H:%M:%S')}] Processed {self.frame_count} frames, {person_count} people")
                
        except Exception as e:
            print(f"Error in callback: {e}")

@app.route('/')
def index():
    return HTML

@app.route('/video_feed')
def video_feed():
    def generate():
        global output_frame
        while True:
            with frame_lock:
                if output_frame is not None:
                    yield (b'--frame\r\n'
                           b'Content-Type: image/jpeg\r\n\r\n' + output_frame + b'\r\n')
            time.sleep(0.03)
    
    return Response(generate(), mimetype='multipart/x-mixed-replace; boundary=frame')

def run_flask():
    print("Starting web server on http://192.168.88.153:8080")
    app.run(host='0.0.0.0', port=8080, debug=False, threaded=True)

def signal_handler(sig, frame):
    print("\nShutting down...")
    os.system('espeak "Goodbye!" 2>/dev/null')
    sys.exit(0)

def main():
    # Set up signal handler
    signal.signal(signal.SIGINT, signal_handler)
    
    # Initialize ROS2
    try:
        rclpy.init()
    except:
        pass
    
    # Start Flask in background
    flask_thread = threading.Thread(target=run_flask, daemon=True)
    flask_thread.start()
    
    # Give Flask time to start
    time.sleep(2)
    
    # Create and run node
    try:
        node = WebWaveDetector()
        print("\n" + "="*50)
        print("SYSTEM RUNNING!")
        print("Web interface: http://192.168.88.153:8080")
        print("Wave at the camera to interact!")
        print("="*50 + "\n")
        
        rclpy.spin(node)
    except Exception as e:
        print(f"Error: {e}")
    finally:
        if 'node' in locals():
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()