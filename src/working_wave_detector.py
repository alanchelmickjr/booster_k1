#!/usr/bin/env python3
"""
Working wave detector with proper ROS2 init and web streaming
"""

import os
import sys
import time
import threading
import cv2
import numpy as np
from flask import Flask, Response
from ultralytics import YOLO

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
    <style>
        body { background: #000; color: #0f0; font-family: monospace; text-align: center; }
        h1 { color: #0f0; }
        img { border: 2px solid #0f0; }
    </style>
</head>
<body>
    <h1>K-1 Robot Vision</h1>
    <img src="/video_feed" width="640">
    <p>Wave at the camera!</p>
</body>
</html>
'''

class WaveDetector(Node):
    def __init__(self):
        super().__init__('wave_detector')
        
        self.bridge = CvBridge()
        self.yolo_model = YOLO('/home/booster/yolov8n.pt')
        
        self.frame_count = 0
        self.last_positions = []
        self.last_wave_time = 0
        
        self.subscription = self.create_subscription(
            Image,
            '/image_left_raw',
            self.camera_callback,
            10
        )
        
        print("Wave detector initialized!")
        os.system('espeak "Wave detector ready!" 2>/dev/null')
    
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
            
            # Copy for output
            display = frame.copy()
            
            # Run YOLO
            results = self.yolo_model(frame, conf=0.3, verbose=False)
            
            # Process detections
            for result in results:
                if result.boxes is None:
                    continue
                
                for box in result.boxes:
                    cls = int(box.cls[0])
                    if cls == 0:  # Person
                        x1, y1, x2, y2 = map(int, box.xyxy[0].cpu().numpy())
                        center_x = (x1 + x2) // 2
                        
                        # Draw box
                        cv2.rectangle(display, (x1, y1), (x2, y2), (0, 255, 0), 2)
                        
                        # Track positions
                        self.last_positions.append(center_x)
                        if len(self.last_positions) > 10:
                            self.last_positions.pop(0)
                        
                        # Wave detection
                        if len(self.last_positions) >= 5:
                            movement = max(self.last_positions) - min(self.last_positions)
                            
                            if movement > 25:  # Wave detected
                                cv2.putText(display, "WAVE DETECTED!", (x1, y1-10),
                                          cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 255), 2)
                                
                                # Speak if not recently
                                current_time = time.time()
                                if current_time - self.last_wave_time > 5:
                                    print(f"WAVE DETECTED! Movement: {movement}")
                                    os.system('espeak "Hello! Nice wave!" 2>/dev/null &')
                                    self.last_wave_time = current_time
            
            # Add frame info
            cv2.putText(display, f"Frame: {self.frame_count}", (10, 30),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
            
            # Update global frame
            with frame_lock:
                ret, buffer = cv2.imencode('.jpg', display)
                if ret:
                    output_frame = buffer.tobytes()
            
            # Log periodically
            if self.frame_count % 60 == 0:
                print(f"Processed {self.frame_count} frames")
                
        except Exception as e:
            print(f"Error: {e}")

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
    print("Starting web server on port 8080...")
    app.run(host='0.0.0.0', port=8080, debug=False, threaded=True)

def main():
    # Initialize ROS2 properly
    try:
        if not rclpy.ok():
            rclpy.init()
    except:
        rclpy.init()
    
    # Start Flask in background
    flask_thread = threading.Thread(target=run_flask, daemon=True)
    flask_thread.start()
    
    # Create and run node
    try:
        node = WaveDetector()
        print("System running! Open http://192.168.88.153:8080")
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\nShutting down...")
    except Exception as e:
        print(f"Error: {e}")
    finally:
        if 'node' in locals():
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()