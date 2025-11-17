#!/usr/bin/env python3
"""
Enhanced YOLO Person Detection with Wave Recognition, TTS, and Web Streaming
Streams the YOLO view to http://192.168.88.153:8080
"""

import os
import sys
import time
import threading
from collections import deque
import io
from flask import Flask, Response, render_template_string

# Ensure ROS2 is available
try:
    import rclpy
    from rclpy.node import Node
    from sensor_msgs.msg import Image
except ImportError as e:
    print(f"Error importing ROS2: {e}")
    print("Please source ROS2 setup.bash first")
    sys.exit(1)

from cv_bridge import CvBridge
import cv2
import numpy as np
from ultralytics import YOLO

# Add parent directory to path for imports
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
try:
    from src.tts_module import TextToSpeech
except ImportError:
    print("Warning: TTS module not found, running without voice")
    class TextToSpeech:
        def __init__(self, engine='auto'):
            self.available = False
        def speak(self, text, blocking=False):
            pass

# Flask app for web streaming
app = Flask(__name__)

# HTML template for viewing stream
HTML_TEMPLATE = '''
<!DOCTYPE html>
<html>
<head>
    <title>K-1 YOLO Vision</title>
    <style>
        body {
            font-family: Arial, sans-serif;
            margin: 0;
            padding: 20px;
            background-color: #1a1a1a;
            color: #fff;
        }
        h1 {
            text-align: center;
            color: #00ff00;
        }
        .container {
            max-width: 1200px;
            margin: 0 auto;
        }
        .video-container {
            text-align: center;
            background-color: #2a2a2a;
            padding: 20px;
            border-radius: 10px;
            box-shadow: 0 4px 6px rgba(0,0,0,0.3);
        }
        img {
            max-width: 100%;
            height: auto;
            border: 2px solid #00ff00;
            border-radius: 5px;
        }
        .stats {
            margin-top: 20px;
            padding: 15px;
            background-color: #333;
            border-radius: 5px;
            font-family: monospace;
        }
        .status {
            color: #00ff00;
            font-weight: bold;
        }
        .wave-detected {
            color: #ffff00;
            font-size: 1.5em;
            animation: pulse 1s infinite;
        }
        @keyframes pulse {
            0% { opacity: 1; }
            50% { opacity: 0.5; }
            100% { opacity: 1; }
        }
    </style>
</head>
<body>
    <div class="container">
        <h1>🤖 K-1 Robot Vision - YOLO Detection</h1>
        <div class="video-container">
            <img src="/video_feed" alt="YOLO Detection Stream">
            <div class="stats">
                <p class="status">👁️ Live Camera Feed</p>
                <p>🎯 Person Detection: Active</p>
                <p>👋 Wave Recognition: Active</p>
                <p>🔊 TTS Greetings: Enabled</p>
                <p id="wave-status"></p>
            </div>
        </div>
        <div style="text-align: center; margin-top: 20px; color: #888;">
            <p>Wave at the camera to interact with K-1!</p>
            <p>Green boxes = People detected | Yellow boxes = Wave detected</p>
        </div>
    </div>
    <script>
        // Auto-refresh if stream stops
        let img = document.querySelector('img');
        let errorCount = 0;
        img.onerror = function() {
            errorCount++;
            if (errorCount > 3) {
                setTimeout(() => location.reload(), 3000);
            }
        };
        img.onload = function() {
            errorCount = 0;
        };
    </script>
</body>
</html>
'''

class YOLOWaveNode(Node):
    """ROS2 node for YOLO person detection with wave recognition and web streaming"""
    
    def __init__(self):
        super().__init__('yolo_wave_detector')
        
        # Initialize components
        self.bridge = CvBridge()
        self.yolo_model = YOLO('/home/booster/yolov8n.pt')
        self.tts = TextToSpeech(engine='auto')
        
        # State tracking
        self.latest_frame = None
        self.output_frame = None  # Frame with YOLO annotations for streaming
        self.people_count = 0
        self.last_greeting_time = 0
        self.greeting_cooldown = 10.0
        self.frame_count = 0
        self.error_count = 0
        self.last_frame_time = time.time()
        self.fps = 0
        
        # Frame skip for performance
        self.frame_skip = 2  # Process every 2nd frame
        self.frame_skip_counter = 0
        
        # Error recovery
        self.max_errors = 10
        self.recovery_delay = 1.0
        
        # Wave detection state
        self.wave_history = deque(maxlen=10)
        self.last_wave_time = 0
        self.wave_cooldown = 5.0
        self.wave_active = False
        
        # Subscribe to camera topic
        try:
            self.subscription = self.create_subscription(
                Image,
                '/image_left_raw',
                self.camera_callback,
                10
            )
            self.get_logger().info('YOLO Wave Detector initialized')
            self.get_logger().info('Using camera topic: /image_left_raw')
            self.get_logger().info('Web stream available at: http://192.168.88.153:8080')
        except Exception as e:
            self.get_logger().error(f'Failed to create subscription: {e}')
            raise
        
        # Announce startup
        if self.tts.available:
            self.tts.speak("Wave detection system online. View my vision at port 8080!", blocking=False)
    
    def camera_callback(self, msg):
        """Process camera frames with error recovery"""
        try:
            # Frame skipping for performance
            self.frame_skip_counter += 1
            if self.frame_skip_counter % self.frame_skip != 0:
                return
            
            # Update FPS
            current_time = time.time()
            if current_time - self.last_frame_time > 0:
                self.fps = 1.0 / (current_time - self.last_frame_time)
            self.last_frame_time = current_time
            
            # Convert image safely
            frame = None
            if msg.encoding == 'nv12':
                # Handle NV12 encoding
                height = msg.height
                width = msg.width
                img_data = np.frombuffer(msg.data, dtype=np.uint8)
                
                # Careful with reshape to avoid memory issues
                try:
                    # Calculate expected size
                    expected_size = int(height * 1.5 * width)
                    if len(img_data) == expected_size:
                        yuv = img_data.reshape((int(height * 1.5), width))
                        frame = cv2.cvtColor(yuv, cv2.COLOR_YUV2BGR_NV12)
                    else:
                        self.get_logger().warning(f'Size mismatch: expected {expected_size}, got {len(img_data)}')
                        return
                except Exception as e:
                    self.get_logger().error(f'NV12 conversion error: {e}')
                    self.error_count += 1
                    return
            else:
                try:
                    frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
                except Exception as e:
                    self.get_logger().error(f'Image conversion error: {e}')
                    self.error_count += 1
                    return
            
            if frame is None:
                return
            
            # Resize for performance (max 640 width)
            if frame.shape[1] > 640:
                scale = 640 / frame.shape[1]
                new_width = int(frame.shape[1] * scale)
                new_height = int(frame.shape[0] * scale)
                frame = cv2.resize(frame, (new_width, new_height))
            
            # Create output frame for streaming
            output = frame.copy()
            
            # Run YOLO detection
            try:
                results = self.yolo_model(frame, conf=0.4, verbose=False)  # Lower confidence for better detection
            except Exception as e:
                self.get_logger().error(f'YOLO detection error: {e}')
                return
            
            # Process detections
            people_detected = 0
            wave_detected = False
            
            for result in results:
                if result.boxes is None:
                    continue
                    
                boxes = result.boxes
                
                for box in boxes:
                    cls = int(box.cls[0])
                    conf = float(box.conf[0])
                    
                    # Check if it's a person (class 0 in COCO)
                    if cls == 0:
                        people_detected += 1
                        x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()
                        x1, y1, x2, y2 = int(x1), int(y1), int(x2), int(y2)
                        
                        # Calculate person dimensions
                        person_height = y2 - y1
                        person_width = x2 - x1
                        center_x = (x1 + x2) // 2
                        
                        # Improved wave detection
                        # Check if arms are raised (upper body motion)
                        if person_height > 100:
                            # Check upper 40% of body for raised arms
                            upper_region = output[y1:y1+int(person_height*0.4), x1:x2]
                            
                            # Track center position for movement
                            self.wave_history.append(center_x)
                            
                            # Detect wave based on horizontal movement
                            if len(self.wave_history) >= 5:
                                movement = max(self.wave_history) - min(self.wave_history)
                                # More sensitive wave detection
                                if movement > 20:  # Reduced threshold
                                    wave_detected = True
                                    self.wave_active = True
                            
                            # Alternative: Check if hands are above shoulders (simplified)
                            # This is a heuristic since YOLOv8n doesn't provide keypoints
                            upper_y_check = y1 + int(person_height * 0.3)
                            if upper_region.size > 0:
                                # Check for skin-like colors in upper region (very simplified)
                                hsv = cv2.cvtColor(upper_region, cv2.COLOR_BGR2HSV)
                                # Skin color range in HSV
                                lower_skin = np.array([0, 20, 70], dtype=np.uint8)
                                upper_skin = np.array([20, 255, 255], dtype=np.uint8)
                                skin_mask = cv2.inRange(hsv, lower_skin, upper_skin)
                                skin_ratio = np.sum(skin_mask > 0) / skin_mask.size
                                
                                if skin_ratio > 0.1:  # If significant skin detected in upper region
                                    wave_detected = True
                                    self.wave_active = True
                        
                        # Draw bounding box
                        color = (0, 255, 0)  # Green for person
                        thickness = 2
                        
                        if wave_detected or self.wave_active:
                            color = (0, 255, 255)  # Yellow for waving
                            thickness = 4
                        
                        cv2.rectangle(output, (x1, y1), (x2, y2), color, thickness)
                        
                        # Add label with larger font
                        label = f"Person {conf:.2f}"
                        if wave_detected:
                            label = "WAVING! " + label
                        
                        # Add background for text
                        (text_width, text_height), _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.9, 2)
                        cv2.rectangle(output, (x1, y1-text_height-10), (x1+text_width, y1), color, -1)
                        cv2.putText(output, label, (x1, y1-5),
                                   cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 0, 0), 2)
                        
                        # Draw center point for debugging
                        cv2.circle(output, (center_x, y1 + person_height//2), 5, (255, 0, 0), -1)
            
            # Decay wave active state
            if not wave_detected and self.wave_active:
                self.wave_active = False
            
            # Update people count
            self.people_count = people_detected
            
            # Handle greetings
            if people_detected > 0:
                self._handle_greeting(people_detected, wave_detected)
            
            # Add stats overlay with better visibility
            overlay_bg = output.copy()
            cv2.rectangle(overlay_bg, (0, 0), (350, 120), (0, 0, 0), -1)
            output = cv2.addWeighted(output, 0.7, overlay_bg, 0.3, 0)
            
            cv2.putText(output, f'FPS: {self.fps:.1f}', (10, 30),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 255), 2)
            cv2.putText(output, f'People: {people_detected}', (10, 60),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 255), 2)
            
            if wave_detected:
                cv2.putText(output, 'WAVE DETECTED!', (10, 90),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 3)
            
            # Store frames
            self.latest_frame = frame
            self.output_frame = output
            self.frame_count += 1
            
            # Reset error count on successful frame
            self.error_count = 0
            
            # Log periodically
            if self.frame_count % 30 == 0:
                self.get_logger().info(f'Processed {self.frame_count} frames, FPS: {self.fps:.1f}, People: {people_detected}')
                
        except Exception as e:
            self.error_count += 1
            self.get_logger().error(f'Frame processing error #{self.error_count}: {e}')
            
            # Automatic recovery
            if self.error_count >= self.max_errors:
                self.get_logger().warning('Too many errors, attempting recovery...')
                time.sleep(self.recovery_delay)
                self.error_count = 0
    
    def _handle_greeting(self, people_count, wave_detected):
        """Handle TTS greetings for detected people"""
        if not self.tts.available:
            return
            
        current_time = time.time()
        
        # Special handling for waves
        if wave_detected:
            if current_time - self.last_wave_time > self.wave_cooldown:
                self.tts.speak("Hello! I see you waving! Nice to meet you!", blocking=False)
                self.last_wave_time = current_time
                self.last_greeting_time = current_time
            return
        
        # Check cooldown for general greetings
        if current_time - self.last_greeting_time < self.greeting_cooldown:
            return
        
        # Greet based on detection
        if people_count == 1 and self.frame_count < 100:  # New person
            self.tts.speak("Hello there! Wave at me to say hi!", blocking=False)
            self.last_greeting_time = current_time
        elif people_count > 1 and self.frame_count < 100:
            self.tts.speak(f"Hello everyone! I see {people_count} people!", blocking=False)
            self.last_greeting_time = current_time
    
    def get_output_frame(self):
        """Get the latest processed frame for streaming"""
        return self.output_frame


# Global node reference for Flask
node = None

@app.route('/')
def index():
    """Home page"""
    return render_template_string(HTML_TEMPLATE)

@app.route('/video_feed')
def video_feed():
    """Video streaming route"""
    def generate():
        while True:
            if node and node.output_frame is not None:
                # Encode frame as JPEG
                ret, buffer = cv2.imencode('.jpg', node.output_frame)
                if ret:
                    frame = buffer.tobytes()
                    yield (b'--frame\r\n'
                           b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')
            time.sleep(0.033)  # ~30 FPS
    
    return Response(generate(),
                    mimetype='multipart/x-mixed-replace; boundary=frame')


def run_flask():
    """Run Flask server in a separate thread"""
    app.run(host='0.0.0.0', port=8080, debug=False, threaded=True)


def main():
    """Main entry point"""
    global node
    
    print("Starting YOLO Wave Detection System with Web Streaming...")
    print("="*60)
    
    # Check if ROS2 is properly initialized
    if not rclpy.ok():
        try:
            rclpy.init()
        except Exception as e:
            print(f"Failed to initialize ROS2: {e}")
            print("Please ensure ROS2 is sourced properly")
            return 1
    
    try:
        # Create node
        node = YOLOWaveNode()
        
        # Start Flask server in background thread
        flask_thread = threading.Thread(target=run_flask, daemon=True)
        flask_thread.start()
        
        print("System initialized!")
        print("- Person detection: Active")
        print("- Wave detection: Active (improved sensitivity)") 
        print("- TTS greetings: Active" if node.tts.available else "- TTS greetings: Disabled")
        print("- Camera topic: /image_left_raw")
        print("- Web interface: http://192.168.88.153:8080")
        print("="*60)
        print("Wave at the camera to interact!")
        print("View the live stream at: http://192.168.88.153:8080")
        print("Press Ctrl+C to stop")
        
        # Spin node
        rclpy.spin(node)
        
    except KeyboardInterrupt:
        print("\nShutting down...")
        if node and node.tts.available:
            node.tts.speak("Goodbye!", blocking=True)
    except Exception as e:
        print(f"Error running node: {e}")
        return 1
    finally:
        # Cleanup
        if node:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    
    return 0


if __name__ == '__main__':
    sys.exit(main())