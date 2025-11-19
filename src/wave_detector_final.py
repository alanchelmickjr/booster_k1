#!/usr/bin/env python3
"""
FINAL Wave detector using CORRECT camera topic from bridge
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
from http.server import HTTPServer, BaseHTTPRequestHandler
import threading
import time
import os
from ultralytics import YOLO

class WaveDetector:
    """Wave detection with YOLO"""
    
    def __init__(self):
        print("Loading YOLO model...")
        self.model = YOLO('/home/booster/yolov8n.pt')
        self.last_positions = []
        self.last_wave_time = 0
        self.last_greeting_time = 0
        print("YOLO ready!")
    
    def detect(self, frame):
        """Run detection and annotate frame"""
        # Run YOLO
        results = self.model(frame, conf=0.3, verbose=False)
        
        # Copy frame for annotation
        annotated = frame.copy()
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
                    
                    # Track positions for wave
                    self.last_positions.append(center_x)
                    if len(self.last_positions) > 10:
                        self.last_positions.pop(0)
                    
                    # Check for wave
                    movement = 0
                    if len(self.last_positions) >= 5:
                        movement = max(self.last_positions) - min(self.last_positions)
                        if movement > 25:
                            wave_detected = True
                    
                    # Draw box
                    color = (0, 255, 255) if wave_detected else (0, 255, 0)
                    thickness = 3 if wave_detected else 2
                    cv2.rectangle(annotated, (x1, y1), (x2, y2), color, thickness)
                    
                    # Label
                    label = "WAVING!" if wave_detected else "Person"
                    cv2.putText(annotated, label, (x1, y1-10),
                               cv2.FONT_HERSHEY_SIMPLEX, 0.9, color, 2)
                    
                    # Show movement
                    if movement > 0:
                        cv2.putText(annotated, f"Movement: {movement}", (x1, y2+20),
                                   cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 1)
        
        # Handle TTS
        current_time = time.time()
        if person_count > 0 and current_time - self.last_greeting_time > 10:
            os.system('espeak "Hello! Wave at me!" 2>/dev/null &')
            self.last_greeting_time = current_time
            print(f"Person detected at {time.strftime('%H:%M:%S')}")
        
        if wave_detected and current_time - self.last_wave_time > 5:
            os.system('espeak "Nice wave! Hello friend!" 2>/dev/null &')
            self.last_wave_time = current_time
            print(f"*** WAVE DETECTED at {time.strftime('%H:%M:%S')} ***")
        
        # Add status
        cv2.putText(annotated, f"People: {person_count}", (10, 30),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 255), 2)
        
        if wave_detected:
            cv2.putText(annotated, "WAVE DETECTED!", (10, 60),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
        
        return annotated

class CameraSubscriber(Node):
    """ROS2 node using CORRECT bridge topic"""
    
    def __init__(self, detector):
        super().__init__('wave_camera_subscriber')
        
        self.bridge = CvBridge()
        self.detector = detector
        self.latest_frame = None
        
        # Performance tracking
        self.fps = 0
        self.frame_count = 0
        self.last_fps_time = time.time()
        
        # Use the CORRECT bridge topic!
        topic = '/booster_camera_bridge/image_left_raw'
        self.get_logger().info(f'Using BRIDGE topic: {topic}')
        print(f"Subscribing to: {topic}")
        
        self.subscription = self.create_subscription(
            Image,
            topic,
            self.camera_callback,
            10
        )
    
    def convert_nv12_to_bgr(self, msg):
        """Convert NV12 to BGR"""
        if msg.encoding == 'nv12':
            height = msg.height
            width = msg.width
            img_data = np.frombuffer(msg.data, dtype=np.uint8)
            
            expected_size = int(height * 1.5 * width)
            if len(img_data) != expected_size:
                return None
            
            yuv = img_data.reshape((int(height * 1.5), width))
            cv_image = cv2.cvtColor(yuv, cv2.COLOR_YUV2BGR_NV12)
            return cv_image
        else:
            return self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
    
    def update_fps(self):
        """Update FPS counter"""
        self.frame_count += 1
        current_time = time.time()
        elapsed = current_time - self.last_fps_time
        
        if elapsed > 1.0:
            self.fps = self.frame_count / elapsed
            self.frame_count = 0
            self.last_fps_time = current_time
    
    def camera_callback(self, msg):
        """Process camera frames"""
        try:
            # Convert image
            cv_image = self.convert_nv12_to_bgr(msg)
            if cv_image is None:
                return
            
            # Resize if needed
            if cv_image.shape[1] > 640:
                scale = 640 / cv_image.shape[1]
                new_size = (int(cv_image.shape[1] * scale), int(cv_image.shape[0] * scale))
                cv_image = cv2.resize(cv_image, new_size)
            
            # Run detection
            detected_frame = self.detector.detect(cv_image)
            
            # Update FPS
            self.update_fps()
            
            # Add FPS
            cv2.putText(detected_frame, f'FPS: {self.fps:.1f}', (10, 90),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 255), 2)
            
            self.latest_frame = detected_frame
            
            # Log first frame
            if self.frame_count == 1:
                print("FIRST FRAME RECEIVED! System working!")
            
        except Exception as e:
            self.get_logger().error(f'Error: {str(e)}')

class WaveHTTPHandler(BaseHTTPRequestHandler):
    """HTTP handler for web interface"""
    
    camera_node = None
    
    def log_message(self, format, *args):
        """Suppress logs"""
        pass
    
    def do_GET(self):
        """Handle GET requests"""
        if self.path == '/':
            # Serve HTML
            self.send_response(200)
            self.send_header('Content-type', 'text/html')
            self.end_headers()
            
            html = """
<!DOCTYPE html>
<html>
<head>
    <title>K-1 Wave Detection WORKING</title>
    <style>
        body {
            font-family: Arial, sans-serif;
            background-color: #1e1e1e;
            color: #ffffff;
            margin: 0;
            padding: 20px;
            text-align: center;
        }
        h1 {
            color: #4CAF50;
        }
        .container {
            max-width: 800px;
            margin: 0 auto;
        }
        .feed-box {
            border: 2px solid #4CAF50;
            padding: 20px;
            border-radius: 8px;
            background-color: #2d2d2d;
        }
        img {
            max-width: 100%;
            height: auto;
            border-radius: 4px;
        }
        .status {
            color: #4CAF50;
            font-size: 1.2em;
            margin: 10px 0;
        }
    </style>
</head>
<body>
    <div class="container">
        <h1>🤖 K-1 Wave Detection - WORKING!</h1>
        <div class="status">Using Bridge Topic: /booster_camera_bridge/image_left_raw</div>
        
        <div class="feed-box">
            <img id="feed" src="/image" alt="Detection Feed">
        </div>
        
        <div style="margin-top: 20px;">
            <h3>👋 Wave Instructions:</h3>
            <p>1. Stand in front of camera</p>
            <p>2. Wave your hand side to side</p>
            <p>3. Yellow box = Wave detected!</p>
        </div>
    </div>
    
    <script>
        function refreshImage() {
            const img = document.getElementById('feed');
            img.src = '/image?t=' + new Date().getTime();
        }
        setInterval(refreshImage, 100);
    </script>
</body>
</html>
"""
            self.wfile.write(html.encode())
            
        elif self.path.startswith('/image'):
            if self.camera_node and self.camera_node.latest_frame is not None:
                self.send_response(200)
                self.send_header('Content-type', 'image/jpeg')
                self.send_header('Cache-Control', 'no-cache')
                self.end_headers()
                
                _, buffer = cv2.imencode('.jpg', self.camera_node.latest_frame,
                                        [cv2.IMWRITE_JPEG_QUALITY, 80])
                self.wfile.write(buffer.tobytes())
            else:
                self.send_error(503, 'No image available')
        else:
            self.send_error(404, 'Not found')

def spin_ros(node):
    """Spin ROS2 node in separate thread"""
    rclpy.spin(node)

def main():
    print(f'\n{"="*60}')
    print('K-1 WAVE DETECTION - FINAL VERSION')
    print('Using CORRECT bridge topic!')
    print(f'{"="*60}')
    
    # Kill anything on port 8080
    os.system("lsof -ti:8080 | xargs kill -9 2>/dev/null")
    time.sleep(1)
    
    # Initialize wave detector
    print("Initializing wave detector...")
    detector = WaveDetector()
    
    # Initialize ROS2
    print("Initializing ROS2...")
    rclpy.init()
    
    # Create camera node
    camera_node = CameraSubscriber(detector)
    
    # Set node for HTTP handler
    WaveHTTPHandler.camera_node = camera_node
    
    # Start ROS2 in separate thread
    ros_thread = threading.Thread(target=spin_ros, args=(camera_node,), daemon=True)
    ros_thread.start()
    
    # Start HTTP server
    port = 8080
    server_address = ('0.0.0.0', port)
    httpd = HTTPServer(server_address, WaveHTTPHandler)
    
    print(f'\nWeb interface: http://192.168.88.153:{port}')
    print('\n✅ SYSTEM READY!')
    print('Open browser to see live feed with wave detection')
    print('Press Ctrl+C to stop\n')
    
    # Announce startup
    os.system('espeak "Wave detection online with bridge!" 2>/dev/null')
    
    try:
        httpd.serve_forever()
    except KeyboardInterrupt:
        print('\n\nShutting down...')
        os.system('espeak "Goodbye!" 2>/dev/null')
        httpd.shutdown()
        camera_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()