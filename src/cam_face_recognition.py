#!/usr/bin/env python3
"""
Web-based Camera Feed with Face Recognition
Integrates DeepFace recognition with ROS2 camera feed
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
from http.server import HTTPServer, BaseHTTPRequestHandler
import threading
import argparse
import time
from face_recognition import FaceRecognizer

class FaceRecognitionNode(Node):
    """ROS2 node that subscribes to camera and runs face recognition"""

    def __init__(self, recognizer: FaceRecognizer, auto_learn=False):
        super().__init__('camera_face_recognition')

        self.bridge = CvBridge()
        self.recognizer = recognizer
        self.auto_learn = auto_learn
        self.latest_frame = None
        self.latest_detections = []

        # Performance tracking
        self.fps = 0
        self.frame_count = 0
        self.last_fps_time = time.time()

        # Learning mode
        self.learning_mode = False
        self.learning_name = None

        # Use direct camera topics from mipi_cam node
        left_topic = '/image_left_raw'

        self.get_logger().info(f'Camera topic: {left_topic}')

        # Subscribe to left camera
        self.subscription = self.create_subscription(
            Image,
            left_topic,
            self.camera_callback,
            10
        )

    def convert_nv12_to_bgr(self, msg):
        """Convert NV12 ROS image to BGR OpenCV image"""
        if msg.encoding == 'nv12':
            height = msg.height
            width = msg.width
            img_data = np.frombuffer(msg.data, dtype=np.uint8)
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
        """Callback for camera image messages"""
        try:
            # Convert to BGR
            frame = self.convert_nv12_to_bgr(msg)

            # Detect faces
            faces = self.recognizer.detect_faces(frame)

            # Process each detected face
            detections = []
            for (x, y, w, h) in faces:
                # Extract face region
                face_img = frame[y:y+h, x:x+w]

                # Recognize face (if DeepFace available)
                name = "Unknown"
                confidence = 0.0

                if self.recognizer.deepface_available:
                    # In learning mode, add face to database
                    if self.learning_mode and self.learning_name:
                        self.recognizer.add_person(self.learning_name, face_img)
                        name = f"{self.learning_name} (learning)"
                        confidence = 1.0
                        # Exit learning mode after one capture
                        self.learning_mode = False
                        self.learning_name = None
                    else:
                        # Try to recognize
                        name, confidence = self.recognizer.recognize_face(face_img)
                        if name is None:
                            name = "Unknown"

                detections.append({
                    'bbox': (x, y, w, h),
                    'name': name,
                    'confidence': confidence
                })

                # Draw rectangle and label
                color = (0, 255, 0) if name != "Unknown" else (0, 165, 255)
                cv2.rectangle(frame, (x, y), (x+w, y+h), color, 2)

                # Draw name and confidence
                label = f"{name}"
                if confidence > 0:
                    label += f" ({confidence:.2f})"

                cv2.putText(frame, label, (x, y-10),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.7, color, 2)

            self.latest_detections = detections

            # Update FPS
            self.update_fps()

            # Add FPS and stats
            cv2.putText(frame, f'FPS: {self.fps:.1f}', (10, 30),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 255), 2)
            cv2.putText(frame, f'Faces: {len(faces)}', (10, 60),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 255), 2)

            # Show known people count
            known_people = len(self.recognizer.get_all_people())
            cv2.putText(frame, f'Known: {known_people}', (10, 90),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 255), 2)

            if self.learning_mode:
                cv2.putText(frame, f'LEARNING: {self.learning_name}', (10, 120),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)

            self.latest_frame = frame

        except Exception as e:
            self.get_logger().error(f'Error processing image: {str(e)}')

    def start_learning(self, name: str):
        """Start learning mode for a new person"""
        self.learning_mode = True
        self.learning_name = name
        self.get_logger().info(f"Started learning mode for: {name}")


class FaceRecognitionHTTPHandler(BaseHTTPRequestHandler):
    """HTTP request handler for serving face recognition feed"""

    camera_node = None

    def log_message(self, format, *args):
        """Override to suppress HTTP request logs"""
        pass

    def do_GET(self):
        """Handle GET requests"""
        if self.path == '/':
            # Serve HTML page
            self.send_response(200)
            self.send_header('Content-type', 'text/html')
            self.end_headers()

            html = """
            <!DOCTYPE html>
            <html>
            <head>
                <title>Face Recognition Feed</title>
                <style>
                    body {
                        font-family: Arial, sans-serif;
                        background-color: #1e1e1e;
                        color: #ffffff;
                        margin: 0;
                        padding: 20px;
                        display: flex;
                        flex-direction: column;
                        align-items: center;
                    }
                    h1 {
                        margin-bottom: 10px;
                    }
                    .info {
                        color: #888;
                        margin-bottom: 20px;
                    }
                    .controls {
                        background-color: #2d2d2d;
                        padding: 15px;
                        border-radius: 8px;
                        margin-bottom: 20px;
                        border: 2px solid #444;
                    }
                    .controls input {
                        padding: 8px;
                        margin-right: 10px;
                        border-radius: 4px;
                        border: 1px solid #555;
                        background-color: #1e1e1e;
                        color: #fff;
                    }
                    .controls button {
                        padding: 8px 16px;
                        background-color: #4CAF50;
                        color: white;
                        border: none;
                        border-radius: 4px;
                        cursor: pointer;
                    }
                    .controls button:hover {
                        background-color: #45a049;
                    }
                    .feed-box {
                        border: 2px solid #444;
                        padding: 10px;
                        border-radius: 8px;
                        background-color: #2d2d2d;
                    }
                    .feed-box h2 {
                        margin-top: 0;
                        margin-bottom: 10px;
                        font-size: 18px;
                        color: #4CAF50;
                    }
                    img {
                        max-width: 800px;
                        height: auto;
                        display: block;
                        border-radius: 4px;
                    }
                    .stats {
                        margin-top: 20px;
                        background-color: #2d2d2d;
                        padding: 15px;
                        border-radius: 8px;
                        border: 2px solid #444;
                        min-width: 300px;
                    }
                    .person-list {
                        list-style: none;
                        padding: 0;
                    }
                    .person-list li {
                        padding: 5px;
                        border-bottom: 1px solid #444;
                    }
                </style>
            </head>
            <body>
                <h1>Face Recognition System</h1>
                <div class="info">Real-time face detection and recognition</div>

                <div class="controls">
                    <input type="text" id="nameInput" placeholder="Enter name to learn">
                    <button onclick="learnFace()">Learn Face</button>
                    <button onclick="refreshStats()">Refresh Stats</button>
                </div>

                <div class="feed-box">
                    <h2>Camera Feed</h2>
                    <img id="camera-feed" src="/feed" alt="Camera Feed">
                </div>

                <div class="stats">
                    <h3>Known People</h3>
                    <div id="stats-content">Loading...</div>
                </div>

                <script>
                    function refreshImage() {
                        const img = document.getElementById('camera-feed');
                        const newSrc = '/feed?t=' + new Date().getTime();
                        img.src = newSrc;
                    }

                    function learnFace() {
                        const name = document.getElementById('nameInput').value;
                        if (name) {
                            fetch('/learn?name=' + encodeURIComponent(name))
                                .then(response => response.text())
                                .then(data => {
                                    alert(data);
                                    document.getElementById('nameInput').value = '';
                                    refreshStats();
                                });
                        } else {
                            alert('Please enter a name');
                        }
                    }

                    function refreshStats() {
                        fetch('/stats')
                            .then(response => response.json())
                            .then(data => {
                                let html = '<ul class="person-list">';
                                if (data.people.length === 0) {
                                    html += '<li>No people in database yet</li>';
                                } else {
                                    data.people.forEach(person => {
                                        html += '<li><strong>' + person + '</strong></li>';
                                    });
                                }
                                html += '</ul>';
                                document.getElementById('stats-content').innerHTML = html;
                            });
                    }

                    // Auto-refresh feed
                    setInterval(refreshImage, 100);

                    // Load stats on page load
                    refreshStats();
                    setInterval(refreshStats, 5000);
                </script>
            </body>
            </html>
            """
            self.wfile.write(html.encode())

        elif self.path.startswith('/feed'):
            # Serve camera feed
            if self.camera_node and self.camera_node.latest_frame is not None:
                self.send_response(200)
                self.send_header('Content-type', 'image/jpeg')
                self.send_header('Cache-Control', 'no-cache, no-store, must-revalidate')
                self.end_headers()

                _, buffer = cv2.imencode('.jpg', self.camera_node.latest_frame,
                                        [cv2.IMWRITE_JPEG_QUALITY, 85])
                self.wfile.write(buffer.tobytes())
            else:
                self.send_error(503, 'No feed available')

        elif self.path.startswith('/learn'):
            # Start learning mode
            import urllib.parse
            query = urllib.parse.urlparse(self.path).query
            params = urllib.parse.parse_qs(query)
            name = params.get('name', [''])[0]

            if name and self.camera_node:
                self.camera_node.start_learning(name)
                self.send_response(200)
                self.send_header('Content-type', 'text/plain')
                self.end_headers()
                self.wfile.write(f"Learning mode started for {name}".encode())
            else:
                self.send_error(400, 'Name required')

        elif self.path.startswith('/stats'):
            # Return database stats as JSON
            if self.camera_node:
                import json
                people = self.camera_node.recognizer.get_all_people()
                stats = {'people': people}

                self.send_response(200)
                self.send_header('Content-type', 'application/json')
                self.end_headers()
                self.wfile.write(json.dumps(stats).encode())
            else:
                self.send_error(503, 'Not available')

        else:
            self.send_error(404, 'Not found')


def spin_ros(node):
    """Spin ROS2 node in a separate thread"""
    rclpy.spin(node)


def main():
    parser = argparse.ArgumentParser(description='Face recognition on camera feed')
    parser.add_argument('--no-deepface', action='store_true',
                       help='Disable DeepFace (detection only)')
    parser.add_argument('--database', type=str, default='face_database.json',
                       help='Path to face database file')
    parser.add_argument('--host', type=str, default='0.0.0.0',
                       help='Host to bind web server to')
    parser.add_argument('--port', type=int, default=8080,
                       help='Port for web server')

    args = parser.parse_args()

    print(f'\n{"="*60}')
    print(f'Face Recognition Camera Viewer')
    print(f'{"="*60}')

    # Initialize face recognizer
    recognizer = FaceRecognizer(
        database_path=args.database,
        use_deepface=not args.no_deepface
    )

    # Initialize ROS2
    rclpy.init()

    # Create face recognition node
    camera_node = FaceRecognitionNode(recognizer=recognizer)

    # Set the camera node for the HTTP handler
    FaceRecognitionHTTPHandler.camera_node = camera_node

    # Start ROS2 spinning in a separate thread
    ros_thread = threading.Thread(target=spin_ros, args=(camera_node,), daemon=True)
    ros_thread.start()

    # Start HTTP server
    server_address = (args.host, args.port)
    httpd = HTTPServer(server_address, FaceRecognitionHTTPHandler)

    print(f'Database: {args.database}')
    print(f'Known people: {len(recognizer.get_all_people())}')
    print(f'Web server: http://{args.host}:{args.port}')
    print(f'\nOpen this URL in your browser to:')
    print(f'  - View face recognition feed')
    print(f'  - Learn new faces')
    print(f'  - See database statistics')
    print(f'\nPress Ctrl+C to stop')
    print(f'{"="*60}\n')

    try:
        httpd.serve_forever()
    except KeyboardInterrupt:
        print('\n\nShutting down...')
        httpd.shutdown()
        camera_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
