#!/usr/bin/env python3
"""
Web-based Camera Feed Viewer for Booster Camera Bridge
Uses /booster_camera_bridge topics directly
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
import io
from datetime import datetime

class CameraSubscriber(Node):
    """ROS2 node that subscribes to booster camera bridge topics"""

    def __init__(self, show_stereo=False):
        super().__init__('camera_web_viewer')

        self.bridge = CvBridge()
        self.latest_left_frame = None
        self.latest_right_frame = None
        self.show_stereo = show_stereo

        # Use booster camera bridge topics
        left_topic = '/booster_camera_bridge/image_left_raw'
        right_topic = '/booster_camera_bridge/image_right_raw'

        self.get_logger().info(f'Left camera topic: {left_topic}')
        if show_stereo:
            self.get_logger().info(f'Right camera topic: {right_topic}')

        # Subscribe to left camera
        self.left_subscription = self.create_subscription(
            Image,
            left_topic,
            self.left_callback,
            10
        )

        # Subscribe to right camera if requested
        if show_stereo:
            self.right_subscription = self.create_subscription(
                Image,
                right_topic,
                self.right_callback,
                10
            )

    def left_callback(self, msg):
        """Callback for left camera image messages"""
        try:
            # Handle NV12 encoding
            if msg.encoding == 'nv12':
                # NV12 is YUV 4:2:0 format
                # First height*width bytes are Y, next (height/2)*width bytes are UV
                height = msg.height
                width = msg.width

                # Convert ROS message data to numpy array
                img_data = np.frombuffer(msg.data, dtype=np.uint8)

                # Reshape to NV12 format
                yuv = img_data.reshape((int(height * 1.5), width))

                # Convert NV12 to BGR
                cv_image = cv2.cvtColor(yuv, cv2.COLOR_YUV2BGR_NV12)
                self.latest_left_frame = cv_image
            else:
                # Use cv_bridge for other encodings
                cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
                self.latest_left_frame = cv_image
        except Exception as e:
            self.get_logger().error(f'Error converting left image: {str(e)}')

    def right_callback(self, msg):
        """Callback for right camera image messages"""
        try:
            # Handle NV12 encoding
            if msg.encoding == 'nv12':
                # NV12 is YUV 4:2:0 format
                # First height*width bytes are Y, next (height/2)*width bytes are UV
                height = msg.height
                width = msg.width

                # Convert ROS message data to numpy array
                img_data = np.frombuffer(msg.data, dtype=np.uint8)

                # Reshape to NV12 format
                yuv = img_data.reshape((int(height * 1.5), width))

                # Convert NV12 to BGR
                cv_image = cv2.cvtColor(yuv, cv2.COLOR_YUV2BGR_NV12)
                self.latest_right_frame = cv_image
            else:
                # Use cv_bridge for other encodings
                cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
                self.latest_right_frame = cv_image
        except Exception as e:
            self.get_logger().error(f'Error converting right image: {str(e)}')


class CameraHTTPHandler(BaseHTTPRequestHandler):
    """HTTP request handler for serving camera images"""

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
                <title>Booster Camera Feed</title>
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
                    .feed-container {
                        display: flex;
                        gap: 20px;
                        flex-wrap: wrap;
                        justify-content: center;
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
                        max-width: 100%;
                        height: auto;
                        display: block;
                        border-radius: 4px;
                    }
                </style>
            </head>
            <body>
                <h1>Booster Camera Feed</h1>
                <div class="info">Auto-refreshing feed</div>
                <div class="feed-container">
                    <div class="feed-box">
                        <h2>Left Camera</h2>
                        <img id="left-feed" src="/left" alt="Left Camera">
                    </div>
                    """ + ("""
                    <div class="feed-box">
                        <h2>Right Camera</h2>
                        <img id="right-feed" src="/right" alt="Right Camera">
                    </div>
                    """ if self.camera_node and self.camera_node.show_stereo else "") + """
                </div>
                <script>
                    function refreshImage(id, src) {
                        const img = document.getElementById(id);
                        const newSrc = src + '?t=' + new Date().getTime();
                        img.src = newSrc;
                    }

                    setInterval(() => {
                        refreshImage('left-feed', '/left');
                    }, 100);

                    """ + ("""
                    setInterval(() => {
                        refreshImage('right-feed', '/right');
                    }, 100);
                    """ if self.camera_node and self.camera_node.show_stereo else "") + """
                </script>
            </body>
            </html>
            """
            self.wfile.write(html.encode())

        elif self.path.startswith('/left'):
            # Serve left camera image
            if self.camera_node and self.camera_node.latest_left_frame is not None:
                self.send_response(200)
                self.send_header('Content-type', 'image/jpeg')
                self.send_header('Cache-Control', 'no-cache, no-store, must-revalidate')
                self.send_header('Pragma', 'no-cache')
                self.send_header('Expires', '0')
                self.end_headers()

                _, buffer = cv2.imencode('.jpg', self.camera_node.latest_left_frame,
                                        [cv2.IMWRITE_JPEG_QUALITY, 85])
                self.wfile.write(buffer.tobytes())
            else:
                self.send_error(503, 'No left camera feed available')

        elif self.path.startswith('/right'):
            # Serve right camera image
            if self.camera_node and self.camera_node.latest_right_frame is not None:
                self.send_response(200)
                self.send_header('Content-type', 'image/jpeg')
                self.send_header('Cache-Control', 'no-cache, no-store, must-revalidate')
                self.send_header('Pragma', 'no-cache')
                self.send_header('Expires', '0')
                self.end_headers()

                _, buffer = cv2.imencode('.jpg', self.camera_node.latest_right_frame,
                                        [cv2.IMWRITE_JPEG_QUALITY, 85])
                self.wfile.write(buffer.tobytes())
            else:
                self.send_error(503, 'No right camera feed available')
        else:
            self.send_error(404, 'Not found')


def spin_ros(node):
    """Spin ROS2 node in a separate thread"""
    rclpy.spin(node)


def main():
    parser = argparse.ArgumentParser(description='Web-based camera feed viewer for Booster Camera Bridge')
    parser.add_argument('--stereo', action='store_true',
                       help='Show both left and right cameras')
    parser.add_argument('--host', type=str, default='0.0.0.0',
                       help='Host to bind web server to (default: 0.0.0.0)')
    parser.add_argument('--port', type=int, default=8080,
                       help='Port for web server (default: 8080)')

    args = parser.parse_args()

    # Initialize ROS2
    rclpy.init()

    # Create camera subscriber node
    camera_node = CameraSubscriber(show_stereo=args.stereo)

    # Set the camera node for the HTTP handler
    CameraHTTPHandler.camera_node = camera_node

    # Start ROS2 spinning in a separate thread
    ros_thread = threading.Thread(target=spin_ros, args=(camera_node,), daemon=True)
    ros_thread.start()

    # Start HTTP server
    server_address = (args.host, args.port)
    httpd = HTTPServer(server_address, CameraHTTPHandler)

    print(f'\n{"="*60}')
    print(f'Booster Camera Feed Viewer Started')
    print(f'{"="*60}')
    print(f'Using topics: /booster_camera_bridge/image_left_raw')
    if args.stereo:
        print(f'             /booster_camera_bridge/image_right_raw')
    print(f'Web server: http://{args.host}:{args.port}')
    print(f'\nOpen this URL in your browser to view the camera feed')
    print(f'Press Ctrl+C to stop')
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
