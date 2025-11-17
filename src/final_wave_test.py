#!/usr/bin/env python3
"""
Final simplified wave detector - just detection and TTS
"""

import os
import sys
import time
import cv2
import numpy as np

# Initialize ROS2 first
import rclpy
rclpy.init()

from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from ultralytics import YOLO

class FinalWaveTest(Node):
    def __init__(self):
        super().__init__('final_wave_test')
        
        self.bridge = CvBridge()
        print("Loading YOLO model...")
        self.yolo_model = YOLO('/home/booster/yolov8n.pt')
        print("YOLO model loaded!")
        
        # State
        self.frame_count = 0
        self.last_positions = []
        self.last_wave_time = 0
        self.last_greeting_time = 0
        
        # Subscribe
        self.subscription = self.create_subscription(
            Image,
            '/image_left_raw',
            self.camera_callback,
            10
        )
        
        print("="*50)
        print("WAVE DETECTOR RUNNING!")
        print("Stand in front of camera and wave!")
        print("="*50)
        
        # Initial greeting
        os.system('espeak "Wave detector ready! Wave at me!" 2>/dev/null')
    
    def camera_callback(self, msg):
        try:
            # Process every 3rd frame
            self.frame_count += 1
            if self.frame_count % 3 != 0:
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
            
            # Resize for performance
            if frame.shape[1] > 640:
                scale = 640 / frame.shape[1]
                frame = cv2.resize(frame, (int(frame.shape[1] * scale), int(frame.shape[0] * scale)))
            
            # Run YOLO
            results = self.yolo_model(frame, conf=0.25, verbose=False)
            
            # Process detections
            person_detected = False
            for result in results:
                if result.boxes is None:
                    continue
                
                for box in result.boxes:
                    cls = int(box.cls[0])
                    if cls == 0:  # Person
                        person_detected = True
                        x1, y1, x2, y2 = map(int, box.xyxy[0].cpu().numpy())
                        center_x = (x1 + x2) // 2
                        
                        # Track positions
                        self.last_positions.append(center_x)
                        if len(self.last_positions) > 10:
                            self.last_positions.pop(0)
                        
                        # Greet person once
                        current_time = time.time()
                        if current_time - self.last_greeting_time > 10:
                            print(f"[Frame {self.frame_count}] Person detected!")
                            os.system('espeak "Hello! I see you! Wave at me!" 2>/dev/null &')
                            self.last_greeting_time = current_time
                        
                        # Wave detection
                        if len(self.last_positions) >= 5:
                            movement = max(self.last_positions) - min(self.last_positions)
                            
                            if movement > 20:  # Wave detected
                                if current_time - self.last_wave_time > 5:
                                    print(f"*** WAVE DETECTED! Movement: {movement} ***")
                                    os.system('espeak "Great wave! Nice to meet you!" 2>/dev/null &')
                                    self.last_wave_time = current_time
                                    self.last_positions = []  # Reset
            
            # Status log
            if self.frame_count % 30 == 0:
                status = "Person in view" if person_detected else "No one detected"
                print(f"[Frame {self.frame_count}] {status}")
                
        except Exception as e:
            print(f"Error: {e}")

def main():
    try:
        node = FinalWaveTest()
        print("\nSystem active! Wave at the camera!\n")
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\nShutting down...")
        os.system('espeak "Goodbye!" 2>/dev/null')
    except Exception as e:
        print(f"Error: {e}")
    finally:
        if 'node' in locals():
            node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()