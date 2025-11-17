#!/usr/bin/env python3
"""
Simple wave detection test - minimal code for debugging
"""

import sys
import time
import cv2
import numpy as np

# ROS2 imports
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from ultralytics import YOLO

class SimpleWaveTest(Node):
    def __init__(self):
        super().__init__('simple_wave_test')
        
        self.bridge = CvBridge()
        self.yolo_model = YOLO('/home/booster/yolov8n.pt')
        
        # State
        self.frame_count = 0
        self.person_detected = False
        self.last_positions = []
        
        # Subscribe
        self.subscription = self.create_subscription(
            Image,
            '/image_left_raw',
            self.camera_callback,
            10
        )
        
        print("="*50)
        print("SIMPLE WAVE TEST STARTED")
        print("1. Stand in front of camera")
        print("2. Wave your hand side to side")
        print("="*50)
        
        # Test TTS
        try:
            import os
            os.system('espeak "Wave test started" 2>/dev/null')
        except:
            pass
    
    def camera_callback(self, msg):
        try:
            # Process every 3rd frame
            self.frame_count += 1
            if self.frame_count % 3 != 0:
                return
            
            # Convert image
            if msg.encoding == 'nv12':
                height = msg.height
                width = msg.width
                img_data = np.frombuffer(msg.data, dtype=np.uint8)
                expected_size = int(height * 1.5 * width)
                if len(img_data) == expected_size:
                    yuv = img_data.reshape((int(height * 1.5), width))
                    frame = cv2.cvtColor(yuv, cv2.COLOR_YUV2BGR_NV12)
                else:
                    return
            else:
                frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            
            # Resize
            if frame.shape[1] > 640:
                scale = 640 / frame.shape[1]
                new_width = int(frame.shape[1] * scale)
                new_height = int(frame.shape[0] * scale)
                frame = cv2.resize(frame, (new_width, new_height))
            
            # Run YOLO
            results = self.yolo_model(frame, conf=0.3, verbose=False)
            
            # Check for people
            for result in results:
                if result.boxes is None:
                    continue
                
                for box in result.boxes:
                    cls = int(box.cls[0])
                    if cls == 0:  # Person
                        x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()
                        center_x = (x1 + x2) / 2
                        
                        # Track positions
                        self.last_positions.append(center_x)
                        if len(self.last_positions) > 10:
                            self.last_positions.pop(0)
                        
                        # Simple wave detection
                        if len(self.last_positions) >= 5:
                            movement = max(self.last_positions) - min(self.last_positions)
                            
                            if not self.person_detected:
                                print(f"[Frame {self.frame_count}] Person detected!")
                                self.person_detected = True
                                try:
                                    import os
                                    os.system('espeak "Hello! I see you!" 2>/dev/null')
                                except:
                                    pass
                            
                            if movement > 30:  # Wave threshold
                                print(f"[Frame {self.frame_count}] WAVE DETECTED! Movement: {movement:.1f}")
                                try:
                                    import os
                                    os.system('espeak "Nice wave!" 2>/dev/null')
                                except:
                                    pass
                                # Reset to avoid spam
                                self.last_positions = []
                            elif self.frame_count % 30 == 0:
                                print(f"[Frame {self.frame_count}] Person tracking... Movement: {movement:.1f}")
            
            # Log every 60 frames
            if self.frame_count % 60 == 0:
                print(f"Processed {self.frame_count} frames...")
                
        except Exception as e:
            print(f"Error: {e}")

def main():
    rclpy.init()
    node = SimpleWaveTest()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\nStopping...")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()