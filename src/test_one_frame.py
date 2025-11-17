#!/usr/bin/env python3
"""
Simple test to get one frame from camera and save it
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
import sys

class OneFrameTest(Node):
    def __init__(self):
        super().__init__('one_frame_test')
        self.bridge = CvBridge()
        self.got_frame = False
        
        print("Waiting for one frame from /image_left_raw...")
        
        self.subscription = self.create_subscription(
            Image,
            '/image_left_raw',
            self.camera_callback,
            10
        )
    
    def camera_callback(self, msg):
        if self.got_frame:
            return
            
        print(f"Got frame! Encoding: {msg.encoding}, Size: {msg.width}x{msg.height}")
        
        try:
            if msg.encoding == 'nv12':
                height = msg.height
                width = msg.width
                img_data = np.frombuffer(msg.data, dtype=np.uint8)
                expected_size = int(height * 1.5 * width)
                
                print(f"NV12 data size: {len(img_data)}, expected: {expected_size}")
                
                if len(img_data) == expected_size:
                    yuv = img_data.reshape((int(height * 1.5), width))
                    frame = cv2.cvtColor(yuv, cv2.COLOR_YUV2BGR_NV12)
                    
                    # Save the frame
                    cv2.imwrite('/home/booster/test_frame.jpg', frame)
                    print("Frame saved to /home/booster/test_frame.jpg")
                    print("SUCCESS! Camera is working!")
                    self.got_frame = True
                else:
                    print(f"ERROR: Size mismatch!")
            else:
                frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
                cv2.imwrite('/home/booster/test_frame.jpg', frame)
                print("Frame saved to /home/booster/test_frame.jpg")
                print("SUCCESS! Camera is working!")
                self.got_frame = True
                
        except Exception as e:
            print(f"ERROR converting frame: {e}")

def main():
    rclpy.init()
    node = OneFrameTest()
    
    # Spin for a few seconds
    import time
    start = time.time()
    while time.time() - start < 5 and not node.got_frame:
        rclpy.spin_once(node, timeout_sec=0.1)
    
    if not node.got_frame:
        print("FAILED: No frames received in 5 seconds!")
        print("Camera might be crashed or not publishing")
    
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()