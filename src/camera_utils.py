#!/usr/bin/env python3
"""
Common utilities for handling camera feeds
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np

class CameraUtils(Node):
    def __init__(self, node_name: str):
        super().__init__(node_name)
        self.bridge = CvBridge()

    def create_subscription(self, topic: str, callback):
        return self.create_subscription(
            Image,
            topic,
            callback,
            10
        )

    def convert_nv12_to_bgr(self, msg):
        """Convert NV12 ROS image to BGR OpenCV image"""
        if msg.encoding == 'nv12':
            height = msg.height
            width = msg.width
            img_data = np.frombuffer(msg.data, dtype=np.uint8)
            yuv_image = img_data.reshape((height * 3 // 2, width))
            bgr_image = cv2.cvtColor(yuv_image, cv2.COLOR_YUV2BGR_NV12)
            return bgr_image
        else:
            return self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')