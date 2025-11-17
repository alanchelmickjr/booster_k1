#!/usr/bin/env python3
"""
Comprehensive camera diagnostics for K1 robot
Tests ROS2 topic subscription and data flow
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
import time
import numpy as np
import sys

class CameraDiagnostics(Node):
    def __init__(self):
        super().__init__('camera_diagnostics')
        
        print("[DIAG] Starting camera diagnostics...")
        
        # Diagnostic counters
        self.left_messages = 0
        self.right_messages = 0
        self.last_left_msg = None
        self.last_right_msg = None
        self.start_time = time.time()
        
        # Test different QoS profiles
        qos_profiles = {
            'BEST_EFFORT': QoSProfile(
                reliability=ReliabilityPolicy.BEST_EFFORT,
                history=HistoryPolicy.KEEP_LAST,
                depth=1
            ),
            'RELIABLE': QoSProfile(
                reliability=ReliabilityPolicy.RELIABLE,
                history=HistoryPolicy.KEEP_LAST,
                depth=10
            )
        }
        
        # Try different topic patterns
        topic_patterns = [
            '/booster_camera_bridge/image_left_raw',
            '/booster_camera_bridge/image_right_raw',
            '/camera/image_raw',
            '/camera/left/image_raw',
            '/camera/right/image_raw',
            '/image_left_raw',
            '/image_right_raw'
        ]
        
        print(f"[DIAG] Testing {len(topic_patterns)} topic patterns...")
        
        # Check which topics exist
        existing_topics = []
        try:
            topic_list = self.get_topic_names_and_types()
            for topic_name, topic_types in topic_list:
                if 'sensor_msgs' in str(topic_types):
                    existing_topics.append(topic_name)
                    print(f"[DIAG] Found sensor_msgs topic: {topic_name} - Types: {topic_types}")
        except Exception as e:
            print(f"[DIAG] Error getting topic list: {e}")
        
        # Subscribe to all camera-related topics with different QoS
        self.subscriptions = []
        for topic in topic_patterns:
            for qos_name, qos_profile in qos_profiles.items():
                try:
                    sub = self.create_subscription(
                        Image,
                        topic,
                        lambda msg, t=topic, q=qos_name: self.image_callback(msg, t, q),
                        qos_profile
                    )
                    self.subscriptions.append((topic, qos_name, sub))
                    print(f"[DIAG] Subscribed to {topic} with {qos_name} QoS")
                except Exception as e:
                    print(f"[DIAG] Failed to subscribe to {topic} with {qos_name}: {e}")
        
        # Create timer for periodic status updates
        self.timer = self.create_timer(2.0, self.print_status)
        
        print(f"[DIAG] Diagnostics initialized with {len(self.subscriptions)} subscriptions")
        print(f"[DIAG] Waiting for messages...")
    
    def image_callback(self, msg, topic, qos):
        """Handle incoming image messages"""
        current_time = time.time()
        
        if 'left' in topic.lower():
            self.left_messages += 1
            self.last_left_msg = current_time
            msg_type = "LEFT"
        elif 'right' in topic.lower():
            self.right_messages += 1
            self.last_right_msg = current_time
            msg_type = "RIGHT"
        else:
            msg_type = "UNKNOWN"
        
        # Log first message from each topic/QoS combination
        key = f"{topic}_{qos}"
        if not hasattr(self, f'first_{key}'):
            setattr(self, f'first_{key}', True)
            print(f"\n[DIAG] *** FIRST MESSAGE RECEIVED ***")
            print(f"[DIAG] Topic: {topic}")
            print(f"[DIAG] QoS: {qos}")
            print(f"[DIAG] Type: {msg_type}")
            print(f"[DIAG] Encoding: {msg.encoding}")
            print(f"[DIAG] Size: {msg.width}x{msg.height}")
            print(f"[DIAG] Data size: {len(msg.data)} bytes")
            
            # Check data integrity
            expected_size = self.calculate_expected_size(msg)
            actual_size = len(msg.data)
            
            if actual_size == expected_size:
                print(f"[DIAG] Data size CORRECT: {actual_size} bytes")
            else:
                print(f"[DIAG] Data size MISMATCH! Expected: {expected_size}, Got: {actual_size}")
            
            print(f"[DIAG] *** END FIRST MESSAGE ***\n")
    
    def calculate_expected_size(self, msg):
        """Calculate expected data size based on encoding"""
        width = msg.width
        height = msg.height
        
        encoding_sizes = {
            'nv12': int(width * height * 1.5),  # YUV 4:2:0
            'rgb8': width * height * 3,
            'bgr8': width * height * 3,
            'mono8': width * height,
            'yuv420': int(width * height * 1.5),
            'yuyv': width * height * 2,
        }
        
        return encoding_sizes.get(msg.encoding.lower(), -1)
    
    def print_status(self):
        """Print diagnostic status"""
        runtime = time.time() - self.start_time
        
        print(f"\n[DIAG] === Status Update (Runtime: {runtime:.1f}s) ===")
        print(f"[DIAG] Left messages: {self.left_messages}")
        print(f"[DIAG] Right messages: {self.right_messages}")
        
        if self.last_left_msg:
            time_since_left = time.time() - self.last_left_msg
            print(f"[DIAG] Time since last left: {time_since_left:.2f}s")
        else:
            print(f"[DIAG] No left messages received")
        
        if self.last_right_msg:
            time_since_right = time.time() - self.last_right_msg
            print(f"[DIAG] Time since last right: {time_since_right:.2f}s")
        else:
            print(f"[DIAG] No right messages received")
        
        if self.left_messages > 0:
            left_fps = self.left_messages / runtime
            print(f"[DIAG] Left FPS: {left_fps:.1f}")
        
        if self.right_messages > 0:
            right_fps = self.right_messages / runtime
            print(f"[DIAG] Right FPS: {right_fps:.1f}")
        
        print(f"[DIAG] === End Status ===\n")

def main():
    print("[MAIN] Initializing ROS2...")
    rclpy.init(args=None)
    
    print("[MAIN] Creating diagnostics node...")
    node = CameraDiagnostics()
    
    print("[MAIN] Starting spin...")
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\n[MAIN] Shutting down...")
    finally:
        node.destroy_node()
        rclpy.shutdown()
        
        # Print final summary
        print("\n[SUMMARY] === Final Diagnostics Summary ===")
        print(f"[SUMMARY] Total left messages: {node.left_messages}")
        print(f"[SUMMARY] Total right messages: {node.right_messages}")
        
        if node.left_messages == 0 and node.right_messages == 0:
            print("[SUMMARY] ❌ NO CAMERA MESSAGES RECEIVED!")
            print("[SUMMARY] Possible issues:")
            print("[SUMMARY] 1. Camera topics not publishing")
            print("[SUMMARY] 2. QoS mismatch between publisher and subscriber")
            print("[SUMMARY] 3. Network/ROS2 configuration issues")
            print("[SUMMARY] 4. Camera driver not running")
        elif node.left_messages > 0 or node.right_messages > 0:
            print("[SUMMARY] ✅ Camera messages received successfully!")
            print("[SUMMARY] The camera feed is working at the ROS2 level")
            print("[SUMMARY] Issues may be in the processing pipeline")
        
        print("[SUMMARY] === End Summary ===")

if __name__ == '__main__':
    main()