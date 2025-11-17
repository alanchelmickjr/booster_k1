#!/usr/bin/env python3
"""
Enhanced YOLO Person Detection with Wave Recognition and TTS
Stable camera handling with automatic recovery
Fixed ROS2 initialization
"""

import os
import sys
import time
import threading
from collections import deque

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


class YOLOWaveNode(Node):
    """ROS2 node for YOLO person detection with wave recognition"""
    
    def __init__(self):
        super().__init__('yolo_wave_detector')
        
        # Initialize components
        self.bridge = CvBridge()
        self.yolo_model = YOLO('/home/booster/yolov8n.pt')
        self.tts = TextToSpeech(engine='auto')
        
        # State tracking
        self.latest_frame = None
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
        except Exception as e:
            self.get_logger().error(f'Failed to create subscription: {e}')
            raise
        
        # Announce startup
        if self.tts.available:
            self.tts.speak("Wave detection system online. Wave to say hello!", blocking=False)
    
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
            
            # Run YOLO detection
            try:
                results = self.yolo_model(frame, conf=0.5, verbose=False)
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
                        
                        # Draw bounding box
                        color = (0, 255, 0)
                        cv2.rectangle(frame, (x1, y1), (x2, y2), color, 2)
                        
                        # Simple wave detection based on hand position
                        person_height = y2 - y1
                        person_width = x2 - x1
                        
                        # Check if person is large enough in frame
                        if person_height > 100:
                            # Check upper portion for raised hands
                            upper_y = y1 + int(person_height * 0.4)
                            
                            # Store position for wave detection
                            center_x = (x1 + x2) // 2
                            self.wave_history.append(center_x)
                            
                            # Detect wave if person moves side to side
                            if len(self.wave_history) >= 5:
                                movement = max(self.wave_history) - min(self.wave_history)
                                if movement > 30:  # Significant horizontal movement
                                    wave_detected = True
                        
                        # Add label
                        label = f"Person {conf:.2f}"
                        if wave_detected:
                            label += " - Waving!"
                            color = (0, 255, 255)
                        
                        cv2.putText(frame, label, (x1, y1-10),
                                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, color, 2)
            
            # Update people count
            self.people_count = people_detected
            
            # Handle greetings
            if people_detected > 0:
                self._handle_greeting(people_detected, wave_detected)
            
            # Add stats overlay
            cv2.putText(frame, f'FPS: {self.fps:.1f}', (10, 30),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
            cv2.putText(frame, f'People: {people_detected}', (10, 60),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
            
            if wave_detected:
                cv2.putText(frame, 'WAVE DETECTED!', (10, 90),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            
            # Store latest frame
            self.latest_frame = frame
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


def main():
    """Main entry point"""
    print("Starting YOLO Wave Detection System...")
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
        
        print("System initialized!")
        print("- Person detection: Active")
        print("- Wave detection: Active") 
        print("- TTS greetings: Active" if node.tts.available else "- TTS greetings: Disabled")
        print("- Camera topic: /image_left_raw")
        print("="*60)
        print("Wave at the camera to interact!")
        print("Press Ctrl+C to stop")
        
        # Spin node
        rclpy.spin(node)
        
    except KeyboardInterrupt:
        print("\nShutting down...")
        if 'node' in locals() and node.tts.available:
            node.tts.speak("Goodbye!", blocking=True)
    except Exception as e:
        print(f"Error running node: {e}")
        return 1
    finally:
        # Cleanup
        if 'node' in locals():
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    
    return 0


if __name__ == '__main__':
    sys.exit(main())