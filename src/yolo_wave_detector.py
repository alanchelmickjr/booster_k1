#!/usr/bin/env python3
"""
Enhanced YOLO Person Detection with Wave Recognition and TTS
Stable camera handling with automatic recovery
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
from ultralytics import YOLO
import time
import threading
from collections import deque
import os
import sys

# Add parent directory to path for imports
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from src.tts_module import TextToSpeech

class WaveDetector:
    """Detect waving gestures from person keypoints"""
    
    def __init__(self):
        self.wave_history = deque(maxlen=15)  # Track hand positions over time
        self.last_wave_time = 0
        self.wave_cooldown = 5.0  # Don't detect waves too frequently
        
    def detect_wave(self, keypoints):
        """
        Detect if a person is waving based on hand movement
        COCO keypoints: 5=left_shoulder, 6=right_shoulder, 
                       9=left_wrist, 10=right_wrist
        """
        if keypoints is None or len(keypoints) < 11:
            return False
            
        # Get wrist and shoulder positions
        left_wrist = keypoints[9] if len(keypoints) > 9 else None
        right_wrist = keypoints[10] if len(keypoints) > 10 else None
        left_shoulder = keypoints[5] if len(keypoints) > 5 else None
        right_shoulder = keypoints[6] if len(keypoints) > 6 else None
        
        # Check if any hand is raised above shoulder
        wave_detected = False
        
        if left_wrist and left_shoulder:
            if left_wrist[1] < left_shoulder[1] - 20:  # Wrist above shoulder
                self.wave_history.append(('left', left_wrist[0], left_wrist[1]))
                wave_detected = self._check_wave_motion('left')
                
        if right_wrist and right_shoulder:
            if right_wrist[1] < right_shoulder[1] - 20:  # Wrist above shoulder
                self.wave_history.append(('right', right_wrist[0], right_wrist[1]))
                wave_detected = wave_detected or self._check_wave_motion('right')
                
        return wave_detected
    
    def _check_wave_motion(self, hand):
        """Check if hand movement indicates waving"""
        if len(self.wave_history) < 5:
            return False
            
        # Get recent positions for this hand
        hand_positions = [(x, y) for (h, x, y) in self.wave_history if h == hand]
        
        if len(hand_positions) < 3:
            return False
            
        # Check for horizontal movement (waving)
        x_positions = [x for (x, y) in hand_positions[-5:]]
        x_movement = max(x_positions) - min(x_positions)
        
        # Wave detected if hand moves horizontally > 30 pixels
        if x_movement > 30:
            current_time = time.time()
            if current_time - self.last_wave_time > self.wave_cooldown:
                self.last_wave_time = current_time
                return True
                
        return False


class YOLOWaveNode(Node):
    """ROS2 node for YOLO person detection with wave recognition"""
    
    def __init__(self):
        super().__init__('yolo_wave_detector')
        
        # Initialize components
        self.bridge = CvBridge()
        self.yolo_model = YOLO('/home/booster/yolov8n.pt')
        self.wave_detector = WaveDetector()
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
        
        # Subscribe to camera topic
        self.subscription = self.create_subscription(
            Image,
            '/image_left_raw',
            self.camera_callback,
            10
        )
        
        self.get_logger().info('YOLO Wave Detector initialized')
        self.get_logger().info('Using camera topic: /image_left_raw')
        
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
            
            # Convert image
            if msg.encoding == 'nv12':
                # Handle NV12 encoding
                height = msg.height
                width = msg.width
                img_data = np.frombuffer(msg.data, dtype=np.uint8)
                
                # Careful with reshape to avoid memory issues
                try:
                    yuv = img_data.reshape((int(height * 1.5), width))
                    frame = cv2.cvtColor(yuv, cv2.COLOR_YUV2BGR_NV12)
                except Exception as e:
                    self.get_logger().error(f'NV12 conversion error: {e}')
                    self.error_count += 1
                    return
            else:
                frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            
            # Resize for performance (max 640 width)
            if frame.shape[1] > 640:
                scale = 640 / frame.shape[1]
                new_width = int(frame.shape[1] * scale)
                new_height = int(frame.shape[0] * scale)
                frame = cv2.resize(frame, (new_width, new_height))
            
            # Run YOLO detection
            results = self.yolo_model(frame, conf=0.5, verbose=False)
            
            # Process detections
            people_detected = 0
            wave_detected = False
            
            for result in results:
                boxes = result.boxes
                keypoints = result.keypoints if hasattr(result, 'keypoints') else None
                
                for i, box in enumerate(boxes):
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
                        
                        # Check for wave gesture (simplified detection)
                        # Since YOLOv8n doesn't have keypoints, we'll use hand position heuristic
                        person_height = y2 - y1
                        upper_body_y = y1 + person_height * 0.3
                        
                        # Check if hands are raised (simplified wave detection)
                        roi = frame[y1:int(upper_body_y), x1:x2]
                        if roi.size > 0:
                            # Simple motion detection in upper body region
                            # This is a placeholder - in production, use pose estimation
                            wave_detected = self._simple_wave_detection(roi)
                        
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
    
    def _simple_wave_detection(self, roi):
        """Simple heuristic for wave detection in upper body region"""
        # This is a placeholder - for better results, use MediaPipe or OpenPose
        # For now, just check for skin color in raised position
        
        if roi.size == 0:
            return False
            
        # Convert to HSV for skin detection
        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
        
        # Skin color range in HSV
        lower_skin = np.array([0, 20, 70], dtype=np.uint8)
        upper_skin = np.array([20, 255, 255], dtype=np.uint8)
        
        # Create skin mask
        skin_mask = cv2.inRange(hsv, lower_skin, upper_skin)
        
        # Check if significant skin area in upper part (potential raised hand)
        upper_half = skin_mask[:skin_mask.shape[0]//2, :]
        skin_pixels = cv2.countNonZero(upper_half)
        total_pixels = upper_half.size
        
        # If more than 5% is skin color in upper region, might be waving
        return skin_pixels > (total_pixels * 0.05)
    
    def _handle_greeting(self, people_count, wave_detected):
        """Handle TTS greetings for detected people"""
        if not self.tts.available:
            return
            
        current_time = time.time()
        
        # Check cooldown
        if current_time - self.last_greeting_time < self.greeting_cooldown:
            return
        
        # Greet based on detection
        if wave_detected:
            self.tts.speak("Hello! I see you waving! Nice to meet you!", blocking=False)
            self.last_greeting_time = current_time
        elif people_count == 1 and self.frame_count < 100:  # New person
            self.tts.speak("Hello there! Wave at me to say hi!", blocking=False)
            self.last_greeting_time = current_time
        elif people_count > 1 and self.frame_count < 100:
            self.tts.speak(f"Hello everyone! I see {people_count} people!", blocking=False)
            self.last_greeting_time = current_time


def main():
    """Main entry point"""
    print("Starting YOLO Wave Detection System...")
    print("="*60)
    
    # Initialize ROS2
    rclpy.init()
    
    # Create node
    node = YOLOWaveNode()
    
    print("System initialized!")
    print("- Person detection: Active")
    print("- Wave detection: Active")
    print("- TTS greetings: Active")
    print("- Camera topic: /image_left_raw")
    print("="*60)
    print("Wave at the camera to interact!")
    print("Press Ctrl+C to stop")
    
    try:
        # Spin node
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\nShutting down...")
        if node.tts.available:
            node.tts.speak("Goodbye!", blocking=True)
    finally:
        # Cleanup
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()