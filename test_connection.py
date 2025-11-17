#!/usr/bin/env python3
"""
Simple connection test for K-1 robot
Tests if we can connect to the robot SDK
"""

from booster_robotics_sdk_python import B1LocoClient, ChannelFactory, GetModeResponse, RobotMode
import sys
import time

def test_connection():
    print("K-1 Robot Connection Test")
    print("="*50)
    
    # Initialize with localhost
    print("Initializing SDK with 127.0.0.1...")
    ChannelFactory.Instance().Init(0, "127.0.0.1")
    
    client = B1LocoClient()
    client.Init()
    
    # Wait a moment for connection
    time.sleep(1)
    
    # Try to get robot mode
    print("Attempting to get robot mode...")
    gm = GetModeResponse()
    res = client.GetMode(gm)
    
    if res == 0:
        mode_names = {
            RobotMode.kDamping: "Damping",
            RobotMode.kPrepare: "Prepare", 
            RobotMode.kWalking: "Walking",
            RobotMode.kCustom: "Custom",
            RobotMode.kUnknown: "Unknown"
        }
        mode_name = mode_names.get(gm.mode, f"Unknown ({gm.mode})")
        print(f"✓ SUCCESS! Robot mode: {mode_name}")
        return True
    else:
        print(f"✗ FAILED! Error code: {res}")
        return False

if __name__ == "__main__":
    success = test_connection()
    sys.exit(0 if success else 1)