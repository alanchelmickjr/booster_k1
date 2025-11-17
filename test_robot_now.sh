#!/bin/bash

echo "Connecting to K-1 robot and testing controls..."
echo ""
echo "INSTRUCTIONS:"
echo "1. Press 'i' to enter interactive mode"
echo "2. Type 'mw' and press Enter to switch to WALK mode"
echo "3. Then type 'w' to move forward"
echo "4. Type 'md' to return to Damping mode"
echo "5. Press Ctrl+C to exit"
echo ""
echo "Connecting..."

sshpass -p '123456' ssh -t booster@192.168.88.153 'cd /home/booster/booster_k1 && python3 src/basic_controls.py 127.0.0.1'