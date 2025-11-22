#!/bin/bash

echo "============================================"
echo "   K-1 Camera Driver Repair Tool"
echo "============================================"

echo "1. Attempting to reload UVC video drivers..."
if lsmod | grep -q uvcvideo; then
    echo "   Unloading uvcvideo..."
    echo "123456" | sudo -S modprobe -r uvcvideo
    sleep 2
fi

echo "   Loading uvcvideo..."
echo "123456" | sudo -S modprobe uvcvideo
sleep 2

echo "2. Checking for video devices..."
if ls -l /dev/video* 2>/dev/null; then
    echo "✅ SUCCESS: Video devices found!"
else
    echo "❌ FAILURE: No video devices found after driver reload."
    echo "3. Attempting TEGRA USB Controller Reset (Target: 3610000.usb)..."
    echo "⚠️  WARNING: This may disconnect the network connection if Ethernet is USB-based."
    
    USB_DRIVER_PATH="/sys/bus/platform/drivers/tegra-xusb"
    DEVICE_ID="3610000.usb"
    
    if [ -d "$USB_DRIVER_PATH" ]; then
        echo "   Resetting controller $DEVICE_ID..."
        # We run this in a way that tries to persist if network cuts
        echo "123456" | sudo -S sh -c "echo $DEVICE_ID > $USB_DRIVER_PATH/unbind; sleep 2; echo $DEVICE_ID > $USB_DRIVER_PATH/bind"
        sleep 5
        
        if ls -l /dev/video* 2>/dev/null; then
             echo "✅ SUCCESS: USB Reset recovered camera!"
        else
             echo "❌ FAILURE: Camera still missing after USB reset."
        fi
    else
        echo "   Tegra XUSB driver path not found at $USB_DRIVER_PATH"
    fi
fi

echo "4. Restarting Camera Bridge (if camera exists)..."
if ls -l /dev/video* 2>/dev/null; then
    BRIDGE_PATH="/opt/booster/DaemonPerception/bin/booster-camera-bridge"
    if [ -f "$BRIDGE_PATH" ]; then
        echo "   Stopping old bridge..."
        echo "123456" | sudo -S pkill -f booster-camera-bridge
        sleep 2
        echo "   Starting new bridge..."
        echo "123456" | sudo -S nohup "$BRIDGE_PATH" > /dev/null 2>&1 &
        echo "   Bridge restarted."
    else
        echo "   Could not find bridge binary at $BRIDGE_PATH"
    fi
else
    echo "⚠️  Skipping bridge restart (no camera)."
    echo "   RECOMMENDATION: Full System Reboot"
fi

echo "============================================"