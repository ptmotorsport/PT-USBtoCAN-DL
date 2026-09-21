#!/bin/bash
# PT Motorsport AU - RTC Synchronizer Shell Script (Linux/Mac)
# Run this to sync the USBtoCAN device RTC with PC time

echo ""
echo "============================================"
echo "  PT Motorsport AU - RTC Time Synchronizer"
echo "============================================"
echo ""

python3 sync_rtc_time.py "$@"

if [ $? -ne 0 ]; then
    echo ""
    echo "ERROR: Synchronization failed."
    echo ""
    echo "Troubleshooting:"
    echo "- Make sure the device is connected"
    echo "- Verify Python3 is installed"
    echo "- Try specifying the port manually: ./sync_rtc_time.sh --port /dev/ttyACM0"
    echo ""
else
    echo ""
    echo "Synchronization complete."
    echo ""
fi
