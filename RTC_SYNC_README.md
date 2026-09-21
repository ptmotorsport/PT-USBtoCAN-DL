# RTC Time Synchronization Scripts

These scripts automatically synchronize the USBtoCAN device's Real-Time Clock (RTC) with your PC's system time.

## Files

- **sync_rtc_time.py** - Main Python script (cross-platform)
- **sync_rtc_time.bat** - Windows batch wrapper (one-click usage)
- **sync_rtc_time.sh** - Linux/Mac shell wrapper (one-click usage)

## Quick Start

### Windows
1. Connect the USBtoCAN device to your PC via USB
2. Double-click **sync_rtc_time.bat**
3. Wait for confirmation message

### Linux/Mac
1. Connect the USBtoCAN device to your PC via USB
2. Open terminal in this directory
3. Run: `./sync_rtc_time.sh`
4. Or run: `python3 sync_rtc_time.py`

## Prerequisites

- **Python 3.x** installed and in your system PATH
- **pyserial** library: `pip install pyserial`

## Usage

### Basic (Auto-detect port)
```bash
python sync_rtc_time.py
```

### Specify COM port (Windows)
```bash
python sync_rtc_time.py --port COM3
```

### Specify Serial port (Linux/Mac)
```bash
python sync_rtc_time.py --port /dev/ttyACM0
```

### Custom baud rate (if needed)
```bash
python sync_rtc_time.py --port COM3 --baud 115200
```

## How It Works

1. **Connects** to the device via serial port (auto-detects if not specified)
2. **Waits** for device startup (~2 seconds)
3. **Gets** current date and time from your PC
4. **Sends** `RTC SYNC YYYY-MM-DD HH:MM:SS` command to device
5. **Receives** confirmation from device
6. **Closes** connection

## What the Device Responds

- **Success**: `OK: RTC synced to YYYY-MM-DD HH:MM:SS`
- **Error**: `ERR: <error message>`

## Troubleshooting

### "No COM ports found"
- Make sure the device is connected to USB
- Check Device Manager to find the correct COM port
- Use `--port` option to specify manually

### "No response from device"
- Device may have crashed; try disconnecting and reconnecting USB
- Check if the serial monitor is already open (close it first)

### Python/pyserial not found
- Install Python: https://www.python.org/downloads/
- Install pyserial: `pip install pyserial`

### Permission denied (Linux/Mac)
- Add execute permission: `chmod +x sync_rtc_time.sh`
- Or run with sudo: `sudo python3 sync_rtc_time.py`

## Finding Your COM Port

### Windows
1. Open Device Manager
2. Look under "Ports (COM & LPT)"
3. Find "Arduino Uno R4 Minima" or similar
4. Note the COM number (e.g., COM3)

### Linux
```bash
ls /dev/tty*
# Look for /dev/ttyACM0 or /dev/ttyUSB0
```

### Mac
```bash
ls /dev/tty.usbmodem*
# Or
ls /dev/tty.usbserial*
```

## Integration with Development Workflow

### PlatformIO (platformio.ini)
Add this to sync time before uploading:
```ini
[env:uno_r4_minima]
extra_scripts = pre:sync_time.py
```

### Manual Integration
Run the script in your development routine:
1. Build firmware
2. Upload to device
3. Run: `python sync_rtc_time.py`
4. Test with `RTC` command in serial monitor

## Notes

- The device will use PC time regardless of what was previously stored
- Time persists in RTC even after power cycle (due to external crystal)
- Script uses Monday as default day-of-week (will be updated on next full sync)
- Daylight Saving Time is set to INACTIVE
