#!/usr/bin/env python3
"""
PT Motorsport AU - USBtoCAN RTC Time Synchronizer
Syncs the device RTC with the PC's system time
"""

import serial
import serial.tools.list_ports
from datetime import datetime
import time
import sys


def find_device_port():
    """Auto-detect the USBtoCAN device on COM ports"""
    ports = serial.tools.list_ports.comports()
    
    if not ports:
        return None
    
    # Look for Arduino/STM32 devices
    for port in ports:
        if 'Arduino' in port.description or 'USB' in port.description:
            return port.device
    
    # If no specific match, return the first available port
    return ports[0].device if ports else None


def sync_rtc_time(port=None, baud=115200, timeout=2):
    """
    Synchronize RTC time on the device with PC system time
    
    Args:
        port: Serial port name (e.g., 'COM3' or '/dev/ttyACM0'). Auto-detects if None.
        baud: Baud rate (default 115200)
        timeout: Serial read timeout in seconds
    
    Returns:
        bool: True if sync successful, False otherwise
    """
    
    # Auto-detect port if not specified
    if port is None:
        port = find_device_port()
        if port is None:
            print("ERROR: No COM ports found. Please connect the device and try again.")
            return False
        print(f"Auto-detected device on port: {port}")
    
    try:
        # Open serial connection
        print(f"Connecting to {port} at {baud} baud...")
        ser = serial.Serial(port, baud, timeout=timeout)
        ser.reset_input_buffer()
        ser.reset_output_buffer()
        
        # Wait for device startup
        print("Waiting for device startup...")
        time.sleep(2)
        
        # Get current system date and time
        now = datetime.now()
        date_str = now.strftime('%Y-%m-%d')
        time_str = now.strftime('%H:%M:%S')
        
        # Format command
        sync_cmd = f"RTC SYNC {date_str} {time_str}\n"
        
        print(f"Sending: {sync_cmd.strip()}")
        ser.write(sync_cmd.encode())
        
        # Read response
        print("Waiting for response...")
        response_line = ""
        start_time = time.time()
        
        while time.time() - start_time < timeout:
            if ser.in_waiting:
                char = ser.read(1).decode('utf-8', errors='ignore')
                if char == '\n':
                    break
                response_line += char
        
        response_line = response_line.strip()
        
        # Close connection
        ser.close()
        
        # Check response
        if response_line:
            print(f"Device response: {response_line}")
            if response_line.startswith("OK"):
                print("✓ RTC synchronization successful!")
                return True
            elif response_line.startswith("ERR"):
                print(f"✗ Device error: {response_line}")
                return False
            else:
                print(f"✓ Device response received: {response_line}")
                return True
        else:
            print("⚠ No response from device (port may be disconnected)")
            return False
    
    except serial.SerialException as e:
        print(f"ERROR: Serial port error - {e}")
        return False
    except Exception as e:
        print(f"ERROR: {e}")
        return False


def main():
    """Main entry point"""
    import argparse
    
    parser = argparse.ArgumentParser(
        description="Synchronize USBtoCAN device RTC with PC system time",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  python sync_rtc_time.py                    # Auto-detect COM port
  python sync_rtc_time.py --port COM3        # Specify COM port (Windows)
  python sync_rtc_time.py --port /dev/ttyACM0  # Specify COM port (Linux)
        """
    )
    
    parser.add_argument(
        '--port',
        default=None,
        help='Serial port (e.g., COM3 or /dev/ttyACM0). Auto-detects if omitted.'
    )
    parser.add_argument(
        '--baud',
        type=int,
        default=115200,
        help='Baud rate (default: 115200)'
    )
    parser.add_argument(
        '--timeout',
        type=float,
        default=2,
        help='Response timeout in seconds (default: 2)'
    )
    
    args = parser.parse_args()
    
    print("=" * 50)
    print("  PT Motorsport AU - RTC Time Synchronizer")
    print("=" * 50)
    print()
    
    success = sync_rtc_time(port=args.port, baud=args.baud, timeout=args.timeout)
    
    print()
    sys.exit(0 if success else 1)


if __name__ == '__main__':
    main()
