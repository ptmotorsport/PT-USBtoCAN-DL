@echo off
REM PT Motorsport AU - RTC Synchronizer Batch Script (Windows)
REM Run this to sync the USBtoCAN device RTC with PC time

echo.
echo ============================================
echo   PT Motorsport AU - RTC Time Synchronizer
echo ============================================
echo.

python sync_rtc_time.py %*

if errorlevel 1 (
    echo.
    echo ERROR: Synchronization failed.
    echo.
    echo Troubleshooting:
    echo - Make sure the device is connected to a COM port
    echo - Verify Python is installed and in your PATH
    echo - Try specifying the port manually: sync_rtc_time.bat --port COM3
    echo.
    pause
) else (
    echo.
    echo Synchronization complete. Closing in 5 seconds...
    timeout /t 5 /nobreak
)
