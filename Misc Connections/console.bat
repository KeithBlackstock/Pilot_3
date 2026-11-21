@echo off
setlocal
set OPENOCD_BIN=C:\Users\goats\AppData\Local\Microsoft\WinGet\Packages\xpack-dev-tools.openocd-xpack_Microsoft.Winget.Source_8wekyb3d8bbwe\xpack-openocd-0.12.0-7\bin
set PYTHON=C:\Users\goats\AppData\Local\Programs\Python\Python313\python.exe
set STARTED_OPENOCD=

tasklist /FI "IMAGENAME eq openocd.exe" 2>nul | find /I "openocd.exe" >nul
if errorlevel 1 (
    echo Starting OpenOCD...
    start "OpenOCD - ESP-AT bridge" /min "%OPENOCD_BIN%\openocd.exe" -f interface\stlink.cfg -f target\stm32f3x.cfg -c "adapter speed 950"
    set STARTED_OPENOCD=1
    ping -n 4 127.0.0.1 >nul
) else (
    echo OpenOCD already running -- reusing it.
)

cd /d "%~dp0"
"%PYTHON%" -u esp_at_console.py %*

if defined STARTED_OPENOCD (
    echo Shutting down the OpenOCD instance this script started...
    taskkill /IM openocd.exe /F >nul 2>&1
)
pause
