@echo off
setlocal
set OPENOCD_BIN=C:\Users\goats\AppData\Local\Microsoft\WinGet\Packages\xpack-dev-tools.openocd-xpack_Microsoft.Winget.Source_8wekyb3d8bbwe\xpack-openocd-0.12.0-7\bin
set PYTHON=C:\Users\goats\AppData\Local\Programs\Python\Python313\python.exe

REM BLE module (CC41-A) address -- connect-by-name would be nicer, but Windows
REM discovery usually doesn't surface this clone's advertised name.
set BT_TARGET=00:15:87:10:9E:53

REM Unified dashboard launcher: asks which link to use.
REM   1 = SWD: starts OpenOCD if needed, runs the dashboard, cleans up the
REM       OpenOCD it started (needs the ST-Link attached).
REM   2 = BLE via the CC41-A on the FC's USART2 (needs bleak).
REM Assumes the firmware is already flashed (pilot3_fw_build_flash.bat).

echo Pilot3 Dashboard
echo   1 = SWD (ST-Link)
echo   2 = BLE (CC41-A, %BT_TARGET%)
choice /c 12 /n /m "Select link [1/2]: "
if errorlevel 2 goto run_ble
goto run_swd

:run_swd
set STARTED_OPENOCD=
tasklist /FI "IMAGENAME eq openocd.exe" 2>nul | find /I "openocd.exe" >nul
if errorlevel 1 (
    echo Starting OpenOCD...
    start "OpenOCD - Pilot_3" /min "%OPENOCD_BIN%\openocd.exe" -f interface\stlink.cfg -f target\stm32f3x.cfg -c "adapter speed 950"
    set STARTED_OPENOCD=1
    ping -n 4 127.0.0.1 >nul
) else (
    echo OpenOCD already running -- reusing it.
)
cd /d "%~dp0"
"%PYTHON%" -u tools\pilot3_dashboard.py
echo.
echo Dashboard exited.
if defined STARTED_OPENOCD (
    echo Shutting down the OpenOCD instance this script started...
    taskkill /IM openocd.exe /F >nul 2>&1
)
pause
exit /b 0

:run_ble
REM Compile & Flash still works from the page, but spawns OpenOCD on demand
REM and needs the ST-Link attached at that moment.
cd /d "%~dp0"
"%PYTHON%" -u tools\pilot3_dashboard.py --ble %BT_TARGET%
echo.
echo Dashboard exited.
pause
exit /b 0
