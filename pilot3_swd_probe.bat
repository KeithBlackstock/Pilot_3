@echo off
setlocal
set OPENOCD_BIN=C:\Users\goats\AppData\Local\Microsoft\WinGet\Packages\xpack-dev-tools.openocd-xpack_Microsoft.Winget.Source_8wekyb3d8bbwe\xpack-openocd-0.12.0-7\bin
set PYTHON=C:\Users\goats\AppData\Local\Programs\Python\Python313\python.exe

echo Killing any stale OpenOCD instance so the ST-Link is free...
taskkill /IM openocd.exe /F >nul 2>&1

echo Starting OpenOCD (SWD, stm32f3x cfg for the GD32F303's Cortex-M4, target left running)...
cd /d "%OPENOCD_BIN%"
start "OpenOCD - FC_bluepill" /min openocd.exe -f interface\stlink.cfg -f target\stm32f3x.cfg -c "adapter speed 950"

REM give OpenOCD a moment to attach and open its telnet port (4444)
ping -n 4 127.0.0.1 >nul

echo.
echo Probing FC_bluepill devices: GD32F303 MCU, ICM-42688-P IMU (SPI2), BMP280 baro (I2C1).
echo PA15 is grounded on this board -- the probe never touches GPIOA or AFIO.
echo.
cd /d "%~dp0"
"%PYTHON%" -u pilot3_swd_probe.py

echo.
echo Probe finished. Shutting down OpenOCD...
taskkill /IM openocd.exe /F >nul 2>&1
echo Done.
pause
