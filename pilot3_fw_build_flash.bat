@echo off
setlocal
set OPENOCD_BIN=C:\Users\goats\AppData\Local\Microsoft\WinGet\Packages\xpack-dev-tools.openocd-xpack_Microsoft.Winget.Source_8wekyb3d8bbwe\xpack-openocd-0.12.0-7\bin
set PYTHON=C:\Users\goats\AppData\Local\Programs\Python\Python313\python.exe

cd /d "%~dp0firmware"
call ".\build.bat"
if errorlevel 1 (
    echo Build failed -- not flashing.
    exit /b 1
)

echo Killing any stale OpenOCD instance so the ST-Link is free...
taskkill /IM openocd.exe /F >nul 2>&1

REM OpenOCD's `program` command runs through its Tcl interpreter, which
REM treats backslash as an escape char (\f = form feed, \b = backspace) --
REM a Windows backslash path corrupts silently. Forward slashes are safe
REM on Windows and avoid this entirely.
set "ELF_PATH=%~dp0firmware\build\pilot3_fw.elf"
set "ELF_PATH=%ELF_PATH:\=/%"

echo Flashing %ELF_PATH% via SWD (one-shot instance, exits after reset)...
cd /d "%OPENOCD_BIN%"
"%OPENOCD_BIN%\openocd.exe" -f interface\stlink.cfg -f target\stm32f3x.cfg ^
    -c "program \"%ELF_PATH%\" verify reset exit"
if errorlevel 1 (
    echo Flash failed.
    exit /b 1
)

echo.
echo Starting a persistent OpenOCD instance for telemetry monitoring...
start "OpenOCD - Pilot_3" /min "%OPENOCD_BIN%\openocd.exe" -f interface\stlink.cfg -f target\stm32f3x.cfg -c "adapter speed 950"
ping -n 4 127.0.0.1 >nul

cd /d "%~dp0"
"%PYTHON%" -u tools\pilot3_attitude_monitor.py

echo.
echo Monitor exited. Shutting down OpenOCD...
taskkill /IM openocd.exe /F >nul 2>&1
pause
