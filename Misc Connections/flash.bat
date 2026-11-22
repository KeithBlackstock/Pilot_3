@echo off
setlocal
set OPENOCD_BIN=C:\Users\goats\AppData\Local\Microsoft\WinGet\Packages\xpack-dev-tools.openocd-xpack_Microsoft.Winget.Source_8wekyb3d8bbwe\xpack-openocd-0.12.0-7\bin

cd /d "%~dp0"
echo WARNING: this temporarily replaces the Pilot_3 flight firmware.
echo Remove the propeller. The normal firmware must be reflashed afterward.
choice /c YN /n /m "Flash the ESP-AT bridge now? [Y/N]: "
if errorlevel 2 exit /b 1
if not exist build\esp_at_bridge.elf (
    echo Build output missing -- run build.bat first.
    exit /b 1
)
"%OPENOCD_BIN%\openocd.exe" -f interface\stlink.cfg -f target\stm32f3x.cfg -c "adapter speed 950" -c "program build/esp_at_bridge.elf verify reset exit"
if errorlevel 1 exit /b 1
echo ESP-AT bridge flashed. Run console.bat.
