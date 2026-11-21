@echo off
setlocal
REM Build the temporary ESP-AT bridge, not the normal flight firmware.

set TOOLCHAIN=C:\Program Files (x86)\GNU Arm Embedded Toolchain\10 2021.10\bin
set CXX=%TOOLCHAIN%\arm-none-eabi-g++.exe
set AS=%TOOLCHAIN%\arm-none-eabi-gcc.exe
set OBJCOPY=%TOOLCHAIN%\arm-none-eabi-objcopy.exe
set SIZE=%TOOLCHAIN%\arm-none-eabi-size.exe

set MCU_FLAGS=-mcpu=cortex-m4 -mthumb -mfpu=fpv4-sp-d16 -mfloat-abi=hard
set CXXFLAGS=%MCU_FLAGS% -std=c++17 -Os -ffunction-sections -fdata-sections -fno-exceptions -fno-rtti -Wall -I..\firmware\src
set ASFLAGS=%MCU_FLAGS%

cd /d "%~dp0"
if not exist build mkdir build

echo Assembling startup...
"%AS%" %ASFLAGS% -c ..\firmware\startup\startup_gd32f303.s -o build\startup_gd32f303.o
if errorlevel 1 goto :fail

echo Compiling ESP-AT bridge...
"%CXX%" %CXXFLAGS% -c main.cpp -o build\main.o
if errorlevel 1 goto :fail

echo Linking...
"%CXX%" %MCU_FLAGS% -T..\firmware\linker\gd32f303cc.ld -nostartfiles -Wl,--gc-sections -Wl,-Map=build\esp_at_bridge.map build\startup_gd32f303.o build\main.o -o build\esp_at_bridge.elf
if errorlevel 1 goto :fail

"%OBJCOPY%" -O binary build\esp_at_bridge.elf build\esp_at_bridge.bin
"%SIZE%" build\esp_at_bridge.elf

echo.
echo Build OK: build\esp_at_bridge.elf
exit /b 0

:fail
echo.
echo Build FAILED.
exit /b 1
