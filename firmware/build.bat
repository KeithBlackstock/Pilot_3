@echo off
setlocal
REM No `make` on this machine -- this runs the same compiler invocations
REM the Makefile describes, directly. Keep the two in sync if you edit one.

set TOOLCHAIN=C:\Program Files (x86)\GNU Arm Embedded Toolchain\10 2021.10\bin
set CXX=%TOOLCHAIN%\arm-none-eabi-g++.exe
set AS=%TOOLCHAIN%\arm-none-eabi-gcc.exe
set OBJCOPY=%TOOLCHAIN%\arm-none-eabi-objcopy.exe
set SIZE=%TOOLCHAIN%\arm-none-eabi-size.exe

set MCU_FLAGS=-mcpu=cortex-m4 -mthumb -mfpu=fpv4-sp-d16 -mfloat-abi=hard
set CXXFLAGS=%MCU_FLAGS% -std=c++17 -Os -ffunction-sections -fdata-sections -fno-exceptions -fno-rtti -Wall -Isrc
set ASFLAGS=%MCU_FLAGS%

cd /d "%~dp0"
if not exist build mkdir build

echo Assembling startup...
"%AS%" %ASFLAGS% -c startup\startup_gd32f303.s -o build\startup_gd32f303.o
if errorlevel 1 goto :fail

echo Compiling...
"%CXX%" %CXXFLAGS% -c src\main.cpp      -o build\main.o
if errorlevel 1 goto :fail
"%CXX%" %CXXFLAGS% -c src\spi2.cpp      -o build\spi2.o
if errorlevel 1 goto :fail
"%CXX%" %CXXFLAGS% -c src\icm42688.cpp  -o build\icm42688.o
if errorlevel 1 goto :fail
"%CXX%" %CXXFLAGS% -c src\i2c1.cpp      -o build\i2c1.o
if errorlevel 1 goto :fail
"%CXX%" %CXXFLAGS% -c src\bmp280.cpp    -o build\bmp280.o
if errorlevel 1 goto :fail
"%CXX%" %CXXFLAGS% -c src\ibus.cpp      -o build\ibus.o
if errorlevel 1 goto :fail
"%CXX%" %CXXFLAGS% -c src\servo_pwm.cpp -o build\servo_pwm.o
if errorlevel 1 goto :fail
"%CXX%" %CXXFLAGS% -c src\gps.cpp       -o build\gps.o
if errorlevel 1 goto :fail

echo Linking...
"%CXX%" %MCU_FLAGS% -Tlinker\gd32f303cc.ld -nostartfiles -Wl,--gc-sections -lm -Wl,-Map=build\pilot3_fw.map ^
    build\startup_gd32f303.o build\main.o build\spi2.o build\icm42688.o build\i2c1.o build\bmp280.o build\ibus.o build\servo_pwm.o build\gps.o -o build\pilot3_fw.elf
if errorlevel 1 goto :fail

"%OBJCOPY%" -O binary build\pilot3_fw.elf build\pilot3_fw.bin
"%SIZE%" build\pilot3_fw.elf

echo.
echo Build OK: build\pilot3_fw.elf
exit /b 0

:fail
echo.
echo Build FAILED.
exit /b 1
