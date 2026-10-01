# Pilot_3 firmware build. Portable reference build (needs `make` on PATH --
# e.g. via MSYS2 or WSL). On a plain Windows install without make, use
# build.bat instead; it runs the same compiler invocations directly.

TARGET    = pilot3_fw
BUILD_DIR = build

CXX     = arm-none-eabi-g++
AS      = arm-none-eabi-gcc
OBJCOPY = arm-none-eabi-objcopy
SIZE    = arm-none-eabi-size

MCU_FLAGS = -mcpu=cortex-m4 -mthumb -mfpu=fpv4-sp-d16 -mfloat-abi=hard

CXXFLAGS = $(MCU_FLAGS) -std=c++17 -Os -ffunction-sections -fdata-sections \
           -fno-exceptions -fno-rtti -Wall -Isrc
ASFLAGS  = $(MCU_FLAGS)
LDFLAGS  = $(MCU_FLAGS) -Tlinker/gd32f303cc.ld -nostartfiles \
           -Wl,--gc-sections -lm -Wl,-Map=$(BUILD_DIR)/$(TARGET).map

SRC_CPP = src/main.cpp src/spi2.cpp src/icm42688.cpp src/i2c1.cpp src/bmp280.cpp src/ibus.cpp src/servo_pwm.cpp src/gps.cpp
OBJS    = $(patsubst src/%.cpp,$(BUILD_DIR)/%.o,$(SRC_CPP)) \
          $(BUILD_DIR)/startup_gd32f303.o

all: $(BUILD_DIR)/$(TARGET).elf $(BUILD_DIR)/$(TARGET).bin
	$(SIZE) $(BUILD_DIR)/$(TARGET).elf

$(BUILD_DIR):
	mkdir -p $(BUILD_DIR)

$(BUILD_DIR)/%.o: src/%.cpp | $(BUILD_DIR)
	$(CXX) $(CXXFLAGS) -c $< -o $@

$(BUILD_DIR)/startup_gd32f303.o: startup/startup_gd32f303.s | $(BUILD_DIR)
	$(AS) $(ASFLAGS) -c $< -o $@

$(BUILD_DIR)/$(TARGET).elf: $(OBJS)
	$(CXX) $(LDFLAGS) $(OBJS) -o $@

$(BUILD_DIR)/$(TARGET).bin: $(BUILD_DIR)/$(TARGET).elf
	$(OBJCOPY) -O binary $< $@

clean:
	rm -rf $(BUILD_DIR)

.PHONY: all clean
