# Makefile for Duet IAP flasher binary
# written by Christian Hammacher
#
# Licensed under the terms of the GNU Public License v2
# see http://www.gnu.org/licenses/gpl-2.0.html
#

# Referenced component versions
ARDUINO_VERSION := 1.6.5
GCC_VERSION := 4.8.3-2014q1
BOSSAC_VERSION := 1.3a-arduino

DUET_BOARD_VERSION := 1.0.8

# Workspace paths
LIBRARY_PATH := $(PWD)/Libraries
BUILD_PATH := $(PWD)/obj
OUTPUT_DIR := $(PWD)/Release

# Compiler options
OPTIMIZATION := -O2


# ================================ Prerequisites ====================================

# Determine Arduino path
UNAME := $(shell uname -s)
ifeq ($(UNAME),Linux)
  ARDUINO_PATH := $(HOME)/.arduino15
endif
ifeq ($(UNAME),Darwin)
  ARDUINO_PATH := $(HOME)/Library/Arduino15
endif
ifeq (,$(wildcard $(ARDUINO_PATH)/.))
  $(error Arduino directory not found! Are you using Arduino $(ARDUINO_VERSION)?)
endif

# Detect Duet board path
DUET_BOARD_PATH := $(ARDUINO_PATH)/packages/RepRap/hardware/sam/$(DUET_BOARD_VERSION)
ifeq (,$(wildcard $(DUET_BOARD_PATH)/.))
  $(error Duet board not found! Install it first via Arduino''s Boards Manager.)
endif

# Detect GCC path
GCC_PATH := $(ARDUINO_PATH)/packages/arduino/tools/arm-none-eabi-gcc/$(GCC_VERSION)
ifeq (,$(wildcard $(GCC_PATH)/.))
  $(error GCC toolchain not found! Check your installation.)
endif


# ================================ GCC Options ======================================

CROSS_COMPILE := arm-none-eabi-
CC := $(GCC_PATH)/bin/$(CROSS_COMPILE)gcc
CXX := $(GCC_PATH)/bin/$(CROSS_COMPILE)g++
LD := $(GCC_PATH)/bin/$(CROSS_COMPILE)gcc
OBJCOPY := $(GCC_PATH)/bin/$(CROSS_COMPILE)objcopy

INCLUDES := $(DUET_BOARD_PATH)/libraries/Flash $(DUET_BOARD_PATH)/libraries/SD_HSMCI $(DUET_BOARD_PATH)/libraries/SD_HSMCI/utility
INCLUDES += $(DUET_BOARD_PATH)/cores/arduino $(DUET_BOARD_PATH)/variants/duet
INCLUDES += $(DUET_BOARD_PATH)/system/libsam $(DUET_BOARD_PATH)/system/libsam/include $(DUET_BOARD_PATH)/system/CMSIS/CMSIS/Include $(DUET_BOARD_PATH)/system/CMSIS/Device/ATMEL

CFLAGS += -c -g $(OPTIMIZATION) -std=gnu11 -Wall -Wno-unused-variable -ffunction-sections -fdata-sections -nostdlib --param max-inline-insns-single=500 -Dprintf=iprintf -MMD -MP
CPPFLAGS += -c -g $(OPTIMIZATION) -Wall -Wno-unused-variable -Wno-format -std=gnu++11 -ffunction-sections -fdata-sections -nostdlib -fno-threadsafe-statics --param max-inline-insns-single=500 -fno-rtti -fno-exceptions -Dprintf=iprintf -MMD -MP

DEVICE_FLAGS = -mcpu=cortex-m3 -DF_CPU=84000000L -DARDUINO=$(subst .,,$(ARDUINO_VERSION)) -DARDUINO_SAM_DUE -DARDUINO_ARCH_SAM -D__SAM3X8E__ -mthumb -DUSB_VID=0x2341 -DUSB_PID=0x003e -DUSBCON -DUSB_MANUFACTURER=\"Unknown\" -DUSB_PRODUCT=\"Duet\"

CFLAGS += $(foreach dir,$(INCLUDES),-I$(dir))
CPPFLAGS += $(foreach dir,$(INCLUDES),-I$(dir))

LDFLAGS += $(OPTIMIZATION) -Wl,--gc-sections -mcpu=cortex-m3 "-T$(DUET_BOARD_PATH)/variants/duet/linker_scripts/gcc/flash_iap.ld" "-Wl,-Map,$(OUTPUT_DIR)/iap.map" "-L$(BUILD_PATH)" -mthumb -Wl,--cref -Wl,--check-sections -Wl,--gc-sections -Wl,--entry=Reset_Handler -Wl,--unresolved-symbols=report-all -Wl,--warn-common -Wl,--warn-section-align -Wl,--warn-unresolved-symbols -Wl,--start-group $(BUILD_PATH)/*.o "$(DUET_BOARD_PATH)/variants/duet/libsam_sam3x8e_gcc_rel.a" -lm -lgcc -Wl,--end-group

# Unfortunately make doesn't support directory wildcards in targets, so instead we must explicitly specify the source paths by using VPATH
VPATH := $(DUET_BOARD_PATH)/cores/arduino $(DUET_BOARD_PATH)/cores/arduino/USB $(DUET_BOARD_PATH)/variants/duet
VPATH += $(DUET_BOARD_PATH)/libraries/Flash $(DUET_BOARD_PATH)/libraries/SD_HSMCI $(DUET_BOARD_PATH)/libraries/SD_HSMCI/utility

C_SOURCES += $(foreach dir,$(VPATH),$(wildcard $(dir)/*.c)) $(wildcard $(PWD)/*.c)
CPP_SOURCES := $(foreach dir,$(VPATH),$(wildcard $(dir)/*.cpp)) $(wildcard $(PWD)/*.cpp)

C_OBJS := $(foreach src,$(C_SOURCES),$(BUILD_PATH)/$(notdir $(src:.c=.c.o)))
CPP_OBJS := $(foreach src,$(CPP_SOURCES),$(BUILD_PATH)/$(notdir $(src:.cpp=.cpp.o)))

DEPS := $(C_OBJS:%.o=%.d) $(CPP_OBJS:%.o=%.d)


# ================================= Target all ======================================
.PHONY += all
all: $(OUTPUT_DIR)/iap.bin
$(OUTPUT_DIR)/iap.bin: $(OUTPUT_DIR)/iap.elf
	@echo "  BIN     iap.bin"
	@$(OBJCOPY) -O binary $(OUTPUT_DIR)/iap.elf $(OUTPUT_DIR)/iap.bin

$(OUTPUT_DIR)/iap.elf: $(BUILD_PATH) $(OUTPUT_DIR) $(C_OBJS) $(CPP_OBJS)
	@echo "  LD      iap.elf"
	@$(LD) $(LDFLAGS) -o $(OUTPUT_DIR)/iap.elf
-include $(DEPS)

$(BUILD_PATH)/%.c.o: %.c
	@echo "  CC      $(subst $(DUET_BOARD_PATH)/,,$(subst $(PWD)/,,$<))"
	@$(CC) $(CFLAGS) $(DEVICE_FLAGS) $< -o $@

$(BUILD_PATH)/%.cpp.o: %.cpp
	@echo "  CC      $(subst $(DUET_BOARD_PATH)/,,$(subst $(PWD)/,,$<))"
	@$(CXX) $(CPPFLAGS) $(DEVICE_FLAGS) $< -o $@

$(BUILD_PATH):
	@mkdir -p $(BUILD_PATH)

$(OUTPUT_DIR):
	@mkdir -p $(OUTPUT_DIR)


# ================================= Target clean ====================================
.PHONY += clean
clean:
	@rm -rf $(BUILD_PATH) $(OUTPUT_DIR)
	@rm -f $(PWD)/*.d
	$(info Build directories removed.)

