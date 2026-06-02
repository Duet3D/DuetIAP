# DuetIAP Duet3Mini_SD Configuration Makefile
# Duet 3 Mini 5+ - SD card IAP (SAME54P20A, Cortex-M4)

# Build directory and output
D3MINISD_BUILD_DIR := Duet3Mini_SD
D3MINISD_TARGET_NAME := Duet3_SDiap32_Mini5plus
D3MINISD_TARGET_ELF := $(D3MINISD_BUILD_DIR)/$(D3MINISD_TARGET_NAME).elf
D3MINISD_TARGET_BIN := $(D3MINISD_BUILD_DIR)/$(D3MINISD_TARGET_NAME).bin
D3MINISD_TARGET_MAP := $(D3MINISD_BUILD_DIR)/$(D3MINISD_TARGET_NAME).map

# Workspace root
WORKSPACE := ..

# Library dependencies
D3MINISD_COREN2G_LIB := $(WORKSPACE)/CoreN2G/SAME5x_SDHC/libCoreN2G.a
D3MINISD_RRFLIBS_LIB := $(WORKSPACE)/RRFLibraries/SAME51/libRRFLibraries.a

# Source directories
D3MINISD_SRC_DIR := src

# Find all source files (SD builds include Libraries/)
D3MINISD_CPP_SRCS := $(shell find $(D3MINISD_SRC_DIR) -name '*.cpp')

D3MINISD_C_SRCS := $(shell find $(D3MINISD_SRC_DIR) -name '*.c')

# Include paths - C compiler
D3MINISD_C_INCLUDES := \
	-I$(D3MINISD_SRC_DIR) \
	-I$(WORKSPACE)/CoreN2G/src \
	-I$(WORKSPACE)/CoreN2G/src/SAME5x_C21/SAME5x \
	-I$(WORKSPACE)/CoreN2G/src/SAME5x_C21/SAME5x/Config \
	-I$(WORKSPACE)/CoreN2G/src/atmel/SAME54_DFP/1.1.134/include \
	-I$(WORKSPACE)/CoreN2G/src/arm/CMSIS/5.4.0/CMSIS/Core/Include \
	-I$(WORKSPACE)/RRFLibraries/src

# Include paths - C++ compiler
D3MINISD_CXX_INCLUDES := \
	-I$(WORKSPACE)/CoreN2G/src \
	-I$(WORKSPACE)/CoreN2G/src/SAME5x_C21 \
	-I$(WORKSPACE)/CoreN2G/src/SAME5x_C21/SAME5x \
	-I$(WORKSPACE)/CoreN2G/src/SAME5x_C21/SAME5x/Config \
	-I$(WORKSPACE)/CoreN2G/src/SAME5x_C21/SAME5x/hal/include \
	-I$(WORKSPACE)/CoreN2G/src/SAME5x_C21/SAME5x/hal/utils/include \
	-I$(WORKSPACE)/CoreN2G/src/SAME5x_C21/SAME5x/hri \
	-I$(WORKSPACE)/CoreN2G/src/arm/CMSIS/5.4.0/CMSIS/Core/Include \
	-I$(WORKSPACE)/CoreN2G/src/atmel/SAME54_DFP/1.1.134/include \
	-I$(WORKSPACE)/RRFLibraries/src \
	-I$(D3MINISD_SRC_DIR)/Libraries/Fatfs

# Preprocessor defines - C
D3MINISD_C_DEFINES := \
	-D__SAME54P20A__ \
	-DDUET3_MINI \
	-Dnoexcept=

# Preprocessor defines - C++
D3MINISD_CXX_DEFINES := \
	-D__SAME54P20A__ \
	-DDUET3_MINI \
	-DIAP_IN_RAM

# Compiler flags - C
D3MINISD_CFLAGS := -c -std=gnu99 \
	-Os \
	-mcpu=cortex-m4 \
	-mthumb \
	-mfpu=fpv4-sp-d16 \
	-mfloat-abi=hard \
	-mfp16-format=ieee \
	-ffunction-sections \
	-fdata-sections \
	-nostdlib \
	-Wundef \
	-Wdouble-promotion \
	-fsingle-precision-constant \
	$(D3MINISD_C_INCLUDES) \
	$(D3MINISD_C_DEFINES) \
	$(DEBUG_FLAGS)

# Compiler flags - C++
D3MINISD_CXXFLAGS := -c -std=c++20 \
	-Os \
	-mcpu=cortex-m4 \
	-mthumb \
	-mfpu=fpv4-sp-d16 \
	-mfloat-abi=hard \
	-mfp16-format=ieee \
	-ffunction-sections \
	-fdata-sections \
	-fno-threadsafe-statics \
	-fno-rtti \
	-fno-exceptions \
	-nostdlib \
	-Wundef \
	-Wdouble-promotion \
	-fsingle-precision-constant \
	$(D3MINISD_CXX_INCLUDES) \
	$(D3MINISD_CXX_DEFINES) \
	$(DEBUG_FLAGS)

# Linker flags - before -o
D3MINISD_LDFLAGS1 := --specs=nano.specs \
	-Os \
	-Wl,--gc-sections \
	-Wl,--fatal-warnings \
	-mcpu=cortex-m4 \
	-mfpu=fpv4-sp-d16 \
	-mfloat-abi=hard \
	-T$(D3MINISD_SRC_DIR)/LinkerScripts/same54p20a_iap_ram.ld \
	-Wl,-Map,$(D3MINISD_TARGET_MAP) \
	-mthumb

# Linker flags - after -o
D3MINISD_LDFLAGS2 := \
	-Wl,--cref \
	-Wl,--check-sections \
	-Wl,--gc-sections \
	-Wl,--entry=Reset_Handler \
	-Wl,--unresolved-symbols=report-all \
	-Wl,--warn-common \
	-Wl,--warn-section-align \
	-Wl,--warn-unresolved-symbols

# Library search paths and libraries
D3MINISD_LDLIBS := \
	-L$(WORKSPACE)/CoreN2G/SAME5x_SDHC \
	-L$(WORKSPACE)/RRFLibraries/SAME51 \
	-lCoreN2G \
	-lRRFLibraries \
	-lsupc++

D3MINISD_LDLIBS_POST := -Wl,--end-group -lm

# Object files
D3MINISD_CPP_OBJS := $(D3MINISD_CPP_SRCS:%.cpp=$(D3MINISD_BUILD_DIR)/%.o)
D3MINISD_C_OBJS := $(D3MINISD_C_SRCS:%.c=$(D3MINISD_BUILD_DIR)/%.o)
D3MINISD_OBJS := $(D3MINISD_CPP_OBJS) $(D3MINISD_C_OBJS)

# Dependency files
D3MINISD_DEPS := $(D3MINISD_OBJS:.o=.d)

# Target rule
.PHONY: Duet3Mini_SD
Duet3Mini_SD: $(D3MINISD_TARGET_BIN)
	$(Q)echo "========================================"
	$(Q)echo "Duet3Mini_SD IAP build complete!"
	$(Q)echo "Output: $(D3MINISD_TARGET_BIN)"
	$(Q)echo "========================================"
	$(Q)$(SIZE) $(D3MINISD_TARGET_ELF)

# Link ELF file
$(D3MINISD_TARGET_ELF): $(D3MINISD_OBJS) $(D3MINISD_COREN2G_LIB) $(D3MINISD_RRFLIBS_LIB)
	$(Q)echo "  LD      $@"
	$(Q)mkdir -p $(@D)
	$(Q)$(LD) $(D3MINISD_LDFLAGS1) -o $@ $(D3MINISD_LDFLAGS2) -Wl,--start-group $(D3MINISD_OBJS) $(D3MINISD_LDLIBS) $(D3MINISD_LDLIBS_POST)

# Generate binary file
$(D3MINISD_TARGET_BIN): $(D3MINISD_TARGET_ELF)
	$(Q)echo "  OBJCOPY $@"
	$(Q)$(OBJCOPY) -O binary $< $@

# Compile C++ files
$(D3MINISD_BUILD_DIR)/%.o: %.cpp
	$(Q)echo "  CXX     $<"
	$(Q)mkdir -p $(@D)
	$(Q)$(CXX) $(D3MINISD_CXXFLAGS) -MMD -MP -o $@ $<

# Compile C files
$(D3MINISD_BUILD_DIR)/%.o: %.c
	$(Q)echo "  CC      $<"
	$(Q)mkdir -p $(@D)
	$(Q)$(CC) $(D3MINISD_CFLAGS) -MMD -MP -o $@ $<

# Include dependencies
-include $(D3MINISD_DEPS)

# Clean target
.PHONY: clean-Duet3Mini_SD
clean-Duet3Mini_SD:
	$(Q)echo "  RM      $(D3MINISD_BUILD_DIR)"
	$(Q)rm -rf $(D3MINISD_BUILD_DIR)
