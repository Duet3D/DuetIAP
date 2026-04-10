# DuetIAP FMDC_SD Configuration Makefile
# FMDC - SD card IAP (SAME51N19A, Cortex-M4)

# Build directory and output
FMDCSD_BUILD_DIR := FMDC_SD
FMDCSD_TARGET_NAME := Duet3_SDiap32_FMDC
FMDCSD_TARGET_ELF := $(FMDCSD_BUILD_DIR)/$(FMDCSD_TARGET_NAME).elf
FMDCSD_TARGET_BIN := $(FMDCSD_BUILD_DIR)/$(FMDCSD_TARGET_NAME).bin
FMDCSD_TARGET_MAP := $(FMDCSD_BUILD_DIR)/$(FMDCSD_TARGET_NAME).map

# Workspace root
WORKSPACE := ..

# Library dependencies
FMDCSD_COREN2G_LIB := $(WORKSPACE)/CoreN2G/SAME5x_SDHC/libCoreN2G.a
FMDCSD_RRFLIBS_LIB := $(WORKSPACE)/RRFLibraries/SAME51/libRRFLibraries.a

# Source directories
FMDCSD_SRC_DIR := src

# Find all source files (SD builds include Libraries/)
FMDCSD_CPP_SRCS := $(shell find $(FMDCSD_SRC_DIR) -name '*.cpp')

FMDCSD_C_SRCS := $(shell find $(FMDCSD_SRC_DIR) -name '*.c')

# Include paths - C compiler
FMDCSD_C_INCLUDES := \
	-I$(FMDCSD_SRC_DIR) \
	-I$(WORKSPACE)/CoreN2G/src \
	-I$(WORKSPACE)/CoreN2G/src/SAME5x_C21/SAME5x \
	-I$(WORKSPACE)/CoreN2G/src/SAME5x_C21/SAME5x/Config \
	-I$(WORKSPACE)/CoreN2G/src/atmel/SAME51_DFP/1.1.139/include \
	-I$(WORKSPACE)/CoreN2G/src/arm/CMSIS/5.4.0/CMSIS/Core/Include \
	-I$(WORKSPACE)/RRFLibraries/src

# Include paths - C++ compiler
FMDCSD_CXX_INCLUDES := \
	-I$(WORKSPACE)/CoreN2G/src \
	-I$(WORKSPACE)/CoreN2G/src/SAME5x_C21 \
	-I$(WORKSPACE)/CoreN2G/src/SAME5x_C21/SAME5x \
	-I$(WORKSPACE)/CoreN2G/src/SAME5x_C21/SAME5x/Config \
	-I$(WORKSPACE)/CoreN2G/src/SAME5x_C21/SAME5x/hal/include \
	-I$(WORKSPACE)/CoreN2G/src/SAME5x_C21/SAME5x/hal/utils/include \
	-I$(WORKSPACE)/CoreN2G/src/SAME5x_C21/SAME5x/hri \
	-I$(WORKSPACE)/CoreN2G/src/arm/CMSIS/5.4.0/CMSIS/Core/Include \
	-I$(WORKSPACE)/CoreN2G/src/atmel/SAME51_DFP/1.1.139/include \
	-I$(WORKSPACE)/RRFLibraries/src \
	-I$(FMDCSD_SRC_DIR)/Libraries/Fatfs

# Preprocessor defines - C
FMDCSD_C_DEFINES := \
	-D__SAME51N19A__ \
	-DFMDC \
	-Dnoexcept=

# Preprocessor defines - C++
FMDCSD_CXX_DEFINES := \
	-D__SAME51N19A__ \
	-DFMDC \
	-DIAP_IN_RAM

# Compiler flags - C
FMDCSD_CFLAGS := -c -std=gnu99 \
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
	$(FMDCSD_C_INCLUDES) \
	$(FMDCSD_C_DEFINES) \
	$(DEBUG_FLAGS)

# Compiler flags - C++
FMDCSD_CXXFLAGS := -c -std=gnu++17 \
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
	$(FMDCSD_CXX_INCLUDES) \
	$(FMDCSD_CXX_DEFINES) \
	$(DEBUG_FLAGS)

# Linker flags - before -o
FMDCSD_LDFLAGS1 := --specs=nano.specs \
	-Os \
	-Wl,--gc-sections \
	-Wl,--fatal-warnings \
	-mcpu=cortex-m4 \
	-mfpu=fpv4-sp-d16 \
	-mfloat-abi=hard \
	-T$(FMDCSD_SRC_DIR)/LinkerScripts/same51n19a_iap_ram.ld \
	-Wl,-Map,$(FMDCSD_TARGET_MAP) \
	-mthumb

# Linker flags - after -o
FMDCSD_LDFLAGS2 := \
	-Wl,--cref \
	-Wl,--check-sections \
	-Wl,--gc-sections \
	-Wl,--entry=Reset_Handler \
	-Wl,--unresolved-symbols=report-all \
	-Wl,--warn-common \
	-Wl,--warn-section-align \
	-Wl,--warn-unresolved-symbols

# Library search paths and libraries
FMDCSD_LDLIBS := \
	-L$(WORKSPACE)/CoreN2G/SAME5x_SDHC \
	-L$(WORKSPACE)/RRFLibraries/SAME51 \
	-lCoreN2G \
	-lRRFLibraries \
	-lsupc++

FMDCSD_LDLIBS_POST := -Wl,--end-group -lm

# Object files
FMDCSD_CPP_OBJS := $(FMDCSD_CPP_SRCS:%.cpp=$(FMDCSD_BUILD_DIR)/%.o)
FMDCSD_C_OBJS := $(FMDCSD_C_SRCS:%.c=$(FMDCSD_BUILD_DIR)/%.o)
FMDCSD_OBJS := $(FMDCSD_CPP_OBJS) $(FMDCSD_C_OBJS)

# Dependency files
FMDCSD_DEPS := $(FMDCSD_OBJS:.o=.d)

# Target rule
.PHONY: FMDC_SD
FMDC_SD: $(FMDCSD_TARGET_BIN)
	$(Q)echo "========================================"
	$(Q)echo "FMDC_SD IAP build complete!"
	$(Q)echo "Output: $(FMDCSD_TARGET_BIN)"
	$(Q)echo "========================================"
	$(Q)$(SIZE) $(FMDCSD_TARGET_ELF)

# Link ELF file
$(FMDCSD_TARGET_ELF): $(FMDCSD_OBJS) $(FMDCSD_COREN2G_LIB) $(FMDCSD_RRFLIBS_LIB)
	$(Q)echo "  LD      $@"
	$(Q)mkdir -p $(@D)
	$(Q)$(LD) $(FMDCSD_LDFLAGS1) -o $@ $(FMDCSD_LDFLAGS2) -Wl,--start-group $(FMDCSD_OBJS) $(FMDCSD_LDLIBS) $(FMDCSD_LDLIBS_POST)

# Generate binary file
$(FMDCSD_TARGET_BIN): $(FMDCSD_TARGET_ELF)
	$(Q)echo "  OBJCOPY $@"
	$(Q)$(OBJCOPY) -O binary $< $@

# Compile C++ files
$(FMDCSD_BUILD_DIR)/%.o: %.cpp
	$(Q)echo "  CXX     $<"
	$(Q)mkdir -p $(@D)
	$(Q)$(CXX) $(FMDCSD_CXXFLAGS) -MMD -MP -o $@ $<

# Compile C files
$(FMDCSD_BUILD_DIR)/%.o: %.c
	$(Q)echo "  CC      $<"
	$(Q)mkdir -p $(@D)
	$(Q)$(CC) $(FMDCSD_CFLAGS) -MMD -MP -o $@ $<

# Include dependencies
-include $(FMDCSD_DEPS)

# Clean target
.PHONY: clean-FMDC_SD
clean-FMDC_SD:
	$(Q)echo "  RM      $(FMDCSD_BUILD_DIR)"
	$(Q)rm -rf $(FMDCSD_BUILD_DIR)
