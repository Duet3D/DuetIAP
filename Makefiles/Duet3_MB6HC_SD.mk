# DuetIAP Duet3_MB6HC_SD Configuration Makefile
# Duet 3 MB6HC - SD card IAP (SAME70Q20B, Cortex-M7)

# Build directory and output
D3MB6HCSD_BUILD_DIR := Duet3_MB6HC_SD
D3MB6HCSD_TARGET_NAME := Duet3_SDiap32_MB6HC
D3MB6HCSD_TARGET_ELF := $(D3MB6HCSD_BUILD_DIR)/$(D3MB6HCSD_TARGET_NAME).elf
D3MB6HCSD_TARGET_BIN := $(D3MB6HCSD_BUILD_DIR)/$(D3MB6HCSD_TARGET_NAME).bin
D3MB6HCSD_TARGET_MAP := $(D3MB6HCSD_BUILD_DIR)/$(D3MB6HCSD_TARGET_NAME).map

# Workspace root
WORKSPACE := ..

# Library dependencies
D3MB6HCSD_COREN2G_LIB := $(WORKSPACE)/CoreN2G/SAME70_SDHC/libCoreN2G.a
D3MB6HCSD_RRFLIBS_LIB := $(WORKSPACE)/RRFLibraries/SAME70/libRRFLibraries.a

# Source directories
D3MB6HCSD_SRC_DIR := src

# Find all source files (SD builds include Libraries/)
D3MB6HCSD_CPP_SRCS := $(shell find $(D3MB6HCSD_SRC_DIR) -name '*.cpp')

D3MB6HCSD_C_SRCS := $(shell find $(D3MB6HCSD_SRC_DIR) -name '*.c')

# Include paths - C compiler
D3MB6HCSD_C_INCLUDES := \
	-I$(D3MB6HCSD_SRC_DIR) \
	-I$(WORKSPACE)/CoreN2G \
	-I$(WORKSPACE)/CoreN2G/src \
	-I$(WORKSPACE)/CoreN2G/src/SAM4S_4E_E70 \
	-I$(WORKSPACE)/CoreN2G/src/SAM4S_4E_E70/SAME70 \
	-I$(WORKSPACE)/CoreN2G/src/SAM4S_4E_E70/asf \
	-I$(WORKSPACE)/CoreN2G/src/SAM4S_4E_E70/asf/common/utils \
	-I$(WORKSPACE)/CoreN2G/src/SAM4S_4E_E70/asf/sam/drivers \
	-I$(WORKSPACE)/CoreN2G/src/SAM4S_4E_E70/asf/sam/utils \
	-I$(WORKSPACE)/CoreN2G/src/SAM4S_4E_E70/asf/sam/utils/cmsis/same70/include \
	-I$(WORKSPACE)/CoreN2G/src/SAM4S_4E_E70/asf/sam/utils/header_files \
	-I$(WORKSPACE)/CoreN2G/src/SAM4S_4E_E70/asf/sam/utils/preprocessor \
	-I$(WORKSPACE)/CoreN2G/src/arm/CMSIS/5.4.0/CMSIS/Core/Include \
	-I$(WORKSPACE)/RRFLibraries/src

# Include paths - C++ compiler
D3MB6HCSD_CXX_INCLUDES := \
	-I$(WORKSPACE)/CoreN2G/src \
	-I$(WORKSPACE)/CoreN2G/src/SAM4S_4E_E70 \
	-I$(WORKSPACE)/CoreN2G/src/SAM4S_4E_E70/asf \
	-I$(WORKSPACE)/CoreN2G/src/SAM4S_4E_E70/asf/common/utils \
	-I$(WORKSPACE)/CoreN2G/src/SAM4S_4E_E70/asf/sam/drivers \
	-I$(WORKSPACE)/CoreN2G/src/SAM4S_4E_E70/asf/sam/utils \
	-I$(WORKSPACE)/CoreN2G/src/SAM4S_4E_E70/asf/sam/utils/cmsis/same70/include \
	-I$(WORKSPACE)/CoreN2G/src/SAM4S_4E_E70/asf/sam/utils/header_files \
	-I$(WORKSPACE)/CoreN2G/src/SAM4S_4E_E70/asf/sam/utils/preprocessor \
	-I$(WORKSPACE)/CoreN2G/src/SAM4S_4E_E70/SAME70 \
	-I$(WORKSPACE)/CoreN2G/src/arm/CMSIS/5.4.0/CMSIS/Core/Include \
	-I$(D3MB6HCSD_SRC_DIR)/Libraries/Fatfs \
	-I$(WORKSPACE)/RRFLibraries/src

# Preprocessor defines - C
D3MB6HCSD_C_DEFINES := \
	-D__SAME70Q20B__ \
	-Dnoexcept=

# Preprocessor defines - C++
D3MB6HCSD_CXX_DEFINES := \
	-D__SAME70Q20B__ \
	-DDUET3_MB6HC \
	-DIAP_IN_RAM

# Compiler flags - C
D3MB6HCSD_CFLAGS := -c -std=gnu99 \
	-Os \
	-mcpu=cortex-m7 \
	-mthumb \
	-mfpu=fpv5-d16 \
	-mfloat-abi=hard \
	-mfp16-format=ieee \
	-mno-unaligned-access \
	-ffunction-sections \
	-fdata-sections \
	-nostdlib \
	-Wundef \
	-Wdouble-promotion \
	-fsingle-precision-constant \
	$(D3MB6HCSD_C_INCLUDES) \
	$(D3MB6HCSD_C_DEFINES) \
	$(DEBUG_FLAGS)

# Compiler flags - C++
D3MB6HCSD_CXXFLAGS := -c -std=gnu++17 \
	-Os \
	-mcpu=cortex-m7 \
	-mthumb \
	-mfpu=fpv5-d16 \
	-mfloat-abi=hard \
	-mfp16-format=ieee \
	-mno-unaligned-access \
	-ffunction-sections \
	-fdata-sections \
	-fno-threadsafe-statics \
	-fno-rtti \
	-fno-exceptions \
	-nostdlib \
	-Wundef \
	-Wdouble-promotion \
	-fsingle-precision-constant \
	$(D3MB6HCSD_CXX_INCLUDES) \
	$(D3MB6HCSD_CXX_DEFINES) \
	$(DEBUG_FLAGS)

# Linker flags - before -o
D3MB6HCSD_LDFLAGS1 := --specs=nano.specs \
	-Os \
	-Wl,--gc-sections \
	-Wl,--fatal-warnings \
	-mcpu=cortex-m7 \
	-mfpu=fpv5-d16 \
	-mfloat-abi=hard \
	-T$(D3MB6HCSD_SRC_DIR)/LinkerScripts/same70_iap_ram.ld \
	-Wl,-Map,$(D3MB6HCSD_TARGET_MAP) \
	-mthumb

# Linker flags - after -o
D3MB6HCSD_LDFLAGS2 := \
	-Wl,--cref \
	-Wl,--check-sections \
	-Wl,--gc-sections \
	-Wl,--entry=Reset_Handler \
	-Wl,--unresolved-symbols=report-all \
	-Wl,--warn-common \
	-Wl,--warn-section-align \
	-Wl,--warn-unresolved-symbols

# Library search paths and libraries
D3MB6HCSD_LDLIBS := \
	-L$(WORKSPACE)/CoreN2G/SAME70_SDHC \
	-L$(WORKSPACE)/RRFLibraries/SAME70 \
	-lCoreN2G \
	-lRRFLibraries \
	-lsupc++

D3MB6HCSD_LDLIBS_POST := -Wl,--end-group -lm

# Object files
D3MB6HCSD_CPP_OBJS := $(D3MB6HCSD_CPP_SRCS:%.cpp=$(D3MB6HCSD_BUILD_DIR)/%.o)
D3MB6HCSD_C_OBJS := $(D3MB6HCSD_C_SRCS:%.c=$(D3MB6HCSD_BUILD_DIR)/%.o)
D3MB6HCSD_OBJS := $(D3MB6HCSD_CPP_OBJS) $(D3MB6HCSD_C_OBJS)

# Dependency files
D3MB6HCSD_DEPS := $(D3MB6HCSD_OBJS:.o=.d)

# Target rule
.PHONY: Duet3_MB6HC_SD
Duet3_MB6HC_SD: $(D3MB6HCSD_TARGET_BIN)
	$(Q)echo "========================================"
	$(Q)echo "Duet3_MB6HC_SD IAP build complete!"
	$(Q)echo "Output: $(D3MB6HCSD_TARGET_BIN)"
	$(Q)echo "========================================"
	$(Q)$(SIZE) $(D3MB6HCSD_TARGET_ELF)

# Link ELF file
$(D3MB6HCSD_TARGET_ELF): $(D3MB6HCSD_OBJS) $(D3MB6HCSD_COREN2G_LIB) $(D3MB6HCSD_RRFLIBS_LIB)
	$(Q)echo "  LD      $@"
	$(Q)mkdir -p $(@D)
	$(Q)$(LD) $(D3MB6HCSD_LDFLAGS1) -o $@ $(D3MB6HCSD_LDFLAGS2) -Wl,--start-group $(D3MB6HCSD_OBJS) $(D3MB6HCSD_LDLIBS) $(D3MB6HCSD_LDLIBS_POST)

# Generate binary file
$(D3MB6HCSD_TARGET_BIN): $(D3MB6HCSD_TARGET_ELF)
	$(Q)echo "  OBJCOPY $@"
	$(Q)$(OBJCOPY) -O binary $< $@

# Compile C++ files
$(D3MB6HCSD_BUILD_DIR)/%.o: %.cpp
	$(Q)echo "  CXX     $<"
	$(Q)mkdir -p $(@D)
	$(Q)$(CXX) $(D3MB6HCSD_CXXFLAGS) -MMD -MP -o $@ $<

# Compile C files
$(D3MB6HCSD_BUILD_DIR)/%.o: %.c
	$(Q)echo "  CC      $<"
	$(Q)mkdir -p $(@D)
	$(Q)$(CC) $(D3MB6HCSD_CFLAGS) -MMD -MP -o $@ $<

# Include dependencies
-include $(D3MB6HCSD_DEPS)

# Clean target
.PHONY: clean-Duet3_MB6HC_SD
clean-Duet3_MB6HC_SD:
	$(Q)echo "  RM      $(D3MB6HCSD_BUILD_DIR)"
	$(Q)rm -rf $(D3MB6HCSD_BUILD_DIR)
