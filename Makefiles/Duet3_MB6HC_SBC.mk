# DuetIAP Duet3_MB6HC_SBC Configuration Makefile
# Duet 3 MB6HC - SBC/SPI IAP (SAME70Q20B, Cortex-M7)

# Build directory and output
D3MB6HCSBC_BUILD_DIR := Duet3_MB6HC_SBC
D3MB6HCSBC_TARGET_NAME := Duet3_SBCiap32_MB6HC
D3MB6HCSBC_TARGET_ELF := $(D3MB6HCSBC_BUILD_DIR)/$(D3MB6HCSBC_TARGET_NAME).elf
D3MB6HCSBC_TARGET_BIN := $(D3MB6HCSBC_BUILD_DIR)/$(D3MB6HCSBC_TARGET_NAME).bin
D3MB6HCSBC_TARGET_MAP := $(D3MB6HCSBC_BUILD_DIR)/$(D3MB6HCSBC_TARGET_NAME).map

# Workspace root
WORKSPACE := ..

# Library dependencies
D3MB6HCSBC_COREN2G_LIB := $(WORKSPACE)/CoreN2G/SAME70_SDHC_USB/libCoreN2G.a
D3MB6HCSBC_RRFLIBS_LIB := $(WORKSPACE)/RRFLibraries/SAME70/libRRFLibraries.a

# Source directories
D3MB6HCSBC_SRC_DIR := src

# Find all source files (SBC builds exclude Libraries/ and the bare-metal USB driver)
D3MB6HCSBC_CPP_SRCS := $(shell find $(D3MB6HCSBC_SRC_DIR) -name '*.cpp' \
	! -path '*/Libraries/*')

D3MB6HCSBC_C_SRCS := $(shell find $(D3MB6HCSBC_SRC_DIR) -name '*.c' \
	! -path '*/Libraries/*')

# Include paths - C compiler
D3MB6HCSBC_C_INCLUDES := \
	-I$(WORKSPACE)/CoreN2G/src/SAM4S_4E_E70/asf \
	-I$(WORKSPACE)/CoreN2G/src/SAM4S_4E_E70/asf/common/utils \
	-I$(WORKSPACE)/CoreN2G/src/SAM4S_4E_E70/asf/sam/drivers \
	-I$(WORKSPACE)/CoreN2G/src/SAM4S_4E_E70/asf/sam/utils \
	-I$(WORKSPACE)/CoreN2G/src/SAM4S_4E_E70/asf/sam/utils/cmsis/same70/include \
	-I$(WORKSPACE)/CoreN2G/src/SAM4S_4E_E70/asf/sam/utils/header_files \
	-I$(WORKSPACE)/CoreN2G/src/SAM4S_4E_E70/asf/sam/utils/preprocessor \
	-I$(WORKSPACE)/CoreN2G/src/arm/CMSIS/5.4.0/CMSIS/Core/Include

# Include paths - C++ compiler
D3MB6HCSBC_CXX_INCLUDES := \
	-I$(WORKSPACE)/CoreN2G/src \
	-I$(WORKSPACE)/CoreN2G/src/SAM4S_4E_E70 \
	-I$(WORKSPACE)/CoreN2G/src/SAM4S_4E_E70/asf/common/utils \
	-I$(WORKSPACE)/CoreN2G/src/SAM4S_4E_E70/asf/sam/utils \
	-I$(WORKSPACE)/CoreN2G/src/SAM4S_4E_E70/asf/sam/utils/cmsis/same70/include \
	-I$(WORKSPACE)/CoreN2G/src/SAM4S_4E_E70/asf/sam/utils/header_files \
	-I$(WORKSPACE)/CoreN2G/src/SAM4S_4E_E70/asf/sam/utils/preprocessor \
	-I$(WORKSPACE)/CoreN2G/src/SAM4S_4E_E70/SAME70 \
	-I$(WORKSPACE)/CoreN2G/src/arm/CMSIS/5.4.0/CMSIS/Core/Include \
	-I$(D3MB6HCSBC_SRC_DIR)/Libraries/Fatfs \
	-I$(WORKSPACE)/RRFLibraries/src

# Preprocessor defines - C
D3MB6HCSBC_C_DEFINES := \
	-D__SAME70Q20B__ \
	-DSUPPORT_USB=1 \
	'-DUSB_DEVICE_PRODUCT_NAME="IAP"' \
	-Dnoexcept=

# Preprocessor defines - C++
D3MB6HCSBC_CXX_DEFINES := \
	-D__SAME70Q20B__ \
	-DDUET3_MB6HC \
	-DIAP_IN_RAM \
	-DIAP_SBC_SPI \
	-DSUPPORT_USB=1 \
	-DIAP_SBC_USB

# Compiler flags - C
D3MB6HCSBC_CFLAGS := -c -std=gnu99 \
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
	$(D3MB6HCSBC_C_INCLUDES) \
	$(D3MB6HCSBC_C_DEFINES) \
	$(DEBUG_FLAGS)

# Compiler flags - C++
D3MB6HCSBC_CXXFLAGS := -c -std=gnu++17 \
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
	$(D3MB6HCSBC_CXX_INCLUDES) \
	$(D3MB6HCSBC_CXX_DEFINES) \
	$(DEBUG_FLAGS)

# Linker flags - before -o
D3MB6HCSBC_LDFLAGS1 := --specs=nano.specs \
	-Os \
	-Wl,--gc-sections \
	-Wl,--fatal-warnings \
	-mcpu=cortex-m7 \
	-mfpu=fpv5-d16 \
	-mfloat-abi=hard \
	-T$(D3MB6HCSBC_SRC_DIR)/LinkerScripts/same70_iap_ram.ld \
	-Wl,-Map,$(D3MB6HCSBC_TARGET_MAP) \
	-mthumb

# Linker flags - after -o
D3MB6HCSBC_LDFLAGS2 := \
	-Wl,--cref \
	-Wl,--check-sections \
	-Wl,--gc-sections \
	-Wl,--entry=Reset_Handler \
	-Wl,-u,_estack \
	-Wl,--unresolved-symbols=report-all \
	-Wl,--warn-common \
	-Wl,--warn-section-align \
	-Wl,--warn-unresolved-symbols

# Library search paths and libraries
D3MB6HCSBC_LDLIBS := \
	-L$(WORKSPACE)/CoreN2G/SAME70_SDHC_USB \
	-L$(WORKSPACE)/RRFLibraries/SAME70 \
	-lCoreN2G \
	-lRRFLibraries \
	-lsupc++

D3MB6HCSBC_LDLIBS_POST := -Wl,--end-group -lm

# Object files
D3MB6HCSBC_CPP_OBJS := $(D3MB6HCSBC_CPP_SRCS:%.cpp=$(D3MB6HCSBC_BUILD_DIR)/%.o)
D3MB6HCSBC_C_OBJS := $(D3MB6HCSBC_C_SRCS:%.c=$(D3MB6HCSBC_BUILD_DIR)/%.o)
D3MB6HCSBC_OBJS := $(D3MB6HCSBC_CPP_OBJS) $(D3MB6HCSBC_C_OBJS)

# Dependency files
D3MB6HCSBC_DEPS := $(D3MB6HCSBC_OBJS:.o=.d)

# Target rule
.PHONY: Duet3_MB6HC_SBC
Duet3_MB6HC_SBC: $(D3MB6HCSBC_TARGET_BIN)
	$(Q)echo "========================================"
	$(Q)echo "Duet3_MB6HC_SBC IAP build complete!"
	$(Q)echo "Output: $(D3MB6HCSBC_TARGET_BIN)"
	$(Q)echo "========================================"
	$(Q)$(SIZE) $(D3MB6HCSBC_TARGET_ELF)

# Link ELF file
$(D3MB6HCSBC_TARGET_ELF): $(D3MB6HCSBC_OBJS) $(D3MB6HCSBC_COREN2G_LIB) $(D3MB6HCSBC_RRFLIBS_LIB)
	$(Q)echo "  LD      $@"
	$(Q)mkdir -p $(@D)
	$(Q)$(LD) $(D3MB6HCSBC_LDFLAGS1) -o $@ $(D3MB6HCSBC_LDFLAGS2) -Wl,--start-group $(D3MB6HCSBC_OBJS) $(D3MB6HCSBC_LDLIBS) $(D3MB6HCSBC_LDLIBS_POST)

# Generate binary file
$(D3MB6HCSBC_TARGET_BIN): $(D3MB6HCSBC_TARGET_ELF)
	$(Q)echo "  OBJCOPY $@"
	$(Q)$(OBJCOPY) -O binary $< $@

# Compile C++ files
$(D3MB6HCSBC_BUILD_DIR)/%.o: %.cpp
	$(Q)echo "  CXX     $<"
	$(Q)mkdir -p $(@D)
	$(Q)$(CXX) $(D3MB6HCSBC_CXXFLAGS) -MMD -MP -o $@ $<

# Compile C files
$(D3MB6HCSBC_BUILD_DIR)/%.o: %.c
	$(Q)echo "  CC      $<"
	$(Q)mkdir -p $(@D)
	$(Q)$(CC) $(D3MB6HCSBC_CFLAGS) -MMD -MP -o $@ $<

# Include dependencies
-include $(D3MB6HCSBC_DEPS)

# Clean target
.PHONY: clean-Duet3_MB6HC_SBC
clean-Duet3_MB6HC_SBC:
	$(Q)echo "  RM      $(D3MB6HCSBC_BUILD_DIR)"
	$(Q)rm -rf $(D3MB6HCSBC_BUILD_DIR)
