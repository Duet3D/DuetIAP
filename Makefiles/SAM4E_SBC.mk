# DuetIAP SAM4E_SBC Configuration Makefile
# Duet 2 WiFi/Ethernet - SBC/SPI IAP (SAM4E8E, Cortex-M4)

# Build directory and output
SAM4ESBC_BUILD_DIR := SAM4E_SBC
SAM4ESBC_TARGET_NAME := Duet2_SBCiap32_SBC
SAM4ESBC_TARGET_ELF := $(SAM4ESBC_BUILD_DIR)/$(SAM4ESBC_TARGET_NAME).elf
SAM4ESBC_TARGET_BIN := $(SAM4ESBC_BUILD_DIR)/$(SAM4ESBC_TARGET_NAME).bin
SAM4ESBC_TARGET_MAP := $(SAM4ESBC_BUILD_DIR)/$(SAM4ESBC_TARGET_NAME).map

# Workspace root
WORKSPACE := ..

# Library dependencies
SAM4ESBC_COREN2G_LIB := $(WORKSPACE)/CoreN2G/SAM4E_SDHC/libCoreN2G.a
SAM4ESBC_RRFLIBS_LIB := $(WORKSPACE)/RRFLibraries/SAM4E/libRRFLibraries.a

# Source directories
SAM4ESBC_SRC_DIR := src

# Find all source files (SPI builds exclude Libraries/)
SAM4ESBC_CPP_SRCS := $(shell find $(SAM4ESBC_SRC_DIR) -name '*.cpp' \
	! -path '*/Libraries/*')

SAM4ESBC_C_SRCS := $(shell find $(SAM4ESBC_SRC_DIR) -name '*.c' \
	! -path '*/Libraries/*')

# Include paths - C compiler
SAM4ESBC_C_INCLUDES := \
	-I$(WORKSPACE)/CoreN2G/src/SAM4S_4E_E70/asf \
	-I$(WORKSPACE)/CoreN2G/src/SAM4S_4E_E70/asf/common/utils \
	-I$(WORKSPACE)/CoreN2G/src/SAM4S_4E_E70/asf/sam/drivers \
	-I$(WORKSPACE)/CoreN2G/src/SAM4S_4E_E70/asf/sam/utils \
	-I$(WORKSPACE)/CoreN2G/src/SAM4S_4E_E70/asf/sam/utils/cmsis/sam4e/include \
	-I$(WORKSPACE)/CoreN2G/src/SAM4S_4E_E70/asf/sam/utils/header_files \
	-I$(WORKSPACE)/CoreN2G/src/SAM4S_4E_E70/asf/sam/utils/preprocessor \
	-I$(WORKSPACE)/CoreN2G/src/arm/CMSIS/5.4.0/CMSIS/Core/Include

# Include paths - C++ compiler
SAM4ESBC_CXX_INCLUDES := \
	-I$(WORKSPACE)/CoreN2G/src \
	-I$(WORKSPACE)/CoreN2G/src/SAM4S_4E_E70 \
	-I$(WORKSPACE)/CoreN2G/src/SAM4S_4E_E70/asf/common/utils \
	-I$(WORKSPACE)/CoreN2G/src/SAM4S_4E_E70/asf/sam/utils \
	-I$(WORKSPACE)/CoreN2G/src/SAM4S_4E_E70/asf/sam/utils/cmsis/sam4e/include \
	-I$(WORKSPACE)/CoreN2G/src/SAM4S_4E_E70/asf/sam/utils/header_files \
	-I$(WORKSPACE)/CoreN2G/src/SAM4S_4E_E70/asf/sam/utils/preprocessor \
	-I$(WORKSPACE)/CoreN2G/src/SAM4S_4E_E70/SAM4E \
	-I$(WORKSPACE)/CoreN2G/src/arm/CMSIS/5.4.0/CMSIS/Core/Include \
	-I$(SAM4ESBC_SRC_DIR)/Libraries/Fatfs \
	-I$(WORKSPACE)/RRFLibraries/src

# Preprocessor defines - C
SAM4ESBC_C_DEFINES := \
	-D__SAM4E8E__ \
	-DDUET_NG \
	-Dnoexcept=

# Preprocessor defines - C++
SAM4ESBC_CXX_DEFINES := \
	-D__SAM4E8E__ \
	-DDUET_NG \
	-DIAP_IN_RAM \
	-DIAP_SBC_SPI

# Compiler flags - C
SAM4ESBC_CFLAGS := -c -std=gnu99 \
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
	$(SAM4ESBC_C_INCLUDES) \
	$(SAM4ESBC_C_DEFINES) \
	$(DEBUG_FLAGS)

# Compiler flags - C++
SAM4ESBC_CXXFLAGS := -c -std=c++20 \
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
	$(SAM4ESBC_CXX_INCLUDES) \
	$(SAM4ESBC_CXX_DEFINES) \
	$(DEBUG_FLAGS)

# Linker flags - before -o
SAM4ESBC_LDFLAGS1 := --specs=nano.specs \
	-Os \
	-Wl,--gc-sections \
	-Wl,--fatal-warnings \
	-mcpu=cortex-m4 \
	-mfpu=fpv4-sp-d16 \
	-mfloat-abi=hard \
	-T$(SAM4ESBC_SRC_DIR)/LinkerScripts/sam4e_iap_ram.ld \
	-Wl,-Map,$(SAM4ESBC_TARGET_MAP) \
	-mthumb

# Linker flags - after -o
SAM4ESBC_LDFLAGS2 := \
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
SAM4ESBC_LDLIBS := \
	-L$(WORKSPACE)/CoreN2G/SAM4E_SDHC \
	-L$(WORKSPACE)/RRFLibraries/SAM4E \
	-lCoreN2G \
	-lRRFLibraries \
	-lsupc++

SAM4ESBC_LDLIBS_POST := -Wl,--end-group -lm

# Object files
SAM4ESBC_CPP_OBJS := $(SAM4ESBC_CPP_SRCS:%.cpp=$(SAM4ESBC_BUILD_DIR)/%.o)
SAM4ESBC_C_OBJS := $(SAM4ESBC_C_SRCS:%.c=$(SAM4ESBC_BUILD_DIR)/%.o)
SAM4ESBC_OBJS := $(SAM4ESBC_CPP_OBJS) $(SAM4ESBC_C_OBJS)

# Dependency files
SAM4ESBC_DEPS := $(SAM4ESBC_OBJS:.o=.d)

# Target rule
.PHONY: SAM4E_SBC
SAM4E_SBC: $(SAM4ESBC_TARGET_BIN)
	$(Q)echo "========================================"
	$(Q)echo "SAM4E_SBC IAP build complete!"
	$(Q)echo "Output: $(SAM4ESBC_TARGET_BIN)"
	$(Q)echo "========================================"
	$(Q)$(SIZE) $(SAM4ESBC_TARGET_ELF)

# Link ELF file
$(SAM4ESBC_TARGET_ELF): $(SAM4ESBC_OBJS) $(SAM4ESBC_COREN2G_LIB) $(SAM4ESBC_RRFLIBS_LIB)
	$(Q)echo "  LD      $@"
	$(Q)mkdir -p $(@D)
	$(Q)$(LD) $(SAM4ESBC_LDFLAGS1) -o $@ $(SAM4ESBC_LDFLAGS2) -Wl,--start-group $(SAM4ESBC_OBJS) $(SAM4ESBC_LDLIBS) $(SAM4ESBC_LDLIBS_POST)

# Generate binary file
$(SAM4ESBC_TARGET_BIN): $(SAM4ESBC_TARGET_ELF)
	$(Q)echo "  OBJCOPY $@"
	$(Q)$(OBJCOPY) -O binary $< $@

# Compile C++ files
$(SAM4ESBC_BUILD_DIR)/%.o: %.cpp
	$(Q)echo "  CXX     $<"
	$(Q)mkdir -p $(@D)
	$(Q)$(CXX) $(SAM4ESBC_CXXFLAGS) -MMD -MP -o $@ $<

# Compile C files
$(SAM4ESBC_BUILD_DIR)/%.o: %.c
	$(Q)echo "  CC      $<"
	$(Q)mkdir -p $(@D)
	$(Q)$(CC) $(SAM4ESBC_CFLAGS) -MMD -MP -o $@ $<

# Include dependencies
-include $(SAM4ESBC_DEPS)

# Clean target
.PHONY: clean-SAM4E_SBC
clean-SAM4E_SBC:
	$(Q)echo "  RM      $(SAM4ESBC_BUILD_DIR)"
	$(Q)rm -rf $(SAM4ESBC_BUILD_DIR)
