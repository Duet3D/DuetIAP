# DuetIAP SAM4E_SD Configuration Makefile
# Duet 2 WiFi/Ethernet - SD card IAP (SAM4E8E, Cortex-M4)

# Build directory and output
SAM4ESD_BUILD_DIR := SAM4E_SD
SAM4ESD_TARGET_NAME := Duet2_SDiap32_WiFiEth
SAM4ESD_TARGET_ELF := $(SAM4ESD_BUILD_DIR)/$(SAM4ESD_TARGET_NAME).elf
SAM4ESD_TARGET_BIN := $(SAM4ESD_BUILD_DIR)/$(SAM4ESD_TARGET_NAME).bin
SAM4ESD_TARGET_MAP := $(SAM4ESD_BUILD_DIR)/$(SAM4ESD_TARGET_NAME).map

# Workspace root
WORKSPACE := ..

# Library dependencies
SAM4ESD_COREN2G_LIB := $(WORKSPACE)/CoreN2G/SAM4E_SDHC/libCoreN2G.a
SAM4ESD_RRFLIBS_LIB := $(WORKSPACE)/RRFLibraries/SAM4E/libRRFLibraries.a

# Source directories
SAM4ESD_SRC_DIR := src

# Find all source files (SD builds include Libraries/)
SAM4ESD_CPP_SRCS := $(shell find $(SAM4ESD_SRC_DIR) -name '*.cpp')

SAM4ESD_C_SRCS := $(shell find $(SAM4ESD_SRC_DIR) -name '*.c')

# Include paths - C compiler
SAM4ESD_C_INCLUDES := \
	-I$(SAM4ESD_SRC_DIR) \
	-I$(WORKSPACE)/CoreN2G/src \
	-I$(WORKSPACE)/CoreN2G/src/SAM4S_4E_E70 \
	-I$(WORKSPACE)/CoreN2G/src/SAM4S_4E_E70/SAM4E \
	-I$(WORKSPACE)/CoreN2G/src/SAM4S_4E_E70/asf \
	-I$(WORKSPACE)/CoreN2G/src/SAM4S_4E_E70/asf/common/utils \
	-I$(WORKSPACE)/CoreN2G/src/SAM4S_4E_E70/asf/sam/drivers \
	-I$(WORKSPACE)/CoreN2G/src/SAM4S_4E_E70/asf/sam/utils \
	-I$(WORKSPACE)/CoreN2G/src/SAM4S_4E_E70/asf/sam/utils/cmsis/sam4e/include \
	-I$(WORKSPACE)/CoreN2G/src/SAM4S_4E_E70/asf/sam/utils/header_files \
	-I$(WORKSPACE)/CoreN2G/src/SAM4S_4E_E70/asf/sam/utils/preprocessor \
	-I$(WORKSPACE)/CoreN2G/src/arm/CMSIS/5.4.0/CMSIS/Core/Include \
	-I$(WORKSPACE)/RRFLibraries/src

# Include paths - C++ compiler
SAM4ESD_CXX_INCLUDES := \
	-I$(WORKSPACE)/CoreN2G/src \
	-I$(WORKSPACE)/CoreN2G/src/SAM4S_4E_E70 \
	-I$(WORKSPACE)/CoreN2G/src/SAM4S_4E_E70/asf \
	-I$(WORKSPACE)/CoreN2G/src/SAM4S_4E_E70/asf/common/utils \
	-I$(WORKSPACE)/CoreN2G/src/SAM4S_4E_E70/asf/sam/drivers \
	-I$(WORKSPACE)/CoreN2G/src/SAM4S_4E_E70/asf/sam/utils \
	-I$(WORKSPACE)/CoreN2G/src/SAM4S_4E_E70/asf/sam/utils/cmsis/sam4e/include \
	-I$(WORKSPACE)/CoreN2G/src/SAM4S_4E_E70/asf/sam/utils/header_files \
	-I$(WORKSPACE)/CoreN2G/src/SAM4S_4E_E70/asf/sam/utils/preprocessor \
	-I$(WORKSPACE)/CoreN2G/src/SAM4S_4E_E70/SAM4E \
	-I$(WORKSPACE)/CoreN2G/src/arm/CMSIS/5.4.0/CMSIS/Core/Include \
	-I$(SAM4ESD_SRC_DIR)/Libraries/Fatfs \
	-I$(WORKSPACE)/RRFLibraries/src

# Preprocessor defines - C
SAM4ESD_C_DEFINES := \
	-D__SAM4E8E__ \
	-DDUET_NG \
	-Dnoexcept=

# Preprocessor defines - C++
SAM4ESD_CXX_DEFINES := \
	-D__SAM4E8E__ \
	-DDUET_NG \
	-DIAP_IN_RAM

# Compiler flags - C
SAM4ESD_CFLAGS := -c -std=gnu99 \
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
	$(SAM4ESD_C_INCLUDES) \
	$(SAM4ESD_C_DEFINES) \
	$(DEBUG_FLAGS)

# Compiler flags - C++
SAM4ESD_CXXFLAGS := -c -std=c++20 \
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
	$(SAM4ESD_CXX_INCLUDES) \
	$(SAM4ESD_CXX_DEFINES) \
	$(DEBUG_FLAGS)

# Linker flags - before -o
SAM4ESD_LDFLAGS1 := --specs=nano.specs \
	-Os \
	-Wl,--gc-sections \
	-Wl,--fatal-warnings \
	-mcpu=cortex-m4 \
	-mfpu=fpv4-sp-d16 \
	-mfloat-abi=hard \
	-T$(SAM4ESD_SRC_DIR)/LinkerScripts/sam4e_iap_ram.ld \
	-Wl,-Map,$(SAM4ESD_TARGET_MAP) \
	-mthumb

# Linker flags - after -o
SAM4ESD_LDFLAGS2 := \
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
SAM4ESD_LDLIBS := \
	-L$(WORKSPACE)/CoreN2G/SAM4E_SDHC \
	-L$(WORKSPACE)/RRFLibraries/SAM4E \
	-lCoreN2G \
	-lRRFLibraries \
	-lsupc++

SAM4ESD_LDLIBS_POST := -Wl,--end-group -lm

# Object files
SAM4ESD_CPP_OBJS := $(SAM4ESD_CPP_SRCS:%.cpp=$(SAM4ESD_BUILD_DIR)/%.o)
SAM4ESD_C_OBJS := $(SAM4ESD_C_SRCS:%.c=$(SAM4ESD_BUILD_DIR)/%.o)
SAM4ESD_OBJS := $(SAM4ESD_CPP_OBJS) $(SAM4ESD_C_OBJS)

# Dependency files
SAM4ESD_DEPS := $(SAM4ESD_OBJS:.o=.d)

# Target rule
.PHONY: SAM4E_SD
SAM4E_SD: $(SAM4ESD_TARGET_BIN)
	$(Q)echo "========================================"
	$(Q)echo "SAM4E_SD IAP build complete!"
	$(Q)echo "Output: $(SAM4ESD_TARGET_BIN)"
	$(Q)echo "========================================"
	$(Q)$(SIZE) $(SAM4ESD_TARGET_ELF)

# Link ELF file
$(SAM4ESD_TARGET_ELF): $(SAM4ESD_OBJS) $(SAM4ESD_COREN2G_LIB) $(SAM4ESD_RRFLIBS_LIB)
	$(Q)echo "  LD      $@"
	$(Q)mkdir -p $(@D)
	$(Q)$(LD) $(SAM4ESD_LDFLAGS1) -o $@ $(SAM4ESD_LDFLAGS2) -Wl,--start-group $(SAM4ESD_OBJS) $(SAM4ESD_LDLIBS) $(SAM4ESD_LDLIBS_POST)

# Generate binary file
$(SAM4ESD_TARGET_BIN): $(SAM4ESD_TARGET_ELF)
	$(Q)echo "  OBJCOPY $@"
	$(Q)$(OBJCOPY) -O binary $< $@

# Compile C++ files
$(SAM4ESD_BUILD_DIR)/%.o: %.cpp
	$(Q)echo "  CXX     $<"
	$(Q)mkdir -p $(@D)
	$(Q)$(CXX) $(SAM4ESD_CXXFLAGS) -MMD -MP -o $@ $<

# Compile C files
$(SAM4ESD_BUILD_DIR)/%.o: %.c
	$(Q)echo "  CC      $<"
	$(Q)mkdir -p $(@D)
	$(Q)$(CC) $(SAM4ESD_CFLAGS) -MMD -MP -o $@ $<

# Include dependencies
-include $(SAM4ESD_DEPS)

# Clean target
.PHONY: clean-SAM4E_SD
clean-SAM4E_SD:
	$(Q)echo "  RM      $(SAM4ESD_BUILD_DIR)"
	$(Q)rm -rf $(SAM4ESD_BUILD_DIR)
