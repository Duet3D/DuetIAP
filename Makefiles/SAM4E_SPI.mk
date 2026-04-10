# DuetIAP SAM4E_SPI Configuration Makefile
# Duet 2 WiFi/Ethernet - SBC/SPI IAP (SAM4E8E, Cortex-M4)

# Build directory and output
SAM4ESPI_BUILD_DIR := SAM4E_SPI
SAM4ESPI_TARGET_NAME := Duet2_SBCiap32_SBC
SAM4ESPI_TARGET_ELF := $(SAM4ESPI_BUILD_DIR)/$(SAM4ESPI_TARGET_NAME).elf
SAM4ESPI_TARGET_BIN := $(SAM4ESPI_BUILD_DIR)/$(SAM4ESPI_TARGET_NAME).bin
SAM4ESPI_TARGET_MAP := $(SAM4ESPI_BUILD_DIR)/$(SAM4ESPI_TARGET_NAME).map

# Workspace root
WORKSPACE := ..

# Library dependencies
SAM4ESPI_COREN2G_LIB := $(WORKSPACE)/CoreN2G/SAM4E_SDHC/libCoreN2G.a
SAM4ESPI_RRFLIBS_LIB := $(WORKSPACE)/RRFLibraries/SAM4E/libRRFLibraries.a

# Source directories
SAM4ESPI_SRC_DIR := src

# Find all source files (SPI builds exclude Libraries/)
SAM4ESPI_CPP_SRCS := $(shell find $(SAM4ESPI_SRC_DIR) -name '*.cpp' \
	! -path '*/Libraries/*')

SAM4ESPI_C_SRCS := $(shell find $(SAM4ESPI_SRC_DIR) -name '*.c' \
	! -path '*/Libraries/*')

# Include paths - C compiler
SAM4ESPI_C_INCLUDES := \
	-I$(WORKSPACE)/CoreN2G/src/SAM4S_4E_E70/asf \
	-I$(WORKSPACE)/CoreN2G/src/SAM4S_4E_E70/asf/common/utils \
	-I$(WORKSPACE)/CoreN2G/src/SAM4S_4E_E70/asf/sam/drivers \
	-I$(WORKSPACE)/CoreN2G/src/SAM4S_4E_E70/asf/sam/utils \
	-I$(WORKSPACE)/CoreN2G/src/SAM4S_4E_E70/asf/sam/utils/cmsis/sam4e/include \
	-I$(WORKSPACE)/CoreN2G/src/SAM4S_4E_E70/asf/sam/utils/header_files \
	-I$(WORKSPACE)/CoreN2G/src/SAM4S_4E_E70/asf/sam/utils/preprocessor \
	-I$(WORKSPACE)/CoreN2G/src/arm/CMSIS/5.4.0/CMSIS/Core/Include

# Include paths - C++ compiler
SAM4ESPI_CXX_INCLUDES := \
	-I$(WORKSPACE)/CoreN2G/src \
	-I$(WORKSPACE)/CoreN2G/src/SAM4S_4E_E70 \
	-I$(WORKSPACE)/CoreN2G/src/SAM4S_4E_E70/asf/common/utils \
	-I$(WORKSPACE)/CoreN2G/src/SAM4S_4E_E70/asf/sam/utils \
	-I$(WORKSPACE)/CoreN2G/src/SAM4S_4E_E70/asf/sam/utils/cmsis/sam4e/include \
	-I$(WORKSPACE)/CoreN2G/src/SAM4S_4E_E70/asf/sam/utils/header_files \
	-I$(WORKSPACE)/CoreN2G/src/SAM4S_4E_E70/asf/sam/utils/preprocessor \
	-I$(WORKSPACE)/CoreN2G/src/SAM4S_4E_E70/SAM4E \
	-I$(WORKSPACE)/CoreN2G/src/arm/CMSIS/5.4.0/CMSIS/Core/Include \
	-I$(SAM4ESPI_SRC_DIR)/Libraries/Fatfs \
	-I$(WORKSPACE)/RRFLibraries/src

# Preprocessor defines - C
SAM4ESPI_C_DEFINES := \
	-D__SAM4E8E__ \
	-DDUET_NG \
	-Dnoexcept=

# Preprocessor defines - C++
SAM4ESPI_CXX_DEFINES := \
	-D__SAM4E8E__ \
	-DDUET_NG \
	-DIAP_IN_RAM \
	-DIAP_VIA_SPI

# Compiler flags - C
SAM4ESPI_CFLAGS := -c -std=gnu99 \
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
	$(SAM4ESPI_C_INCLUDES) \
	$(SAM4ESPI_C_DEFINES) \
	$(DEBUG_FLAGS)

# Compiler flags - C++
SAM4ESPI_CXXFLAGS := -c -std=gnu++17 \
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
	$(SAM4ESPI_CXX_INCLUDES) \
	$(SAM4ESPI_CXX_DEFINES) \
	$(DEBUG_FLAGS)

# Linker flags - before -o
SAM4ESPI_LDFLAGS1 := --specs=nano.specs \
	-Os \
	-Wl,--gc-sections \
	-Wl,--fatal-warnings \
	-mcpu=cortex-m4 \
	-mfpu=fpv4-sp-d16 \
	-mfloat-abi=hard \
	-T$(SAM4ESPI_SRC_DIR)/LinkerScripts/sam4e_iap_ram.ld \
	-Wl,-Map,$(SAM4ESPI_TARGET_MAP) \
	-mthumb

# Linker flags - after -o
SAM4ESPI_LDFLAGS2 := \
	-Wl,--cref \
	-Wl,--check-sections \
	-Wl,--gc-sections \
	-Wl,--entry=Reset_Handler \
	-Wl,--unresolved-symbols=report-all \
	-Wl,--warn-common \
	-Wl,--warn-section-align \
	-Wl,--warn-unresolved-symbols

# Library search paths and libraries
SAM4ESPI_LDLIBS := \
	-L$(WORKSPACE)/CoreN2G/SAM4E_SDHC \
	-L$(WORKSPACE)/RRFLibraries/SAM4E \
	-lCoreN2G \
	-lRRFLibraries \
	-lsupc++

SAM4ESPI_LDLIBS_POST := -Wl,--end-group -lm

# Object files
SAM4ESPI_CPP_OBJS := $(SAM4ESPI_CPP_SRCS:%.cpp=$(SAM4ESPI_BUILD_DIR)/%.o)
SAM4ESPI_C_OBJS := $(SAM4ESPI_C_SRCS:%.c=$(SAM4ESPI_BUILD_DIR)/%.o)
SAM4ESPI_OBJS := $(SAM4ESPI_CPP_OBJS) $(SAM4ESPI_C_OBJS)

# Dependency files
SAM4ESPI_DEPS := $(SAM4ESPI_OBJS:.o=.d)

# Target rule
.PHONY: SAM4E_SPI
SAM4E_SPI: $(SAM4ESPI_TARGET_BIN)
	$(Q)echo "========================================"
	$(Q)echo "SAM4E_SPI IAP build complete!"
	$(Q)echo "Output: $(SAM4ESPI_TARGET_BIN)"
	$(Q)echo "========================================"
	$(Q)$(SIZE) $(SAM4ESPI_TARGET_ELF)

# Link ELF file
$(SAM4ESPI_TARGET_ELF): $(SAM4ESPI_OBJS) $(SAM4ESPI_COREN2G_LIB) $(SAM4ESPI_RRFLIBS_LIB)
	$(Q)echo "  LD      $@"
	$(Q)mkdir -p $(@D)
	$(Q)$(LD) $(SAM4ESPI_LDFLAGS1) -o $@ $(SAM4ESPI_LDFLAGS2) -Wl,--start-group $(SAM4ESPI_OBJS) $(SAM4ESPI_LDLIBS) $(SAM4ESPI_LDLIBS_POST)

# Generate binary file
$(SAM4ESPI_TARGET_BIN): $(SAM4ESPI_TARGET_ELF)
	$(Q)echo "  OBJCOPY $@"
	$(Q)$(OBJCOPY) -O binary $< $@

# Compile C++ files
$(SAM4ESPI_BUILD_DIR)/%.o: %.cpp
	$(Q)echo "  CXX     $<"
	$(Q)mkdir -p $(@D)
	$(Q)$(CXX) $(SAM4ESPI_CXXFLAGS) -MMD -MP -o $@ $<

# Compile C files
$(SAM4ESPI_BUILD_DIR)/%.o: %.c
	$(Q)echo "  CC      $<"
	$(Q)mkdir -p $(@D)
	$(Q)$(CC) $(SAM4ESPI_CFLAGS) -MMD -MP -o $@ $<

# Include dependencies
-include $(SAM4ESPI_DEPS)

# Clean target
.PHONY: clean-SAM4E_SPI
clean-SAM4E_SPI:
	$(Q)echo "  RM      $(SAM4ESPI_BUILD_DIR)"
	$(Q)rm -rf $(SAM4ESPI_BUILD_DIR)
