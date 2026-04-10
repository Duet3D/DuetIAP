# DuetIAP Duet3Mini_SPI Configuration Makefile
# Duet 3 Mini 5+ - SBC/SPI IAP (SAME54P20A, Cortex-M4)

# Build directory and output
D3MINISPI_BUILD_DIR := Duet3Mini_SPI
D3MINISPI_TARGET_NAME := Duet3_SBCiap32_Mini5plus
D3MINISPI_TARGET_ELF := $(D3MINISPI_BUILD_DIR)/$(D3MINISPI_TARGET_NAME).elf
D3MINISPI_TARGET_BIN := $(D3MINISPI_BUILD_DIR)/$(D3MINISPI_TARGET_NAME).bin
D3MINISPI_TARGET_MAP := $(D3MINISPI_BUILD_DIR)/$(D3MINISPI_TARGET_NAME).map

# Workspace root
WORKSPACE := ..

# Library dependencies
D3MINISPI_COREN2G_LIB := $(WORKSPACE)/CoreN2G/SAME5x_SDHC/libCoreN2G.a
D3MINISPI_RRFLIBS_LIB := $(WORKSPACE)/RRFLibraries/SAME51/libRRFLibraries.a

# Source directories
D3MINISPI_SRC_DIR := src

# Find all source files (SPI builds exclude Libraries/)
D3MINISPI_CPP_SRCS := $(shell find $(D3MINISPI_SRC_DIR) -name '*.cpp' \
	! -path '*/Libraries/*')

D3MINISPI_C_SRCS := $(shell find $(D3MINISPI_SRC_DIR) -name '*.c' \
	! -path '*/Libraries/*')

# Include paths - C compiler
D3MINISPI_C_INCLUDES := \
	-I$(D3MINISPI_SRC_DIR) \
	-I$(WORKSPACE)/CoreN2G/src \
	-I$(WORKSPACE)/CoreN2G/src/SAME5x_C21/SAME5x \
	-I$(WORKSPACE)/CoreN2G/src/SAME5x_C21/SAME5x/Config \
	-I$(WORKSPACE)/CoreN2G/src/atmel/SAME54_DFP/1.1.134/include \
	-I$(WORKSPACE)/CoreN2G/src/arm/CMSIS/5.4.0/CMSIS/Core/Include \
	-I$(WORKSPACE)/RRFLibraries/src

# Include paths - C++ compiler
D3MINISPI_CXX_INCLUDES := \
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
	-I$(D3MINISPI_SRC_DIR)/Libraries/Fatfs

# Preprocessor defines - C
D3MINISPI_C_DEFINES := \
	-D__SAME54P20A__ \
	-DDUET3_MINI \
	-Dnoexcept=

# Preprocessor defines - C++
D3MINISPI_CXX_DEFINES := \
	-D__SAME54P20A__ \
	-DDUET3_MINI \
	-DIAP_IN_RAM \
	-DIAP_VIA_SPI

# Compiler flags - C
D3MINISPI_CFLAGS := -c -std=gnu99 \
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
	$(D3MINISPI_C_INCLUDES) \
	$(D3MINISPI_C_DEFINES) \
	$(DEBUG_FLAGS)

# Compiler flags - C++
D3MINISPI_CXXFLAGS := -c -std=gnu++17 \
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
	$(D3MINISPI_CXX_INCLUDES) \
	$(D3MINISPI_CXX_DEFINES) \
	$(DEBUG_FLAGS)

# Linker flags - before -o
D3MINISPI_LDFLAGS1 := --specs=nano.specs \
	-Os \
	-Wl,--gc-sections \
	-Wl,--fatal-warnings \
	-mcpu=cortex-m4 \
	-mfpu=fpv4-sp-d16 \
	-mfloat-abi=hard \
	-T$(D3MINISPI_SRC_DIR)/LinkerScripts/same54p20a_iap_ram.ld \
	-Wl,-Map,$(D3MINISPI_TARGET_MAP) \
	-mthumb

# Linker flags - after -o
D3MINISPI_LDFLAGS2 := \
	-Wl,--cref \
	-Wl,--check-sections \
	-Wl,--gc-sections \
	-Wl,--entry=Reset_Handler \
	-Wl,--unresolved-symbols=report-all \
	-Wl,--warn-common \
	-Wl,--warn-section-align \
	-Wl,--warn-unresolved-symbols

# Library search paths and libraries
D3MINISPI_LDLIBS := \
	-L$(WORKSPACE)/CoreN2G/SAME5x_SDHC \
	-L$(WORKSPACE)/RRFLibraries/SAME51 \
	-lCoreN2G \
	-lRRFLibraries \
	-lsupc++

D3MINISPI_LDLIBS_POST := -Wl,--end-group -lm

# Object files
D3MINISPI_CPP_OBJS := $(D3MINISPI_CPP_SRCS:%.cpp=$(D3MINISPI_BUILD_DIR)/%.o)
D3MINISPI_C_OBJS := $(D3MINISPI_C_SRCS:%.c=$(D3MINISPI_BUILD_DIR)/%.o)
D3MINISPI_OBJS := $(D3MINISPI_CPP_OBJS) $(D3MINISPI_C_OBJS)

# Dependency files
D3MINISPI_DEPS := $(D3MINISPI_OBJS:.o=.d)

# Target rule
.PHONY: Duet3Mini_SPI
Duet3Mini_SPI: $(D3MINISPI_TARGET_BIN)
	$(Q)echo "========================================"
	$(Q)echo "Duet3Mini_SPI IAP build complete!"
	$(Q)echo "Output: $(D3MINISPI_TARGET_BIN)"
	$(Q)echo "========================================"
	$(Q)$(SIZE) $(D3MINISPI_TARGET_ELF)

# Link ELF file
$(D3MINISPI_TARGET_ELF): $(D3MINISPI_OBJS) $(D3MINISPI_COREN2G_LIB) $(D3MINISPI_RRFLIBS_LIB)
	$(Q)echo "  LD      $@"
	$(Q)mkdir -p $(@D)
	$(Q)$(LD) $(D3MINISPI_LDFLAGS1) -o $@ $(D3MINISPI_LDFLAGS2) -Wl,--start-group $(D3MINISPI_OBJS) $(D3MINISPI_LDLIBS) $(D3MINISPI_LDLIBS_POST)

# Generate binary file
$(D3MINISPI_TARGET_BIN): $(D3MINISPI_TARGET_ELF)
	$(Q)echo "  OBJCOPY $@"
	$(Q)$(OBJCOPY) -O binary $< $@

# Compile C++ files
$(D3MINISPI_BUILD_DIR)/%.o: %.cpp
	$(Q)echo "  CXX     $<"
	$(Q)mkdir -p $(@D)
	$(Q)$(CXX) $(D3MINISPI_CXXFLAGS) -MMD -MP -o $@ $<

# Compile C files
$(D3MINISPI_BUILD_DIR)/%.o: %.c
	$(Q)echo "  CC      $<"
	$(Q)mkdir -p $(@D)
	$(Q)$(CC) $(D3MINISPI_CFLAGS) -MMD -MP -o $@ $<

# Include dependencies
-include $(D3MINISPI_DEPS)

# Clean target
.PHONY: clean-Duet3Mini_SPI
clean-Duet3Mini_SPI:
	$(Q)echo "  RM      $(D3MINISPI_BUILD_DIR)"
	$(Q)rm -rf $(D3MINISPI_BUILD_DIR)
