# DuetIAP SAM4S_SD Configuration Makefile
# Duet 2 Maestro - SD card IAP (SAM4S8C, Cortex-M4, no FPU)

# Build directory and output
SAM4SSD_BUILD_DIR := SAM4S_SD
SAM4SSD_TARGET_NAME := Duet2_SDiap32_Maestro
SAM4SSD_TARGET_ELF := $(SAM4SSD_BUILD_DIR)/$(SAM4SSD_TARGET_NAME).elf
SAM4SSD_TARGET_BIN := $(SAM4SSD_BUILD_DIR)/$(SAM4SSD_TARGET_NAME).bin
SAM4SSD_TARGET_MAP := $(SAM4SSD_BUILD_DIR)/$(SAM4SSD_TARGET_NAME).map

# Workspace root
WORKSPACE := ..

# Library dependencies
SAM4SSD_COREN2G_LIB := $(WORKSPACE)/CoreN2G/SAM4S_SDHC/libCoreN2G.a
SAM4SSD_RRFLIBS_LIB := $(WORKSPACE)/RRFLibraries/SAM4S/libRRFLibraries.a

# Source directories
SAM4SSD_SRC_DIR := src

# Find all source files (SD builds include Libraries/)
SAM4SSD_CPP_SRCS := $(shell find $(SAM4SSD_SRC_DIR) -name '*.cpp')

SAM4SSD_C_SRCS := $(shell find $(SAM4SSD_SRC_DIR) -name '*.c')

# Include paths - C compiler
SAM4SSD_C_INCLUDES := \
	-I$(SAM4SSD_SRC_DIR) \
	-I$(WORKSPACE)/CoreN2G/src \
	-I$(WORKSPACE)/CoreN2G/src/SAM4S_4E_E70 \
	-I$(WORKSPACE)/CoreN2G/src/SAM4S_4E_E70/SAM4S \
	-I$(WORKSPACE)/CoreN2G/src/SAM4S_4E_E70/asf \
	-I$(WORKSPACE)/CoreN2G/src/SAM4S_4E_E70/asf/common/utils \
	-I$(WORKSPACE)/CoreN2G/src/SAM4S_4E_E70/asf/sam/drivers \
	-I$(WORKSPACE)/CoreN2G/src/SAM4S_4E_E70/asf/sam/utils \
	-I$(WORKSPACE)/CoreN2G/src/SAM4S_4E_E70/asf/sam/utils/cmsis/sam4s/include \
	-I$(WORKSPACE)/CoreN2G/src/SAM4S_4E_E70/asf/sam/utils/header_files \
	-I$(WORKSPACE)/CoreN2G/src/SAM4S_4E_E70/asf/sam/utils/preprocessor \
	-I$(WORKSPACE)/CoreN2G/src/arm/CMSIS/5.4.0/CMSIS/Core/Include \
	-I$(WORKSPACE)/RRFLibraries/src

# Include paths - C++ compiler
SAM4SSD_CXX_INCLUDES := \
	-I$(WORKSPACE)/CoreN2G/src \
	-I$(WORKSPACE)/CoreN2G/src/SAM4S_4E_E70 \
	-I$(WORKSPACE)/CoreN2G/src/SAM4S_4E_E70/asf \
	-I$(WORKSPACE)/CoreN2G/src/SAM4S_4E_E70/asf/common/utils \
	-I$(WORKSPACE)/CoreN2G/src/SAM4S_4E_E70/asf/sam/drivers \
	-I$(WORKSPACE)/CoreN2G/src/SAM4S_4E_E70/asf/sam/utils \
	-I$(WORKSPACE)/CoreN2G/src/SAM4S_4E_E70/asf/sam/utils/cmsis/sam4s/include \
	-I$(WORKSPACE)/CoreN2G/src/SAM4S_4E_E70/asf/sam/utils/header_files \
	-I$(WORKSPACE)/CoreN2G/src/SAM4S_4E_E70/asf/sam/utils/preprocessor \
	-I$(WORKSPACE)/CoreN2G/src/SAM4S_4E_E70/SAM4S \
	-I$(WORKSPACE)/CoreN2G/src/arm/CMSIS/5.4.0/CMSIS/Core/Include \
	-I$(SAM4SSD_SRC_DIR)/Libraries/Fatfs \
	-I$(WORKSPACE)/RRFLibraries/src

# Preprocessor defines - C
SAM4SSD_C_DEFINES := \
	-D__SAM4S8C__ \
	-Dnoexcept=

# Preprocessor defines - C++
SAM4SSD_CXX_DEFINES := \
	-D__SAM4S8C__ \
	-DIAP_IN_RAM

# Compiler flags - C (SAM4S has no hardware FPU)
SAM4SSD_CFLAGS := -c -std=gnu99 \
	-Os \
	-mcpu=cortex-m4 \
	-mthumb \
	-mfp16-format=ieee \
	-ffunction-sections \
	-fdata-sections \
	-nostdlib \
	-Wundef \
	-Wdouble-promotion \
	-fsingle-precision-constant \
	$(SAM4SSD_C_INCLUDES) \
	$(SAM4SSD_C_DEFINES) \
	$(DEBUG_FLAGS)

# Compiler flags - C++ (SAM4S has no hardware FPU)
SAM4SSD_CXXFLAGS := -c -std=gnu++17 \
	-Os \
	-mcpu=cortex-m4 \
	-mthumb \
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
	$(SAM4SSD_CXX_INCLUDES) \
	$(SAM4SSD_CXX_DEFINES) \
	$(DEBUG_FLAGS)

# Linker flags - before -o (no FPU flags for SAM4S)
SAM4SSD_LDFLAGS1 := --specs=nano.specs \
	-Os \
	-Wl,--gc-sections \
	-Wl,--fatal-warnings \
	-mcpu=cortex-m4 \
	-T$(SAM4SSD_SRC_DIR)/LinkerScripts/sam4s_iap_ram.ld \
	-Wl,-Map,$(SAM4SSD_TARGET_MAP) \
	-mthumb

# Linker flags - after -o
SAM4SSD_LDFLAGS2 := \
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
SAM4SSD_LDLIBS := \
	-L$(WORKSPACE)/CoreN2G/SAM4S_SDHC \
	-L$(WORKSPACE)/RRFLibraries/SAM4S \
	-lCoreN2G \
	-lRRFLibraries \
	-lsupc++

SAM4SSD_LDLIBS_POST := -Wl,--end-group -lm

# Object files
SAM4SSD_CPP_OBJS := $(SAM4SSD_CPP_SRCS:%.cpp=$(SAM4SSD_BUILD_DIR)/%.o)
SAM4SSD_C_OBJS := $(SAM4SSD_C_SRCS:%.c=$(SAM4SSD_BUILD_DIR)/%.o)
SAM4SSD_OBJS := $(SAM4SSD_CPP_OBJS) $(SAM4SSD_C_OBJS)

# Dependency files
SAM4SSD_DEPS := $(SAM4SSD_OBJS:.o=.d)

# Target rule
.PHONY: SAM4S_SD
SAM4S_SD: $(SAM4SSD_TARGET_BIN)
	$(Q)echo "========================================"
	$(Q)echo "SAM4S_SD IAP build complete!"
	$(Q)echo "Output: $(SAM4SSD_TARGET_BIN)"
	$(Q)echo "========================================"
	$(Q)$(SIZE) $(SAM4SSD_TARGET_ELF)

# Link ELF file
$(SAM4SSD_TARGET_ELF): $(SAM4SSD_OBJS) $(SAM4SSD_COREN2G_LIB) $(SAM4SSD_RRFLIBS_LIB)
	$(Q)echo "  LD      $@"
	$(Q)mkdir -p $(@D)
	$(Q)$(LD) $(SAM4SSD_LDFLAGS1) -o $@ $(SAM4SSD_LDFLAGS2) -Wl,--start-group $(SAM4SSD_OBJS) $(SAM4SSD_LDLIBS) $(SAM4SSD_LDLIBS_POST)

# Generate binary file
$(SAM4SSD_TARGET_BIN): $(SAM4SSD_TARGET_ELF)
	$(Q)echo "  OBJCOPY $@"
	$(Q)$(OBJCOPY) -O binary $< $@

# Compile C++ files
$(SAM4SSD_BUILD_DIR)/%.o: %.cpp
	$(Q)echo "  CXX     $<"
	$(Q)mkdir -p $(@D)
	$(Q)$(CXX) $(SAM4SSD_CXXFLAGS) -MMD -MP -o $@ $<

# Compile C files
$(SAM4SSD_BUILD_DIR)/%.o: %.c
	$(Q)echo "  CC      $<"
	$(Q)mkdir -p $(@D)
	$(Q)$(CC) $(SAM4SSD_CFLAGS) -MMD -MP -o $@ $<

# Include dependencies
-include $(SAM4SSD_DEPS)

# Clean target
.PHONY: clean-SAM4S_SD
clean-SAM4S_SD:
	$(Q)echo "  RM      $(SAM4SSD_BUILD_DIR)"
	$(Q)rm -rf $(SAM4SSD_BUILD_DIR)
