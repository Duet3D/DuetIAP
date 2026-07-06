# DuetIAP Master Makefile
# Builds In-Application Programming (IAP) binaries for various Duet boards
# IAP binaries are loaded into the last 32KB of RAM before execution

# Cross-compiler toolchain
CROSS_COMPILE ?= ../arm-gnu-toolchain-15.2.rel1-x86_64-arm-none-eabi/bin/arm-none-eabi-
export CROSS_COMPILE

# Toolchain programs
CC  := $(CROSS_COMPILE)gcc
CXX := $(CROSS_COMPILE)g++
AS  := $(CROSS_COMPILE)gcc
AR  := $(CROSS_COMPILE)ar
LD  := $(CROSS_COMPILE)gcc
OBJCOPY := $(CROSS_COMPILE)objcopy
SIZE := $(CROSS_COMPILE)size
export CC CXX AS AR LD OBJCOPY SIZE

# Workspace root
WORKSPACE := ..
export WORKSPACE

# Quiet build support (Linux kernel style)
# Use V=1 for verbose output
ifeq ($(V),1)
	Q :=
	VERBOSE :=
else
	Q := @
	VERBOSE := -s
endif
export Q VERBOSE

# Debug build support
# Use DEBUG=1 to build with debug symbols and reduced optimization
ifeq ($(DEBUG),1)
DEBUG_FLAGS := -g3 -Og -DDEBUG
$(info Building with debug symbols enabled)
else
DEBUG_FLAGS :=
endif
export DEBUG_FLAGS

# Default target
.DEFAULT_GOAL := help

# Available build configurations
SD_CONFIGS := Duet3_MB6HC_SD Duet3_MB6XD_SD Duet3Mini_SD FMDC_SD
SBC_CONFIGS := Duet3_MB6HC_SBC Duet3_MB6XD_SBC Duet3Mini_SBC
CONFIGS := $(SD_CONFIGS) $(SBC_CONFIGS)

# Print available targets
.PHONY: help
help:
	$(Q)echo ""
	$(Q)echo "DuetIAP Build System"
	$(Q)echo "====================="
	$(Q)echo ""
	$(Q)echo "SD card IAP targets (firmware update from SD):"
	$(Q)echo "  Duet3_MB6HC_SD      - Duet 3 MB6HC (SAME70)"
	$(Q)echo "  Duet3_MB6XD_SD      - Duet 3 MB6XD (SAME70)"
	$(Q)echo "  Duet3Mini_SD        - Duet 3 Mini 5+ (SAME54)"
	$(Q)echo "  FMDC_SD             - FMDC (SAME51)"
	$(Q)echo ""
	$(Q)echo "SBC IAP targets (firmware update from SBC via SPI/USB):"
	$(Q)echo "  Duet3_MB6HC_SBC     - Duet 3 MB6HC (SAME70, SPI+USB)"
	$(Q)echo "  Duet3_MB6XD_SBC     - Duet 3 MB6XD (SAME70, SPI+USB)"
	$(Q)echo "  Duet3Mini_SBC       - Duet 3 Mini 5+ (SAME54, SPI+USB)"
	$(Q)echo ""
	$(Q)echo "Other targets:"
	$(Q)echo "  all                 - Build all configurations"
	$(Q)echo "  all-sd              - Build all SD card configurations"
	$(Q)echo "  all-sbc             - Build all SBC configurations"
	$(Q)echo "  clean               - Clean all build outputs"
	$(Q)echo "  clean-all           - Clean all build outputs and libraries"
	$(Q)echo "  clean-<config>      - Clean specific configuration"
	$(Q)echo "  test-toolchain      - Verify toolchain is accessible"
	$(Q)echo ""
	$(Q)echo "Environment variables:"
	$(Q)echo "  CROSS_COMPILE       - Toolchain prefix (default: $(CROSS_COMPILE))"
	$(Q)echo "  V=1                 - Enable verbose build output"
	$(Q)echo "  DEBUG=1             - Build with debug symbols (-g3 -Og)"
	$(Q)echo ""
	$(Q)echo "Examples:"
	$(Q)echo "  make Duet3_MB6HC_SD                         # Build MB6HC SD IAP"
	$(Q)echo "  make Duet3_MB6HC_SBC V=1                    # Build MB6HC SBC with verbose output"
	$(Q)echo "  make all-sd                                 # Build all SD IAP binaries"
	$(Q)echo ""

# Build all configurations
.PHONY: all all-sd all-sbc
all: $(CONFIGS)
all-sd: $(SD_CONFIGS)
all-sbc: $(SBC_CONFIGS)

# Verify toolchain
.PHONY: test-toolchain
test-toolchain:
	$(Q)echo "Testing toolchain..."
	$(Q)if [ ! -f "$(CROSS_COMPILE)gcc" ]; then \
		echo "ERROR: Toolchain not found at: $(CROSS_COMPILE)gcc"; \
		echo "Please install the ARM GCC toolchain and set CROSS_COMPILE"; \
		exit 1; \
	fi
	$(Q)echo "Toolchain: $(CROSS_COMPILE)"
	$(Q)$(CROSS_COMPILE)gcc --version | head -n 1
	$(Q)echo "Toolchain OK"

# Common library build rules
# These are marked as .PHONY so Make always checks if they need rebuilding
.PHONY: $(WORKSPACE)/CoreN2G/SAME70_SDHC/libCoreN2G.a \
        $(WORKSPACE)/CoreN2G/SAME70_SDHC_USB/libCoreN2G.a \
        $(WORKSPACE)/CoreN2G/SAME5x_SDHC/libCoreN2G.a \
        $(WORKSPACE)/CoreN2G/SAME5x_SDHC_USB/libCoreN2G.a \
        $(WORKSPACE)/RRFLibraries/SAME70/libRRFLibraries.a \
        $(WORKSPACE)/RRFLibraries/SAME51/libRRFLibraries.a

$(WORKSPACE)/CoreN2G/SAME70_SDHC/libCoreN2G.a:
	$(Q)echo "  BUILD   CoreN2G/SAME70_SDHC"
	$(Q)$(MAKE) $(VERBOSE) -C $(WORKSPACE)/CoreN2G SAME70_SDHC

$(WORKSPACE)/CoreN2G/SAME70_SDHC_USB/libCoreN2G.a:
	$(Q)echo "  BUILD   CoreN2G/SAME70_SDHC_USB"
	$(Q)$(MAKE) $(VERBOSE) -C $(WORKSPACE)/CoreN2G SAME70_SDHC_USB

$(WORKSPACE)/CoreN2G/SAME5x_SDHC/libCoreN2G.a:
	$(Q)echo "  BUILD   CoreN2G/SAME5x_SDHC"
	$(Q)$(MAKE) $(VERBOSE) -C $(WORKSPACE)/CoreN2G SAME5x_SDHC

$(WORKSPACE)/CoreN2G/SAME5x_SDHC_USB/libCoreN2G.a:
	$(Q)echo "  BUILD   CoreN2G/SAME5x_SDHC_USB"
	$(Q)$(MAKE) $(VERBOSE) -C $(WORKSPACE)/CoreN2G SAME5x_SDHC_USB

$(WORKSPACE)/RRFLibraries/SAME70/libRRFLibraries.a:
	$(Q)echo "  BUILD   RRFLibraries/SAME70"
	$(Q)$(MAKE) $(VERBOSE) -C $(WORKSPACE)/RRFLibraries SAME70

$(WORKSPACE)/RRFLibraries/SAME51/libRRFLibraries.a:
	$(Q)echo "  BUILD   RRFLibraries/SAME51"
	$(Q)$(MAKE) $(VERBOSE) -C $(WORKSPACE)/RRFLibraries SAME51

# Include board-specific makefiles
-include Makefiles/Duet3_MB6HC_SD.mk
-include Makefiles/Duet3_MB6HC_SBC.mk
-include Makefiles/Duet3_MB6XD_SD.mk
-include Makefiles/Duet3_MB6XD_SBC.mk
-include Makefiles/Duet3Mini_SD.mk
-include Makefiles/Duet3Mini_SBC.mk
-include Makefiles/FMDC_SD.mk

# Generic clean target
.PHONY: clean
clean:
	$(Q)echo "Cleaning all build outputs..."
	$(Q)for config in $(CONFIGS); do \
		if [ -d "$$config" ]; then \
			echo "  RM      $$config"; \
			rm -rf "$$config"; \
		fi; \
	done
	$(Q)echo "Clean complete"

# Clean all including libraries
.PHONY: clean-all
clean-all: clean
	$(Q)echo "Cleaning library dependencies..."
	$(Q)$(MAKE) $(VERBOSE) -C $(WORKSPACE)/CoreN2G clean
	$(Q)$(MAKE) $(VERBOSE) -C $(WORKSPACE)/RRFLibraries clean
	$(Q)echo "Clean all complete"
