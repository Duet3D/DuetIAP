# The DuetIAP boards - the in-application programmers that write new RepRapFirmware to flash from
# the SD card (_SD) or from the SBC (_SBC) - everything about DUET_BOARD apart from its MCU's
# toolchain file, which its preset selects. Included by the top-level CMakeLists.txt before the
# libraries are added, because it also sets the options they are built with.
#
# Library configuration:
#   DUET_LIBRARIES    the libraries to build
#   DUET_PART_DEFINE  macro naming the MCU part
#   COREN2G_SDHC, COREN2G_USB, RRFLIBRARIES_PRINTF_FLOAT, and the libraries' optimisation
#
# Firmware:
#   IAP_MCU             the DUET_MCU the board is built for
#   IAP_OUTPUT          image file name, without extension
#   IAP_DEFINES         board macro, and the macros that select how the IAP gets the firmware
#   IAP_LINKER_SCRIPT   linker script, under src/LinkerScripts
#   IAP_SBC             fetch the firmware from the SBC, which needs no SD card support (src/Libraries)
#   IAP_COREN2G_DEFINES macros that configure CoreN2G's USB device for the IAP
#   IAP_LINK_OPTIONS    extra options for the link

# The IAPs run without an RTOS from RAM, and are built for size
set(DUET_LIBRARIES RRFLibraries CoreN2G)
set(DUET_RTOS OFF)
set(COREN2G_SDHC ON)
set(RRFLIBRARIES_PRINTF_FLOAT OFF)
set(RRFLIBRARIES_OPTIMISATION -Os)
set(COREN2G_OPTIMISATION -Os)

set(IAP_SBC OFF)
set(IAP_COREN2G_DEFINES)
set(IAP_LINK_OPTIONS)

if(DUET_BOARD MATCHES "^Duet3_(MB6HC|MB6XD)_(SD|SBC)$")
    set(IAP_MCU SAME70)
    set(DUET_PART_DEFINE __SAME70Q20B__)
    set(IAP_OUTPUT Duet3_${CMAKE_MATCH_2}iap32_${CMAKE_MATCH_1})
    set(IAP_DEFINES DUET3_${CMAKE_MATCH_1})
    set(IAP_LINKER_SCRIPT same70_iap_ram.ld)
    set(IAP_LINK_OPTIONS LINKER:-u,_estack)
    if(DUET_BOARD MATCHES "_SBC$")
        set(IAP_COREN2G_DEFINES
            USB_DEVICE_VENDOR_ID=0x1D50
            USB_DEVICE_PRODUCT_ID=0x60EF
            "USB_DEVICE_PRODUCT_NAME=\"IAP\""
            UDI_CDC_RX_BUFFERS=2048
            USB_DEVICE_HS_SUPPORT)
        if(DUET_BOARD STREQUAL "Duet3_MB6XD_SBC")
            list(APPEND IAP_DEFINES "USB_DEVICE_PRODUCT_NAME=\"IAP\"")
        endif()
    endif()
elseif(DUET_BOARD MATCHES "^Duet3Mini_(SD|SBC)$")
    set(IAP_MCU SAME5x)
    set(DUET_PART_DEFINE __SAME54P20A__)
    set(IAP_OUTPUT Duet3_${CMAKE_MATCH_1}iap32_Mini5plus)
    set(IAP_DEFINES DUET3_MINI)
    set(IAP_LINKER_SCRIPT same54p20a_iap_ram.ld)
    if(DUET_BOARD STREQUAL "Duet3Mini_SBC")
        list(APPEND IAP_DEFINES IAP_USB_VID=0x1d50 IAP_USB_PID=0x60ef)
        set(IAP_COREN2G_DEFINES
            CONF_USB_CDCD_ACM_IDPRODUCT=0x60ef
            CONF_USB_CDCD_ACM_IPRODUCT_EN=1
            "CONF_USB_CDCD_ACM_IPRODUCT_STR=\"IAP\"")
        set(IAP_LINK_OPTIONS LINKER:-u,_estack)
    endif()
elseif(DUET_BOARD STREQUAL "FMDC_SD")
    set(IAP_MCU SAME5x)
    set(DUET_PART_DEFINE __SAME51N19A__)
    set(IAP_OUTPUT Duet3_SDiap32_FMDC)
    set(IAP_DEFINES FMDC)
    set(IAP_LINKER_SCRIPT same51n19a_iap_ram.ld)
else()
    message(FATAL_ERROR "Unknown DUET_BOARD '${DUET_BOARD}': configure with a board's preset (cmake --list-presets)")
endif()

list(APPEND IAP_DEFINES IAP_IN_RAM)
if(DUET_BOARD MATCHES "_SBC$")
    # The SBC IAPs talk to the SBC over SPI or USB
    set(IAP_SBC ON)
    set(COREN2G_USB ON)
    list(APPEND IAP_DEFINES IAP_SBC_SPI SUPPORT_USB=1 IAP_SBC_USB)
endif()

if(NOT DUET_MCU STREQUAL IAP_MCU)
    message(FATAL_ERROR "${DUET_BOARD} is a ${IAP_MCU} board, but this build tree is for ${DUET_MCU}: configure it with its preset")
endif()
