# DuetIAP: the in-application programmers that write new RepRapFirmware to a main board's flash,
# built as part of DuetFirmware against the shared libraries in lib/. A build tree builds one
# board, DUET_BOARD, as an executable target of the same name; Boards.cmake describes the boards.
# Its image goes to the top of the build tree: the .elf, the .bin, and the .map.

cmake_minimum_required(VERSION 3.25)
project(DuetIAP LANGUAGES C CXX)

# The C++ sources come before the C ones
file(GLOB_RECURSE _cxx CONFIGURE_DEPENDS src/*.cpp)
file(GLOB_RECURSE _c   CONFIGURE_DEPENDS src/*.c)
if(IAP_SBC)
    # The SD card support
    list(FILTER _cxx EXCLUDE REGEX "/src/Libraries/")
    list(FILTER _c   EXCLUDE REGEX "/src/Libraries/")
endif()

duet_firmware_executable(${DUET_BOARD}
    OUTPUT_NAME ${IAP_OUTPUT}
    LINKER_SCRIPT "${CMAKE_CURRENT_SOURCE_DIR}/src/LinkerScripts/${IAP_LINKER_SCRIPT}"
    SOURCES ${_cxx} ${_c})
# The IAP links libsupc++ for the C++ runtime, so it is linked by the C driver rather than g++,
# which would add libstdc++
set_target_properties(${DUET_BOARD} PROPERTIES LINKER_LANGUAGE C)

target_include_directories(${DUET_BOARD} PRIVATE src src/Libraries/Fatfs)

target_compile_definitions(${DUET_BOARD} PRIVATE
    ${DUET_PART_DEFINE}
    ${IAP_DEFINES}
    $<$<COMPILE_LANGUAGE:C>:noexcept=>)

target_compile_options(${DUET_BOARD} PRIVATE
    $<$<CONFIG:Release>:-Os>
    -fsingle-precision-constant
    -Wundef
    -Wdouble-promotion
    $<$<COMPILE_LANGUAGE:CXX>:-fno-threadsafe-statics;-fno-rtti;-fno-exceptions>)

target_link_options(${DUET_BOARD} PRIVATE
    --specs=nano.specs
    LINKER:--check-sections
    LINKER:--unresolved-symbols=report-all
    LINKER:--warn-common
    LINKER:--warn-section-align
    LINKER:--warn-unresolved-symbols
    ${IAP_LINK_OPTIONS})

target_link_libraries(${DUET_BOARD} PRIVATE
    "$<LINK_GROUP:RESCAN,Duet::CoreN2G,Duet::RRFLibraries,supc++>"
    m)

if(IAP_COREN2G_DEFINES)
    target_compile_definitions(CoreN2G PRIVATE ${IAP_COREN2G_DEFINES})
endif()

duet_firmware_images(${DUET_BOARD})
