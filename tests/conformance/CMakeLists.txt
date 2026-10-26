cmake_minimum_required(VERSION 3.28)

# The RP chips' conformance tree (Kvasir_SDK tests/conformance/conformance.cmake), one for chip_rp2040 and chip_rp2350:
# the chip package's headers and these shared ones compiled for the target with a firmware's flags, against the
# generated register headers. Building is the test - the contract rows (conformance.cpp), the dependent register writes
# (instantiate.cpp) and every public header alone. The chip and core SVDs convert with the write-only guard on the way.
# The two chips differ only where the sources say `#ifdef KVASIR_CHIP_RP2040` / `KVASIR_CHIP_RP2350` (the package's
# identity defines) and in not_alone.txt's chip column.
#
# cmake -S tests/conformance -B tests/conformance/build/<chip> -G Ninja -DCONFORMANCE_CHIP=rp2040|rp2350
# [-DCONFORMANCE_VARIANT=RP2350B] [-DCOMPILE_TARGET=arm_gcc] && cmake --build tests/conformance/build/<chip>
#
# The chip (CHIP_ROOT): the chip checkout this one is the src/chip/rp_common submodule of (a chip repository's CI:
# `cmake -S src/chip/rp_common/tests/conformance`; CONFORMANCE_CHIP then defaults to that chip), else chip_<chip> next
# to this checkout, else fetched with its core submodule and THIS checkout put in as its src/chip/rp_common. Kvasir_SDK
# and kvasir_devices: the checkouts next to the chip, else fetched (FETCHCONTENT_SOURCE_DIR_KVASIR_SDK / _KVASIR_DEVICES
# / _CHIP_RP2040 / _CHIP_RP2350 point a lone checkout at local copies).

get_filename_component(_rp_common "${CMAKE_CURRENT_LIST_DIR}/../.." ABSOLUTE)
get_filename_component(_enclosing_chip "${_rp_common}/../../.." ABSOLUTE)
set(_in_chip OFF)
if(EXISTS ${_enclosing_chip}/cmake/chip.cmake AND EXISTS ${_enclosing_chip}/src/chip/rp_common/tests/conformance)
    set(_in_chip ON)
    if(EXISTS ${_enclosing_chip}/src/chip/rp2040.hpp)
        set(_chip_default rp2040)
    elseif(EXISTS ${_enclosing_chip}/src/chip/rp2350.hpp)
        set(_chip_default rp2350)
    endif()
endif()
if(NOT _chip_default)
    set(_chip_default rp2350)
endif()
set(CONFORMANCE_CHIP
    ${_chip_default}
    CACHE STRING "the RP chip the tree is built for: rp2040 or rp2350")
set_property(CACHE CONFORMANCE_CHIP PROPERTY STRINGS rp2040 rp2350)
if(NOT CONFORMANCE_CHIP MATCHES "^(rp2040|rp2350)$")
    message(FATAL_ERROR "CONFORMANCE_CHIP is '${CONFORMANCE_CHIP}'; this tree knows rp2040 and rp2350")
endif()

if(CONFORMANCE_CHIP STREQUAL "rp2350")
    set(CONFORMANCE_VARIANT
        RP2350A
        CACHE STRING "the chip variant (Chip::Part) the TUs are compiled for: RP2350A or RP2350B")
    # the package states the part (cmake/chip.cmake KVASIR_RP2350_MPU -> KVASIR_CHIP_RP2350A/B -> Chip::part)
    set(KVASIR_RP2350_MPU
        ${CONFORMANCE_VARIANT}
        CACHE STRING "set from CONFORMANCE_VARIANT" FORCE)
elseif(CONFORMANCE_VARIANT)
    message(FATAL_ERROR "CONFORMANCE_VARIANT '${CONFORMANCE_VARIANT}': the RP2040 has one part")
endif()

include(FetchContent)
if(NOT CHIP_ROOT)
    get_filename_component(_sibling_chip "${_rp_common}/../chip_${CONFORMANCE_CHIP}" ABSOLUTE)
    if(_in_chip AND EXISTS ${_enclosing_chip}/src/chip/${CONFORMANCE_CHIP}.hpp)
        set(CHIP_ROOT ${_enclosing_chip})
    elseif(EXISTS ${_sibling_chip}/cmake/chip.cmake)
        set(CHIP_ROOT ${_sibling_chip})
    else()
        FetchContent_Declare(
            chip_${CONFORMANCE_CHIP}
            GIT_REPOSITORY https://github.com/kvasir-io/chip_${CONFORMANCE_CHIP}.git
            GIT_TAG master
            GIT_SHALLOW TRUE
            GIT_SUBMODULES "core" SOURCE_SUBDIR none)
        string(TOUPPER "chip_${CONFORMANCE_CHIP}" _chip_uc)
        set(_given ${FETCHCONTENT_SOURCE_DIR_${_chip_uc}})
        FetchContent_MakeAvailable(chip_${CONFORMANCE_CHIP})
        set(CHIP_ROOT ${chip_${CONFORMANCE_CHIP}_SOURCE_DIR})
        get_filename_component(_chip_rp_common "${CHIP_ROOT}/src/chip/rp_common" REALPATH)
        get_filename_component(_this_rp_common "${_rp_common}" REALPATH)
        if(_given)
            # a local copy is used as it is: never changed from here
            if(NOT _chip_rp_common STREQUAL _this_rp_common)
                message(WARNING "${_given}/src/chip/rp_common is not this checkout: that copy is tested")
            endif()
        elseif(NOT _chip_rp_common STREQUAL _this_rp_common)
            # the fetched chip's rp_common is the checkout under test, not the chip's pinned submodule (the fetch leaves
            # that path empty: GIT_SUBMODULES "core")
            file(REMOVE_RECURSE ${CHIP_ROOT}/src/chip/rp_common)
            file(CREATE_LINK ${_rp_common} ${CHIP_ROOT}/src/chip/rp_common SYMBOLIC)
        endif()
    endif()
endif()

# Kvasir_SDK: KVASIR_ROOT, else the checkout next to this repository, else fetched; its submodules and the other
# packages: kvasir_locate (Kvasir_SDK cmake/kvasir_locate.cmake)
if(NOT KVASIR_ROOT AND EXISTS "${CHIP_ROOT}/../Kvasir_SDK/cmake/kvasir_locate.cmake")
    get_filename_component(KVASIR_ROOT "${CHIP_ROOT}/../Kvasir_SDK" ABSOLUTE)
elseif(NOT KVASIR_ROOT)
    include(FetchContent)
    FetchContent_Declare(
        kvasir_sdk
        GIT_REPOSITORY https://github.com/kvasir-io/Kvasir_SDK.git
        GIT_TAG master
        GIT_SHALLOW TRUE
        GIT_SUBMODULES "" SOURCE_SUBDIR fetch-only)
    FetchContent_MakeAvailable(kvasir_sdk)
    set(KVASIR_ROOT "${kvasir_sdk_SOURCE_DIR}")
endif()
include("${KVASIR_ROOT}/cmake/kvasir_locate.cmake")
kvasir_locate(SDK_SUBMODULES ALL DEVICES)
if(NOT KVASIR_DEVICES_ROOT)
    FetchContent_Declare(
        kvasir_devices
        GIT_REPOSITORY https://github.com/kvasir-io/kvasir_devices.git
        GIT_TAG master
        GIT_SHALLOW TRUE
        SOURCE_SUBDIR none)
    FetchContent_MakeAvailable(kvasir_devices)
    set(KVASIR_DEVICES_ROOT ${kvasir_devices_SOURCE_DIR})
endif()

include(${KVASIR_ROOT}/tests/conformance/conformance.cmake)
kvasir_conformance_begin()

# a version: main_tu.cpp lists Kvasir::Boot::Report<>, whose boot line names the project's
project(
    chip_${CONFORMANCE_CHIP}_conformance
    VERSION 0.0.0
    LANGUAGES CXX C ASM)

include(${KVASIR_ROOT}/cmake/util.cmake)

# not_alone.txt: `<header> <chips> <reason>` - the lines of this chip, in the SDK's `<header> <reason>` form
file(STRINGS ${CMAKE_CURRENT_LIST_DIR}/not_alone.txt _not_alone_lines)
set(_not_alone "# generated from ${CMAKE_CURRENT_LIST_DIR}/not_alone.txt for ${CONFORMANCE_CHIP}\n")
foreach(_l IN LISTS _not_alone_lines)
    if(_l MATCHES "^([^# \t]+)[ \t]+([a-z0-9,]+)[ \t]+(.*)$")
        string(REPLACE "," ";" _chips "${CMAKE_MATCH_2}")
        if(CONFORMANCE_CHIP IN_LIST _chips)
            string(APPEND _not_alone "${CMAKE_MATCH_1} ${CMAKE_MATCH_3}\n")
        endif()
    elseif(NOT _l MATCHES "^[ \t]*(#.*)?$")
        message(FATAL_ERROR "not_alone.txt: '${_l}' is not '<header> <chips> <reason>'")
    endif()
endforeach()
# written only when it changes (file(CONFIGURE)): the SDK reads it at configure time and re-configures on its mtime
file(CONFIGURE OUTPUT ${CMAKE_CURRENT_BINARY_DIR}/not_alone.txt CONTENT "${_not_alone}" @ONLY)
set_property(
    DIRECTORY
    APPEND
    PROPERTY CMAKE_CONFIGURE_DEPENDS ${CMAKE_CURRENT_LIST_DIR}/not_alone.txt)

# the RP2040's atomic shim takes the chip's cross-core lock (main_tu.cpp)
set(_main_tu_definitions)
if(CONFORMANCE_CHIP STREQUAL "rp2040")
    set(_main_tu_definitions MAIN_TU_DEFINITIONS KVASIR_MULTICORE=1)
endif()

kvasir_conformance_project(
    NAME
    chip_${CONFORMANCE_CHIP}_conformance
    SOURCES
    conformance.cpp
    INSTANTIATE
    instantiate.cpp
    CONFIG_HEADER
    kvasir_config.hpp
    HEADERS
    ${CHIP_ROOT}/src/chip/*.hpp
    ${CHIP_ROOT}/src/chip/rp_common/*.hpp
    NOT_ALONE
    ${CMAKE_CURRENT_BINARY_DIR}/not_alone.txt
    MAIN_TU
    main_tu.cpp
    ${_main_tu_definitions})
