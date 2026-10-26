cmake_minimum_required(VERSION 3.28)

# Host tests of the chip_rp_common parts that run without a chip. Standalone, not part of any firmware tree:
#
# cmake -S tests -B tests/build -G Ninja && cmake --build tests/build && ctest --test-dir tests/build
#
# * asm_encoding_test.cpp: the constexpr PIO assembler (pio/Asm.hpp): every instruction form by hand from the RP2350
#   datasheet 11.4, side-set and delay packing, labels, whole programs, the chip package's programs and the error
#   messages. Building is the test (static_assert).
# * uart_stream_cursor_test.cpp: UartStream's RX cursor arithmetic driven by a simulated DMA channel.
# * pwm_math_test.cpp: the PWM divider/TOP search (PwmMath.hpp), in constant and in run-time evaluation.
# * adc_math_test.cpp: the ADC pacer divider and the channel-attribution check (AdcMath.hpp).
# * reset_cause_test.cpp: the reset-cause decision (ResetCauseRules.hpp): a SYSRESETREQ after a RUN-pin reset is not
#   "RUN".
# * pin_function_test.cpp: the pin-function formulas and the drivers' pin check (PinConfig.hpp functionSelect,
#   isValidPin, the refusal's text) against RP2040 Table 2 / RP2350 Table 645.
# * register_alias_test.cpp, pll_search_test.cpp: the RP alias backend on the Kvasir SDK's register mock, the PLL search
#   against an exhaustive exact search (the SDK's headers and test support: the sibling checkout, else fetched).
# * dma_crc_hold_test.cpp: the DMA CRC sniffer's debug check (DmaCrcHold.hpp) at debug-check levels 2 and 0.
# * i2c_math_test.cpp, watchdog_math_test.cpp, flash_timing_math_test.cpp: the I2C SCL counts (I2CMath.hpp), the
#   watchdog tick and LOAD incl. RP2040-E1 (WatchdogMath.hpp), the QMI flash timing (FlashTimingMath.hpp) against the
#   datasheets' rules.
# * bad/*.cpp: programs (and PWM configs, refused pins) that must NOT compile, each with the message it must fail with
#   (first line `// expect: ...`).
#
# clang++ unless the caller names a compiler (CXX / -DCMAKE_CXX_COMPILER): the firmware's compiler.
if(NOT CMAKE_CXX_COMPILER AND NOT DEFINED ENV{CXX})
    find_program(_clangxx clang++)
    if(_clangxx)
        set(CMAKE_CXX_COMPILER "${_clangxx}")
    endif()
endif()
project(chip_rp_common_pio_asm_tests LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 26)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

enable_testing()
find_package(Python3 REQUIRED COMPONENTS Interpreter)

get_filename_component(_chip_rp_common "${CMAKE_CURRENT_LIST_DIR}/.." ABSOLUTE)
get_filename_component(_siblings "${_chip_rp_common}/.." ABSOLUTE)

include(FetchContent)

set(_warnings -Wall -Wextra -Wpedantic -Werror)
# clang: the firmware trees' set (cmake_helpers, as in their compile_commands.json), so a warning the chip headers would
# give in a firmware build shows here first
if(CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    list(
        APPEND
        _warnings
        -Weverything
        -Wno-switch-default
        -Wno-c++98-compat
        -Wno-c++98-compat-pedantic
        -Wno-c++20-compat
        -Wno-pre-c2x-compat
        -Wno-pre-c11-compat
        -Wno-padded
        -Wno-covered-switch-default
        -Wno-switch-enum
        -Wno-class-varargs
        -Wno-gnu-zero-variadic-macro-arguments
        -Wno-float-equal
        -Wno-unknown-warning-option
        -Wno-reserved-identifier
        -Wno-zero-as-null-pointer-constant
        -Wno-documentation-unknown-command
        -Wno-documentation
        -Wno-declaration-after-statement
        -Wno-braced-scalar-init
        -Wno-unsafe-buffer-usage
        -Wno-nrvo
        -Wno-unused-macros
        # clang 23: the firmware trees take these out too (Kvasir_SDK arm_clang.cmake)
        -Wno-lifetime-safety-intra-tu-suggestions
        -Wno-lifetime-safety-intra-tu-constructor-suggestions)
endif()

# ---- the encoding test and the test program
# ---------------------------------------------------------------------------
add_executable(pio_asm_tests asm_encoding_test.cpp)
target_include_directories(pio_asm_tests PRIVATE ${_chip_rp_common})
target_compile_options(pio_asm_tests PRIVATE ${_warnings})
add_test(NAME pio_asm_tests COMMAND pio_asm_tests)

# ---- UartStream's RX cursor (UARTStreamCursor.hpp) against a simulated DMA channel
add_executable(uart_stream_cursor_test uart_stream_cursor_test.cpp)
target_include_directories(uart_stream_cursor_test PRIVATE ${_chip_rp_common})
target_compile_options(uart_stream_cursor_test PRIVATE ${_warnings})
add_test(NAME uart_stream_cursor_test COMMAND uart_stream_cursor_test)

# ---- the PWM divider/TOP search (PwmMath.hpp): values, refusal reasons, constant and run-time evaluation alike
add_executable(pwm_math_test pwm_math_test.cpp)
target_include_directories(pwm_math_test PRIVATE ${_chip_rp_common})
target_compile_options(pwm_math_test PRIVATE ${_warnings})
add_test(NAME pwm_math_test COMMAND pwm_math_test)

# ---- the ADC pacer divider (AdcMath.hpp): no paced period of 96 clocks, the rounding at the top rate.
add_executable(adc_math_test adc_math_test.cpp)
target_include_directories(adc_math_test PRIVATE ${_chip_rp_common})
target_compile_options(adc_math_test PRIVATE ${_warnings})
add_test(NAME adc_math_test COMMAND adc_math_test)

# ---- the reset-cause decision (ResetCauseRules.hpp) of both chips, the chip-reset mark
add_executable(reset_cause_test reset_cause_test.cpp)
target_include_directories(reset_cause_test PRIVATE ${_chip_rp_common})
target_compile_options(reset_cause_test PRIVATE ${_warnings})
add_test(NAME reset_cause_test COMMAND reset_cause_test)

# ---- the pin-function formulas (PinConfig.hpp) against the datasheets' tables, for all three variants: building is the
# test (static_assert). PinConfig.hpp wants a chip identity; the test names the variants itself.
add_executable(pin_function_test pin_function_test.cpp)
target_include_directories(pin_function_test PRIVATE ${_chip_rp_common})
target_compile_definitions(pin_function_test PRIVATE KVASIR_CHIP_RP2350 KVASIR_CHIP_RP2350A)
target_compile_options(pin_function_test PRIVATE ${_warnings})
add_test(NAME pin_function_test COMMAND pin_function_test)

# ---- the RP parts that run on the Kvasir SDK's register mock and Prescaler: the alias backend (RegisterAlias.hpp) and
# the PLL search (PllSearch.hpp) against the one it replaced. The SDK: the checkout next to this one, else fetched
# (headers and tests/ only: no submodules), so a lone checkout of this repository (CI) builds them too. The fetch takes
# the SDK's master: an SDK change these tests need reaches it first.
if(NOT KVASIR_ROOT AND EXISTS "${_chip_rp_common}/../Kvasir_SDK/cmake/kvasir_locate.cmake")
    get_filename_component(KVASIR_ROOT "${_chip_rp_common}/../Kvasir_SDK" ABSOLUTE)
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
include(${KVASIR_ROOT}/cmake/kvasir_sdk_targets.cmake) # kvasir::base, kvasir::test_support

# the mock's Recorder is a global by design (the SDK's tests/kvasir_test.hpp)
set(_sdk_test_warnings ${_warnings} $<$<CXX_COMPILER_ID:Clang>:-Wno-global-constructors -Wno-exit-time-destructors>)

add_executable(register_alias_test register_alias_test.cpp)
target_link_libraries(register_alias_test PRIVATE kvasir::test_support)
# the seam as an image gets it (chip_rp20x0 cmake/chip.cmake): Register/Exec.hpp includes RegisterAlias.hpp at its end
target_compile_definitions(register_alias_test PRIVATE KVASIR_REGISTER_MOCK
                                                       [[KVASIR_CHIP_REGISTER_SEAM="RegisterAlias.hpp"]])
target_include_directories(register_alias_test PRIVATE ${_chip_rp_common})
target_compile_options(register_alias_test PRIVATE ${_sdk_test_warnings})
add_test(NAME register_alias_test COMMAND register_alias_test)

add_executable(pll_search_test pll_search_test.cpp)
target_link_libraries(pll_search_test PRIVATE kvasir::test_support) # test_harness.hpp
target_include_directories(pll_search_test PRIVATE ${_chip_rp_common})
target_compile_options(pll_search_test PRIVATE ${_sdk_test_warnings})
add_test(NAME pll_search_test COMMAND pll_search_test)

# the RP constexpr maths, register-free: the I2C SCL counts (I2CMath.hpp, with the SDK's Prescaler), the watchdog tick
# and LOAD incl. RP2040-E1 (WatchdogMath.hpp), the QMI flash timing (FlashTimingMath.hpp); refusals in bad/
add_executable(i2c_math_test i2c_math_test.cpp)
target_link_libraries(i2c_math_test PRIVATE kvasir::base)
target_include_directories(i2c_math_test PRIVATE ${_chip_rp_common})
target_compile_options(i2c_math_test PRIVATE ${_warnings})
add_test(NAME i2c_math_test COMMAND i2c_math_test)
foreach(_t watchdog_math_test flash_timing_math_test)
    add_executable(${_t} ${_t}.cpp)
    target_include_directories(${_t} PRIVATE ${_chip_rp_common})
    target_compile_options(${_t} PRIVATE ${_warnings})
    add_test(NAME ${_t} COMMAND ${_t})
endforeach()

# the DMA CRC sniffer's debug check (DmaCrcHold.hpp), at the debug variants' level and with debug checks off
foreach(_level 2 0)
    add_executable(dma_crc_hold_test_${_level} dma_crc_hold_test.cpp)
    target_link_libraries(dma_crc_hold_test_${_level} PRIVATE kvasir::test_support) # harness/PanicCatcher.hpp
    target_compile_definitions(dma_crc_hold_test_${_level} PRIVATE KVASIR_DCHECK_LEVEL=${_level})
    target_include_directories(dma_crc_hold_test_${_level} PRIVATE ${_chip_rp_common})
    target_compile_options(dma_crc_hold_test_${_level} PRIVATE ${_sdk_test_warnings})
    add_test(NAME dma_crc_hold_test_${_level} COMMAND dma_crc_hold_test_${_level})
endforeach()

# ---- programs that must not compile
# -----------------------------------------------------------------------------------
file(GLOB _bad CONFIGURE_DEPENDS ${CMAKE_CURRENT_LIST_DIR}/bad/*.cpp)
foreach(_file IN LISTS _bad)
    get_filename_component(_name ${_file} NAME_WE)
    # the expected message is read here: a change to it has to re-run cmake
    set_property(
        DIRECTORY
        APPEND
        PROPERTY CMAKE_CONFIGURE_DEPENDS ${_file})
    file(
        STRINGS ${_file} _expect
        REGEX "^// expect: "
        LIMIT_COUNT 1)
    string(REPLACE "// expect: " "" _expect "${_expect}")
    if(_expect STREQUAL "")
        message(FATAL_ERROR "${_file} has no '// expect: <regex>' line")
    endif()
    set(_target bad_${_name})
    add_library(${_target} OBJECT EXCLUDE_FROM_ALL ${_file})
    target_include_directories(${_target} PRIVATE ${_chip_rp_common})
    target_link_libraries(${_target} PRIVATE kvasir::base) # I2CMath.hpp: the SDK's Prescaler
    # PinConfig.hpp wants a chip (the pin cases name their part themselves)
    target_compile_definitions(${_target} PRIVATE KVASIR_CHIP_RP2350 KVASIR_CHIP_RP2350A)
    add_test(NAME ${_target} COMMAND ${CMAKE_COMMAND} --build ${CMAKE_BINARY_DIR} --target ${_target})
    # the build must fail, and for the reason named: a plain WILL_FAIL would pass a failure for any reason. Each case
    # runs a build of this very tree: two at once (ctest -j) corrupt its .ninja_deps.
    set_tests_properties(${_target} PROPERTIES PASS_REGULAR_EXPRESSION "${_expect}" RESOURCE_LOCK build_tree)
endforeach()

# ---- the same sources with gcc: gcc firmware trees build the chip headers too ----------------------------------------
get_target_property(_sources pio_asm_tests SOURCES)
list(
    APPEND
    _sources
    pwm_math_test.cpp
    adc_math_test.cpp
    reset_cause_test.cpp
    watchdog_math_test.cpp
    flash_timing_math_test.cpp)
set(_abs_sources)
foreach(_s IN LISTS _sources)
    get_filename_component(_s ${_s} ABSOLUTE BASE_DIR ${CMAKE_CURRENT_LIST_DIR})
    list(APPEND _abs_sources ${_s})
endforeach()
foreach(_gcc g++ arm-none-eabi-g++)
    find_program(_gcc_path_${_gcc} ${_gcc})
    if(NOT _gcc_path_${_gcc})
        message(STATUS "pio gcc check: ${_gcc} not found, left out")
        continue()
    endif()
    string(MAKE_C_IDENTIFIER ${_gcc} _id)
    add_test(NAME gcc_${_id} COMMAND ${Python3_EXECUTABLE} -B ${CMAKE_CURRENT_LIST_DIR}/gcc_check.py
                                     ${_gcc_path_${_gcc}} ${_chip_rp_common} -- ${_abs_sources})
endforeach()
