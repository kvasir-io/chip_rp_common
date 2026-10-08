cmake_minimum_required(VERSION 3.28)

# Host tests of the constexpr PIO assembler (pio/Asm.hpp, pio/AsmParse.hpp). Standalone, not part of any firmware tree:
#
# cmake -S tests -B tests/build -G Ninja && cmake --build tests/build && ctest --test-dir tests/build
#
# * asm_encoding_test.cpp: every instruction form by hand from the RP2350 datasheet 11.4, side-set and delay packing,
#   the builder against the parser, and the error messages.
# * the cross-check against pioasm, the oracle: every .pio file below the roots listed further down (the pico-sdk and
#   pico-examples checkouts, tests/pio) is assembled by pioasm (-o kvasir -v 0) AND by parse(), and gen_crosscheck.py
#   writes one file per .pio that static_asserts both agree. Building is the test; the test program prints what was
#   compared. A root that does not exist is left out with a note; a file pioasm refuses fails the build (pioasm's own
#   error), it is never dropped.
# * uart_stream_cursor_test.cpp: UartStream's RX cursor arithmetic driven by a simulated DMA channel.
# * bad/*.cpp: programs that must NOT compile, each with the message it must fail with (first line `// expect: ...`).
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

# pioasm, built for the host, with its pioasm_generate(): the checkout next to this one, else fetched, so a lone
# checkout of this repository (CI) builds the tests too. pioasm fetches cmake_helpers; a sibling checkout serves.
include(FetchContent)
if(NOT PIOASM_ROOT AND EXISTS "${_siblings}/pioasm/CMakeLists.txt")
    set(PIOASM_ROOT "${_siblings}/pioasm")
endif()
if(NOT FETCHCONTENT_SOURCE_DIR_CMAKE_HELPERS AND EXISTS "${_siblings}/cmake_helpers/BuildOptions.cmake")
    set(FETCHCONTENT_SOURCE_DIR_CMAKE_HELPERS "${_siblings}/cmake_helpers")
endif()
if(PIOASM_ROOT)
    add_subdirectory(${PIOASM_ROOT} pioasm)
else()
    FetchContent_Declare(
        pioasm
        GIT_REPOSITORY https://github.com/dominicpoeschko/pioasm.git
        GIT_TAG master
        GIT_SHALLOW TRUE)
    FetchContent_MakeAvailable(pioasm)
endif()
# clang 23's [[clang::lifetimebound]] suggestions (-Weverything) on pioasm's verbatim pico-sdk/bison sources: advice,
# not findings, and not ours to edit. The group takes the intra-/cross-TU (constructor) suggestions alike; clang 22 does
# not know it, hence -Wno-unknown-warning-option.
if(CMAKE_CXX_COMPILER_ID STREQUAL "Clang")
    target_compile_options(pioasm PRIVATE -Wno-unknown-warning-option -Wno-lifetime-safety-suggestions)
endif()

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
target_include_directories(pio_asm_tests PRIVATE ${_chip_rp_common} ${CMAKE_CURRENT_LIST_DIR}
                                                 ${CMAKE_CURRENT_BINARY_DIR}/crosscheck)
target_compile_options(pio_asm_tests PRIVATE ${_warnings})
add_test(NAME pio_asm_tests COMMAND pio_asm_tests)

# ---- UartStream's RX cursor (UARTStreamCursor.hpp) against a simulated DMA channel
add_executable(uart_stream_cursor_test uart_stream_cursor_test.cpp)
target_include_directories(uart_stream_cursor_test PRIVATE ${_chip_rp_common})
target_compile_options(uart_stream_cursor_test PRIVATE ${_warnings})
add_test(NAME uart_stream_cursor_test COMMAND uart_stream_cursor_test)

# ---- the cross-check against pioasm
# -----------------------------------------------------------------------------------
# pico-sdk and pico-examples: the checkouts in ~/repos, else fetched (shallow, no submodules, nothing of them is built)
# unless PIO_FETCH_CORPUS is OFF. Only read when PIO_CORPUS_ROOTS is not in the cache yet.
option(PIO_FETCH_CORPUS "fetch pico-sdk and pico-examples for the cross-check when ~/repos has no checkout" ON)
set(_corpus_defaults "pio=${CMAKE_CURRENT_LIST_DIR}/pio")
if(NOT DEFINED CACHE{PIO_CORPUS_ROOTS})
    foreach(_repo sdk:pico-sdk examples:pico-examples)
        string(REPLACE ":" ";" _repo ${_repo})
        list(GET _repo 0 _label)
        list(GET _repo 1 _name)
        if(EXISTS "$ENV{HOME}/repos/${_name}")
            list(APPEND _corpus_defaults "${_label}=$ENV{HOME}/repos/${_name}")
        elseif(PIO_FETCH_CORPUS)
            FetchContent_Declare(
                ${_name}
                GIT_REPOSITORY https://github.com/raspberrypi/${_name}.git
                GIT_TAG master
                GIT_SHALLOW TRUE
                GIT_SUBMODULES "" SOURCE_SUBDIR no_cmake_here)
            FetchContent_MakeAvailable(${_name})
            list(APPEND _corpus_defaults "${_label}=${${_name}_SOURCE_DIR}")
        endif()
    endforeach()
endif()
set(PIO_CORPUS_ROOTS
    ${_corpus_defaults}
    CACHE STRING "label=root pairs; every .pio below each root is cross-checked")

# Random programs (gen_random_pio.py): every instruction form, side-set shape, directive and operator in combinations
# nobody writes by hand, assembled by both pioasm and parse() like the rest of the corpus. Deterministic per seed; a new
# seed is a new set.
set(PIO_RANDOM_SEED
    2026
    CACHE STRING "seed of the random .pio programs")
set(PIO_RANDOM_FILES
    100
    CACHE STRING "how many random .pio files (1 to 3 programs each); 0 for none")
execute_process(
    COMMAND ${Python3_EXECUTABLE} -B ${CMAKE_CURRENT_LIST_DIR}/gen_random_pio.py ${CMAKE_CURRENT_BINARY_DIR}/random_pio
            ${PIO_RANDOM_SEED} ${PIO_RANDOM_FILES} RESULT_VARIABLE _rc)
if(NOT _rc EQUAL 0)
    message(FATAL_ERROR "gen_random_pio.py failed")
endif()
set_property(
    DIRECTORY
    APPEND
    PROPERTY CMAKE_CONFIGURE_DEPENDS ${CMAKE_CURRENT_LIST_DIR}/gen_random_pio.py)
list(APPEND PIO_CORPUS_ROOTS "random=${CMAKE_CURRENT_BINARY_DIR}/random_pio")

# One generated file per .pio, written at configure time; a .pio added under a root needs a re-configure.
execute_process(
    COMMAND ${Python3_EXECUTABLE} -B ${CMAKE_CURRENT_LIST_DIR}/gen_crosscheck.py ${CMAKE_CURRENT_BINARY_DIR}/crosscheck
            ${PIO_CORPUS_ROOTS}
    OUTPUT_VARIABLE _cases
    RESULT_VARIABLE _rc)
if(NOT _rc EQUAL 0)
    message(FATAL_ERROR "gen_crosscheck.py failed")
endif()
set_property(
    DIRECTORY
    APPEND
    PROPERTY CMAKE_CONFIGURE_DEPENDS ${CMAKE_CURRENT_LIST_DIR}/gen_crosscheck.py)

string(REPLACE "\n" ";" _cases "${_cases}")
set(_count 0)
foreach(_case IN LISTS _cases)
    if(_case STREQUAL "")
        continue()
    endif()
    string(REPLACE "\t" ";" _fields "${_case}")
    list(GET _fields 0 _id)
    list(GET _fields 1 _pio)
    set(_target xc_${_id})
    # -v 0: the version a program declares with .pio_version is the one it gets, as parse() does
    pioasm_generate(${_target} INPUT_FILE ${_pio} PIO_VERSION 0)
    target_sources(pio_asm_tests PRIVATE ${CMAKE_CURRENT_BINARY_DIR}/crosscheck/${_target}.cpp)
    target_link_libraries(pio_asm_tests PRIVATE ${_target})
    set_property(
        SOURCE ${CMAKE_CURRENT_BINARY_DIR}/crosscheck/${_target}.cpp
        APPEND
        PROPERTY OBJECT_DEPENDS ${_pio})
    math(EXPR _count "${_count} + 1")
endforeach()
message(STATUS "pio cross-check: ${_count} .pio files against pioasm")

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
    add_test(NAME ${_target} COMMAND ${CMAKE_COMMAND} --build ${CMAKE_BINARY_DIR} --target ${_target})
    # the build must fail, and for the reason named: a plain WILL_FAIL would pass a failure for any reason. Each case
    # runs a build of this very tree: two at once (ctest -j) corrupt its .ninja_deps.
    set_tests_properties(${_target} PROPERTIES PASS_REGULAR_EXPRESSION "${_expect}" RESOURCE_LOCK build_tree)
endforeach()

# ---- the same sources with gcc: gcc firmware trees build the chip headers too ----------------------------------------
get_target_property(_sources pio_asm_tests SOURCES)
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
    add_test(
        NAME gcc_${_id}
        COMMAND
            ${Python3_EXECUTABLE} -B ${CMAKE_CURRENT_LIST_DIR}/gcc_check.py ${_gcc_path_${_gcc}} ${_chip_rp_common}
            ${CMAKE_CURRENT_LIST_DIR} ${CMAKE_CURRENT_BINARY_DIR}/crosscheck
            ${CMAKE_CURRENT_BINARY_DIR}/pioasm_generated -- ${_abs_sources})
    # the cross-check files include pioasm's headers, which the build makes
    set_tests_properties(gcc_${_id} PROPERTIES DEPENDS pio_asm_tests)
endforeach()
