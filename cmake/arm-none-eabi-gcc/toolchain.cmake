set(CMAKE_C_COMPILER arm-none-eabi-gcc)
set(CMAKE_ASM_COMPILER arm-none-eabi-gcc)
set(CMAKE_CXX_COMPILER arm-none-eabi-g++)

set(CMAKE_LINKER arm-none-eabi-gcc)
set(CMAKE_AR arm-none-eabi-ar)
set(CMAKE_OBJCOPY arm-none-eabi-objcopy)
set(CMAKE_SIZE arm-none-eabi-size)

set(_FINEMOTE_TOOLCHAIN_ID arm-none-eabi-gcc CACHE INTERNAL "FineMote toolchain id")

add_compile_options(
        "$<$<CONFIG:Release>:-Os;-g3>"
        "$<$<CONFIG:Debug>:-Og;-g3>"
)

set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)

set(CMAKE_EXECUTABLE_SUFFIX_ASM ".elf")
set(CMAKE_EXECUTABLE_SUFFIX_C ".elf")
set(CMAKE_EXECUTABLE_SUFFIX_CXX ".elf")

get_filename_component(_FINEMOTE_PROJECT_DIR "${CMAKE_CURRENT_LIST_DIR}/../.." ABSOLUTE)

include("${CMAKE_CURRENT_LIST_DIR}/options.cmake")

set(CMAKE_TRY_COMPILE_PLATFORM_VARIABLES BOARD_NAME)
if (NOT DEFINED BOARD_NAME OR BOARD_NAME STREQUAL "")
    message(FATAL_ERROR "BOARD_NAME is not set.")
endif ()

set(_FINEMOTE_BOARD_TOOLCHAIN "${_FINEMOTE_PROJECT_DIR}/BSP/${BOARD_NAME}/cmake/board_toolchain.cmake")
if (NOT EXISTS "${_FINEMOTE_BOARD_TOOLCHAIN}")
    message(FATAL_ERROR "Board toolchain file not found: ${_FINEMOTE_BOARD_TOOLCHAIN}")
endif ()

include("${_FINEMOTE_BOARD_TOOLCHAIN}")
message(STATUS "Configured for arm-none-eabi-gcc toolchain targeting ${BOARD_NAME}")
