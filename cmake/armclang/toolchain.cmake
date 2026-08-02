set(CMAKE_C_COMPILER armclang)
set(CMAKE_ASM_COMPILER armclang)
set(CMAKE_CXX_COMPILER armclang)

set(CMAKE_LINKER armlink)
set(CMAKE_AR armar)
set(CMAKE_OBJCOPY fromelf)
set(CMAKE_SIZE fromelf)

set(_FINEMOTE_TOOLCHAIN_ID armclang CACHE INTERNAL "FineMote toolchain id")
set(TRIPLE arm-arm-none-eabi)

#set(CMAKE_C_COMPILER_TARGET ${TRIPLE})
#set(CMAKE_CXX_COMPILER_TARGET ${TRIPLE})
#set(CMAKE_ASM_COMPILER_TARGET ${TRIPLE})

## for clion to detect the compiler info
set(CMAKE_C_FLAGS_INIT "--target=${TRIPLE}")
set(CMAKE_CXX_FLAGS_INIT "--target=${TRIPLE}")
set(CMAKE_ASM_FLAGS_INIT "--target=${TRIPLE}")

add_compile_options(
        "$<$<CONFIG:Release>:-Os;-gdwarf-4>"
        "$<$<CONFIG:Debug>:-O1;-gdwarf-4>"
        "$<$<COMPILE_LANGUAGE:C,CXX>:-Wall;-Wextra;-Wpedantic>"
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
message(STATUS "Configured for armclang toolchain targeting ${BOARD_NAME}")
