add_subdirectory(cmake/stm32cubemx)

add_library(finemote_board INTERFACE)
add_library(finemote::board ALIAS finemote_board)

target_link_libraries(finemote_board INTERFACE
        stm32cubemx
)

# Macro definitions for board

# For etl
target_compile_definitions(finemote_board INTERFACE
        ETL_TARGET_OS_CMSIS_OS2
)

# For cmsis dsp
target_compile_definitions(finemote_board INTERFACE
        ARM_MATH_CM4
        ARM_MATH_MATRIX_CHECK
        ARM_MATH_ROUNDING
)

string(TOUPPER ${BOARD_NAME} UPPER_BOARD_NAME)
target_compile_definitions(finemote_board INTERFACE
        __${UPPER_BOARD_NAME}
        __CMAKE_APP
)

target_include_directories(finemote_board INTERFACE
        ${CMAKE_CURRENT_SOURCE_DIR}
        ${CMAKE_CURRENT_SOURCE_DIR}/FineMote_BSP
        ${CMAKE_CURRENT_SOURCE_DIR}/Middlewares/Third_Party/FreeRTOS_POSIX
)

add_library(finemote_bsp OBJECT)
add_library(finemote::bsp ALIAS finemote_bsp)

target_link_libraries(finemote_bsp PRIVATE
        finemote::core
        finemote::board
)

file(GLOB _FINEMOTE_BSP_SOURCES CONFIGURE_DEPENDS
        "${CMAKE_CURRENT_SOURCE_DIR}/*.c"
        "${CMAKE_CURRENT_SOURCE_DIR}/*.cpp"
        "${CMAKE_CURRENT_SOURCE_DIR}/FineMote_BSP/*.c"
        "${CMAKE_CURRENT_SOURCE_DIR}/FineMote_BSP/*.cpp"
)

if (_FINEMOTE_TOOLCHAIN_ID STREQUAL "arm-none-eabi-gcc")
    list(FILTER _FINEMOTE_BSP_SOURCES EXCLUDE REGEX ".*/Semihost\\.c$")
endif ()

target_sources(finemote_bsp PRIVATE
        ${_FINEMOTE_BSP_SOURCES}
)

target_compile_options(finemote_bsp PRIVATE
        "-fno-rtti"
        "-fno-exceptions"
)
