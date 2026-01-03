add_subdirectory(cmake/stm32cubemx)

add_library(finemote_board INTERFACE)
add_library(finemote::board ALIAS finemote_board)

target_link_libraries(finemote_board INTERFACE
        stm32cubemx
)

# Macro definitions for board

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

target_link_options(finemote_board INTERFACE
        "--scatter=${CMAKE_CURRENT_LIST_DIR}/MDK-ARM/${BOARD_NAME}/${BOARD_NAME}.sct"
        "--cpu=${FINEMOTE_ARMLINK_CPU}"
        "--strict"
)

target_include_directories(finemote_board INTERFACE
        ${CMAKE_CURRENT_LIST_DIR}
        ${CMAKE_CURRENT_LIST_DIR}/FineMote_BSP
)

add_library(finemote_bsp OBJECT)
add_library(finemote::bsp ALIAS finemote_bsp)

target_link_libraries(finemote_bsp PRIVATE
        finemote::core
        finemote::board
)

target_sources(finemote_bsp PRIVATE
        ${CMAKE_CURRENT_LIST_DIR}/MC_Board.cpp
        ${CMAKE_CURRENT_LIST_DIR}/Semihost.c
        ${CMAKE_CURRENT_LIST_DIR}/FineMote_BSP/BSP_CAN.cpp
        ${CMAKE_CURRENT_LIST_DIR}/FineMote_BSP/BSP_UART.cpp
)

target_compile_options(finemote_bsp PRIVATE
        "-fno-rtti"
        "-fno-exceptions"
)