cmake_minimum_required(VERSION 3.22)

set(CMSISCORE "${PROJECT_SOURCE_DIR}/BSP/${BOARD_NAME}/Drivers/CMSIS")
add_subdirectory(CMSIS-DSP)

target_link_libraries(CMSISDSP PRIVATE finemote::board)

add_subdirectory(etl)