cmake_minimum_required(VERSION 3.22)

set(CMSISCORE "${PROJECT_SOURCE_DIR}/BSP/${BOARD_NAME}/Drivers/CMSIS")
add_subdirectory(CMSIS-DSP)

target_link_libraries(CMSISDSP PRIVATE finemote::board)

add_subdirectory(etl)

# FreeRTOS-POSIX
add_library(FreeRTOS-POSIX STATIC)

target_include_directories(FreeRTOS-POSIX PUBLIC
        "${CMAKE_CURRENT_SOURCE_DIR}/FreeRTOS-POSIX/include"
        "${CMAKE_CURRENT_SOURCE_DIR}/FreeRTOS-POSIX/include/private"
        "${CMAKE_CURRENT_SOURCE_DIR}/FreeRTOS-POSIX/FreeRTOS-Plus-POSIX/include"
        "${CMAKE_CURRENT_SOURCE_DIR}/FreeRTOS-POSIX/FreeRTOS-Plus-POSIX/include/portable"
)

file(GLOB FreeRTOS_POSIX_SOURCES CONFIGURE_DEPENDS
        "${CMAKE_CURRENT_SOURCE_DIR}/FreeRTOS-POSIX/FreeRTOS-Plus-POSIX/source/*.c"
)

target_sources(FreeRTOS-POSIX PRIVATE ${FreeRTOS_POSIX_SOURCES})

target_link_libraries(FreeRTOS-POSIX PRIVATE finemote::board)