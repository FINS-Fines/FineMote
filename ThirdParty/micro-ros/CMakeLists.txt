cmake_minimum_required(VERSION 3.22)

set(MICROROS_LIB_FILE "libmicroros_${FINEMOTE_MICROROS_VARIANT}.a")
set(MICROROS_LIB_PATH "${CMAKE_CURRENT_SOURCE_DIR}/${MICROROS_LIB_FILE}")

add_library(micro-ros STATIC IMPORTED GLOBAL)

set_property(TARGET micro-ros PROPERTY IMPORTED_LOCATION ${MICROROS_LIB_PATH})

target_include_directories(micro-ros INTERFACE
        "${CMAKE_CURRENT_SOURCE_DIR}/include"
)

target_link_libraries(micro-ros INTERFACE
        FreeRTOS-POSIX
)