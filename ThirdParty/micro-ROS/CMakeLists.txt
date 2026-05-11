cmake_minimum_required(VERSION 3.22)

string(REGEX REPLACE "cortex-(m[0-9]+)" "\\1" MICROROS_VARIANT "${CMAKE_SYSTEM_PROCESSOR}")
set(MICROROS_LIB_FILE "libmicroros_${MICROROS_VARIANT}.a")
set(MICROROS_LIB_PATH "${CMAKE_CURRENT_SOURCE_DIR}/${MICROROS_LIB_FILE}")

add_library(micro-ROS STATIC IMPORTED GLOBAL)

set_property(TARGET micro-ROS PROPERTY IMPORTED_LOCATION ${MICROROS_LIB_PATH})

target_include_directories(micro-ROS INTERFACE
        "${CMAKE_CURRENT_SOURCE_DIR}/include"
)

target_link_libraries(micro-ROS INTERFACE
        FreeRTOS-POSIX
)