cmake_minimum_required(VERSION 3.22)

#要求在BSP/xxx_project/CmakePresets.json中配置MICROROS_LIB_FILE
if(NOT DEFINED MICROROS_LIB_FILE OR MICROROS_LIB_FILE STREQUAL "")
    message(FATAL_ERROR
            "MICROROS_LIB_FILE is not configured for board: ${BOARD_NAME}"
    )
endif()

set(MICROROS_LIB_PATH "${CMAKE_CURRENT_SOURCE_DIR}/${MICROROS_LIB_FILE}")

add_library(micro-ROS STATIC IMPORTED GLOBAL)

set_property(TARGET micro-ROS PROPERTY IMPORTED_LOCATION ${MICROROS_LIB_PATH})

target_include_directories(micro-ROS INTERFACE
        "${CMAKE_CURRENT_SOURCE_DIR}/include"
)

if(ENABLE_FREERTOS_POSIX)
    target_link_libraries(finemote_core PUBLIC
            FreeRTOS-POSIX)
endif()