cmake_minimum_required(VERSION 3.22)


add_library(micro-ros STATIC IMPORTED GLOBAL)

set_property(TARGET micro-ros PROPERTY IMPORTED_LOCATION
        "${CMAKE_CURRENT_SOURCE_DIR}/libmicroros.a"
)

target_include_directories(micro-ros INTERFACE
        "${CMAKE_CURRENT_SOURCE_DIR}/include"
)

target_link_libraries(micro-ros INTERFACE
        FreeRTOS-POSIX
)