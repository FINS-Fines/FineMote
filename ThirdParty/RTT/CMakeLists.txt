add_library(RTT STATIC)

target_sources(RTT PRIVATE
        "${CMAKE_CURRENT_SOURCE_DIR}/SEGGER_RTT.c"
        "${CMAKE_CURRENT_SOURCE_DIR}/SEGGER_RTT_printf.c"
)

target_include_directories(RTT PUBLIC
        "${CMAKE_CURRENT_SOURCE_DIR}"
)

target_link_libraries(RTT PRIVATE finemote::board)
