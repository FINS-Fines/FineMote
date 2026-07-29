if(BOARD_NAME_UPPER IN_LIST ESP_IDF_BOARDS)
    set(ETL_INCLUDE_DIR ${CMAKE_SOURCE_DIR}/ThirdParty/etl/include)

    add_library(etl::etl INTERFACE IMPORTED)

    set_target_properties(etl::etl PROPERTIES
            INTERFACE_INCLUDE_DIRECTORIES "${ETL_INCLUDE_DIR}"
    )

    add_subdirectory(micro-ROS)

elseif(BOARD_NAME_UPPER IN_LIST STM32_BOARDS)
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
endif()