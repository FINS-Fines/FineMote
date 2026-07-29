cmake_minimum_required(VERSION 3.22)

#---------------- check BOARD_NAME -----------------
if(NOT DEFINED BOARD_NAME)
    message(FATAL_ERROR "BOARD_NAME is not set\n"
            "Please set --preset board_name in CMake option")
else()
    string(TOUPPER "${BOARD_NAME}" BOARD_NAME_UPPER)
endif()

set(STM32_BOARDS "MC_BOARD" "ROBOMASTER_C" "STM32_HAL_FREERTOS_TEST" "MC_BOARD_02")
set(ESP_IDF_BOARDS "ESP32S3" )

if(BOARD_NAME_UPPER IN_LIST ESP_IDF_BOARDS OR BOARD_NAME_UPPER IN_LIST STM32_BOARDS)
    message("-- BOARD_NAME: ${BOARD_NAME}")
else()
    message(FATAL_ERROR
            "Unsupported BOARD_NAME: ${BOARD_NAME}.\n"
            "Supported values are: ${STM32_BOARDS} ${ESP_IDF_BOARDS} \n"
            "If you use another board, please add it to STM32_BOARDS or ESP_IDF_BOARDS in CMakeLists.txt"
    )
endif()

#---------------- project config -----------------
project(${BOARD_NAME} C CXX ASM)
if(BOARD_NAME_UPPER IN_LIST ESP_IDF_BOARDS)
    #
elseif(BOARD_NAME_UPPER IN_LIST STM32_BOARDS)
    add_executable(${PROJECT_NAME}
            ${_FINEMOTE_STARTUP_SOURCE}
    )
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(BOARD_NAME_UPPER IN_LIST ESP_IDF_BOARDS)
    #
elseif(BOARD_NAME_UPPER IN_LIST STM32_BOARDS)
    set(_FINEMOTE_WARNING_FLAGS
            -Wall
            -Wextra
            -Wpedantic
    )

    add_compile_options(${_FINEMOTE_WARNING_FLAGS})
endif()



# --------------- load finemote core ---------------
add_library(finemote_core OBJECT)

# finemote_core编译时，依据不同宏选择包含不同的头文件
if(BOARD_NAME_UPPER IN_LIST ESP_IDF_BOARDS)
    target_compile_definitions(finemote_core PRIVATE BSP_ESPIDF)
elseif(BOARD_NAME_UPPER IN_LIST STM32_BOARDS)
    target_compile_definitions(finemote_core PRIVATE BSP_STM32)
endif()

add_library(finemote::core ALIAS finemote_core)

if(BOARD_NAME_UPPER IN_LIST ESP_IDF_BOARDS)
    set(CORE_MODULE
            Algorithms
            Components
            Devices
            Interface
            Services
            #BSP/${BOARD_NAME_UPPER}/components/finemote_bsp/include
            ThirdParty/etl/include
            BSP/Esp32s3/components/finemote_core/include
    )
elseif(BOARD_NAME_UPPER IN_LIST STM32_BOARDS)
    set(CORE_MODULE
            Algorithms
            Components
            Devices
            Interface
            Services
    )
endif()

function(add_module module_path)
    # glob source files
    file(GLOB_RECURSE module_sources CONFIGURE_DEPENDS
            "${module_path}/*.c"
            "${module_path}/*.cpp"
            "${module_path}/*.s"
    )
    target_sources(finemote_core PRIVATE ${module_sources})

    # glob header directories
    file(GLOB_RECURSE module_headers CONFIGURE_DEPENDS
            "${module_path}/*.h"
            "${module_path}/*.h++"
            "${module_path}/*.hpp"
    )

    if (module_headers)
        target_include_directories(finemote_core PUBLIC "${module_path}")
    endif ()
endfunction()

foreach (module IN LISTS CORE_MODULE)
    set(module_path "${PROJECT_SOURCE_DIR}/${module}")
    if (NOT IS_DIRECTORY ${module_path})
        message(WARNING "Module path does not exist or is not a directory: ${module_path}")
        continue()
    endif ()
    add_module(${module_path})
endforeach ()

target_compile_options(finemote_core PRIVATE
        "-fno-rtti"
        "-fno-exceptions"
)

message(STATUS "finemote_core initialized with modules: ${CORE_MODULE}")
# --------------- load third party libs ---------------
add_subdirectory(ThirdParty)

# --------------- load finemote bsp ---------------
if(BOARD_NAME_UPPER IN_LIST ESP_IDF_BOARDS)
    add_subdirectory(BSP/${BOARD_NAME})

elseif(BOARD_NAME_UPPER IN_LIST STM32_BOARDS)
    add_subdirectory(${PROJECT_SOURCE_DIR}/BSP/${BOARD_NAME})
    target_link_libraries(finemote_core PUBLIC finemote::board)
endif()
# --------------- load third party libs ---------------
if(BOARD_NAME_UPPER IN_LIST ESP_IDF_BOARDS)
    #由于链接顺序，不允许在此处链接
elseif(BOARD_NAME_UPPER IN_LIST STM32_BOARDS)
    target_link_libraries(finemote_core PUBLIC
            CMSISDSP
            etl::etl
            FreeRTOS-POSIX
    )
endif()
# ------------------ application ------------------
if(BOARD_NAME_UPPER IN_LIST ESP_IDF_BOARDS)

elseif(BOARD_NAME_UPPER IN_LIST STM32_BOARDS)
    target_link_libraries(${PROJECT_NAME} PRIVATE
            finemote::core
            finemote::bsp
    )
    finemote_postprocess(${PROJECT_NAME})
endif()