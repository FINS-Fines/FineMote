cmake_minimum_required(VERSION 3.22)

project(${BOARD_NAME} C CXX ASM)

add_executable(${PROJECT_NAME})

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

set(_FINEMOTE_WARNING_FLAGS
        -Wall
        -Wextra
        -Wpedantic
)

add_compile_options(${_FINEMOTE_WARNING_FLAGS})

# --------------- load finemote core ---------------
add_library(finemote_core OBJECT)
add_library(finemote::core ALIAS finemote_core)

set(CORE_MODULE
        Algorithms
        Components
        Devices
        Interface
        Services
)

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

# --------------- load finemote bsp ---------------
add_subdirectory(${PROJECT_SOURCE_DIR}/BSP/${BOARD_NAME})

target_link_libraries(finemote_core PUBLIC finemote::board)

# --------------- load third party libs ---------------
add_subdirectory(ThirdParty)

target_link_libraries(finemote_core PUBLIC
        CMSISDSP
        etl::etl
        FreeRTOS-POSIX
)

# ------------------ application ------------------
target_link_libraries(${PROJECT_NAME} PRIVATE
        finemote::core
        finemote::bsp
)

finemote_postprocess(${PROJECT_NAME})
