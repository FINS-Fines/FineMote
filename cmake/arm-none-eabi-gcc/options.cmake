macro(finemote_toolchain)
    if (NOT DEFINED CMAKE_SYSTEM_PROCESSOR OR CMAKE_SYSTEM_PROCESSOR STREQUAL "")
        message(FATAL_ERROR "CMAKE_SYSTEM_PROCESSOR is not set.")
    endif ()

    # compiler cpu and abi flags
    set(_FINEMOTE_ARCH_FLAGS
            -mcpu=${CMAKE_SYSTEM_PROCESSOR}
            -mthumb
    )

    if (DEFINED FINEMOTE_FPU AND NOT FINEMOTE_FPU STREQUAL "")
        list(APPEND _FINEMOTE_ARCH_FLAGS -mfpu=${FINEMOTE_FPU})
    endif ()

    if (DEFINED FINEMOTE_FLOAT_ABI AND NOT FINEMOTE_FLOAT_ABI STREQUAL "")
        list(APPEND _FINEMOTE_ARCH_FLAGS -mfloat-abi=${FINEMOTE_FLOAT_ABI})
    endif ()

    set(_FINEMOTE_ABI_FLAGS
            ${_FINEMOTE_ARCH_FLAGS}
            -funsigned-char
            -fshort-enums
            -fshort-wchar
    )

    set(_FINEMOTE_SECTION_FLAGS
            -ffunction-sections
            -fdata-sections
    )

    string(JOIN " " _FINEMOTE_ARCH_FLAGS_STRING ${_FINEMOTE_ARCH_FLAGS})
    string(JOIN " " _FINEMOTE_COMMON_FLAGS_STRING
            ${_FINEMOTE_ABI_FLAGS}
            ${_FINEMOTE_SECTION_FLAGS}
    )

    set(CMAKE_C_FLAGS_INIT "${CMAKE_C_FLAGS_INIT} ${_FINEMOTE_COMMON_FLAGS_STRING}")
    set(CMAKE_CXX_FLAGS_INIT "${CMAKE_CXX_FLAGS_INIT} ${_FINEMOTE_COMMON_FLAGS_STRING}")
    set(CMAKE_ASM_FLAGS_INIT "${CMAKE_ASM_FLAGS_INIT} ${_FINEMOTE_ARCH_FLAGS_STRING} -x assembler-with-cpp")

    # startup file
    set(_FINEMOTE_BOARD_DIR "${_FINEMOTE_PROJECT_DIR}/BSP/${BOARD_NAME}")
    file(GLOB _FINEMOTE_STARTUP_CANDIDATES "${_FINEMOTE_BOARD_DIR}/Core/Startup/startup*.s")
    list(LENGTH _FINEMOTE_STARTUP_CANDIDATES _FINEMOTE_STARTUP_COUNT)
    if (NOT _FINEMOTE_STARTUP_COUNT EQUAL 1)
        message(FATAL_ERROR "Expected exactly one GCC startup file under ${_FINEMOTE_BOARD_DIR}/Core/Startup, found ${_FINEMOTE_STARTUP_COUNT}.")
    endif ()
    list(GET _FINEMOTE_STARTUP_CANDIDATES 0 _FINEMOTE_STARTUP_SOURCE)
    set(_FINEMOTE_STARTUP_SOURCE "${_FINEMOTE_STARTUP_SOURCE}" CACHE INTERNAL "Board startup source" FORCE)

    # linker script
    file(GLOB _FINEMOTE_LINKER_SCRIPT "${_FINEMOTE_BOARD_DIR}/*_FLASH.ld")
    list(LENGTH _FINEMOTE_LINKER_SCRIPT _FINEMOTE_LINKER_COUNT)
    if (NOT _FINEMOTE_LINKER_COUNT EQUAL 1)
        message(FATAL_ERROR "Expected exactly one linker script under ${_FINEMOTE_BOARD_DIR}, found ${_FINEMOTE_LINKER_COUNT}.")
    endif ()

    # linker options
    set(_FINEMOTE_BOARD_LINK_OPTIONS
            ${_FINEMOTE_ABI_FLAGS}
            -T${_FINEMOTE_LINKER_SCRIPT}
            -Wl,-Map=$<TARGET_FILE_DIR:@TARGET@>/$<TARGET_FILE_BASE_NAME:@TARGET@>.map
            -Wl,--gc-sections
            -Wl,--print-memory-usage
            -Wl,--no-wchar-size-warning
            -specs=nano.specs
            -specs=nosys.specs
            CACHE INTERNAL "arm-none-eabi-gcc board link options"
            FORCE
    )
endmacro()

function(finemote_postprocess target)
    # map and report files
    set(_FINEMOTE_TARGET_LINK_OPTIONS ${_FINEMOTE_BOARD_LINK_OPTIONS})
    list(TRANSFORM _FINEMOTE_TARGET_LINK_OPTIONS REPLACE "@TARGET@" "${target}")

    target_link_options(${target} PRIVATE
            ${_FINEMOTE_TARGET_LINK_OPTIONS}
    )

    # firmware image
    add_custom_command(TARGET ${target} POST_BUILD
            COMMAND ${CMAKE_OBJCOPY} -O ihex "$<TARGET_FILE:${target}>" "$<TARGET_FILE_DIR:${target}>/$<TARGET_FILE_BASE_NAME:${target}>.hex"
            COMMAND ${CMAKE_SIZE} "$<TARGET_FILE:${target}>"
            VERBATIM
    )
endfunction()
