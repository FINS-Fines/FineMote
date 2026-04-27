set(_FINEMOTE_FIRMWARE_LINK_OPTIONS
        "--map"
        "--summary_stderr"
        "--info=summarysizes"
        "--info=sizes"
        "--info=totals"
        "--info=unused"
        "--info=veneers"
        "--load_addr_map_info"
        "--xref"
        "--callgraph"
        "--symbols"
        CACHE INTERNAL "ArmClang firmware link options"
        FORCE
)

macro(finemote_apply_board_toolchain)
    if (NOT DEFINED CMAKE_SYSTEM_PROCESSOR OR CMAKE_SYSTEM_PROCESSOR STREQUAL "")
        message(FATAL_ERROR "CMAKE_SYSTEM_PROCESSOR is not set.")
    endif ()

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
    set(CMAKE_ASM_FLAGS_INIT "${CMAKE_ASM_FLAGS_INIT} ${_FINEMOTE_ARCH_FLAGS_STRING} -masm=auto")

    set(_FINEMOTE_ARMLINK_CPU "${CMAKE_SYSTEM_PROCESSOR}")
    if (DEFINED FINEMOTE_FPU AND NOT FINEMOTE_FPU STREQUAL "")
        if (FINEMOTE_FPU MATCHES "-sp-")
            string(APPEND _FINEMOTE_ARMLINK_CPU ".fp.sp")
        elseif (FINEMOTE_FPU MATCHES "-dp-")
            string(APPEND _FINEMOTE_ARMLINK_CPU ".fp.dp")
        endif ()
    endif ()

    set(_FINEMOTE_SCATTER_FILE "${_FINEMOTE_PROJECT_DIR}/BSP/${BOARD_NAME}/MDK-ARM/${BOARD_NAME}/${BOARD_NAME}.sct")

    set(_FINEMOTE_BOARD_LINK_OPTIONS
            "--scatter=${_FINEMOTE_SCATTER_FILE}"
            "--cpu=${_FINEMOTE_ARMLINK_CPU}"
            "--strict"
            CACHE INTERNAL "ArmClang board link options"
            FORCE
    )

    string(JOIN " " _FINEMOTE_BOARD_LINK_FLAGS ${_FINEMOTE_BOARD_LINK_OPTIONS})
    set(CMAKE_EXE_LINKER_FLAGS_INIT "${CMAKE_EXE_LINKER_FLAGS_INIT} ${_FINEMOTE_BOARD_LINK_FLAGS}")
endmacro()

function(finemote_postprocess target)
    target_link_options(${target} PRIVATE
            "--list=$<TARGET_FILE_DIR:${target}>/$<TARGET_FILE_BASE_NAME:${target}>.map"
            ${_FINEMOTE_FIRMWARE_LINK_OPTIONS}
    )

    add_custom_command(TARGET ${target} POST_BUILD
            COMMAND ${CMAKE_OBJCOPY} --i32combined --output "$<TARGET_FILE_DIR:${target}>/$<TARGET_FILE_BASE_NAME:${target}>.hex" "$<TARGET_FILE:${target}>"
            VERBATIM
    )
endfunction()
