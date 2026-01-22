set(CMAKE_SYSTEM_NAME Generic)
set(CMAKE_SYSTEM_PROCESSOR cortex-m4)

set(_FINEMOTE_ARCH_FLAGS
        -mcpu=${CMAKE_SYSTEM_PROCESSOR}
        -mthumb
        -mfpu=fpv4-sp-d16
        -mfloat-abi=hard
)

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

string(JOIN " " _FINEMOTE_ARCH_FLAGS
        ${_FINEMOTE_ARCH_FLAGS})
string(JOIN " " _FINEMOTE_COMMON_FLAGS
        ${_FINEMOTE_ABI_FLAGS}
        ${_FINEMOTE_SECTION_FLAGS}
)

set(FINEMOTE_ARMLINK_CPU "Cortex-M4.fp.sp")

set(CMAKE_C_FLAGS_INIT "${CMAKE_C_FLAGS_INIT} ${_FINEMOTE_COMMON_FLAGS}")
set(CMAKE_CXX_FLAGS_INIT "${CMAKE_CXX_FLAGS_INIT} ${_FINEMOTE_COMMON_FLAGS}")
set(CMAKE_ASM_FLAGS_INIT "${CMAKE_ASM_FLAGS_INIT} ${_FINEMOTE_ARCH_FLAGS} -masm=auto")
