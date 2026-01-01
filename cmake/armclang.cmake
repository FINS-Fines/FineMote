set(CMAKE_SYSTEM_NAME Generic)
set(CMAKE_SYSTEM_PROCESSOR arm)

set(CMAKE_C_COMPILER armclang)
set(CMAKE_ASM_COMPILER armclang)
set(CMAKE_CXX_COMPILER armclang)

set(CMAKE_LINKER armlink)
set(CMAKE_AR armar)
set(CMAKE_OBJCOPY fromelf)
set(CMAKE_SIZE fromelf)

set(TRIPLE arm-arm-none-eabi)

#set(CMAKE_C_COMPILER_TARGET ${TRIPLE})
#set(CMAKE_CXX_COMPILER_TARGET ${TRIPLE})
#set(CMAKE_ASM_COMPILER_TARGET ${TRIPLE})

## for clion to detect the compiler info
set(CMAKE_C_FLAGS_INIT "--target=${TRIPLE}")
set(CMAKE_CXX_FLAGS_INIT "--target=${TRIPLE}")
set(CMAKE_ASM_FLAGS_INIT "--target=${TRIPLE}")

set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)

set(CMAKE_EXECUTABLE_SUFFIX_ASM ".elf")
set(CMAKE_EXECUTABLE_SUFFIX_C ".elf")
set(CMAKE_EXECUTABLE_SUFFIX_CXX ".elf")

message(STATUS "Configured for armclang toolchain targeting ${BOARD_NAME}")