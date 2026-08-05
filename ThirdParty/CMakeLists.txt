cmake_minimum_required(VERSION 3.22)
if(ENABLE_CMSISDSP)
    set(CMSISCORE "${PROJECT_SOURCE_DIR}/BSP/${BOARD_NAME}/Drivers/CMSIS")
    add_subdirectory(CMSIS-DSP)

    target_link_libraries(CMSISDSP PRIVATE finemote::board)
endif()


if(ENABLE_FREERTOS_POSIX)
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

if(ENABLE_ETL)
    add_subdirectory(etl)
endif()

if(ENABLE_RTT)
    add_subdirectory(RTT)
endif()

if(ENABLE_MICRO_ROS)
    add_subdirectory(micro-ROS)
endif()


# ---------- BLASFEO ----------
if(ENABLE_BLASFEO)
    if(NOT EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/blasfeo/CMakeLists.txt")
        message(WARNING "BLASFEO not found in ThirdParty/blasfeo")
    else()
        # Force TARGET=GENERIC (must use variable name TARGET). Some BLASFEO
        # paths check BLASFEO_TARGET as well.
        set(TARGET GENERIC CACHE STRING "Target architecture" FORCE)
        set(BLASFEO_TARGET GENERIC CACHE STRING "Target architecture" FORCE)

        # Backend and matrix format. COLMAJ triggers errors on this target;
        # use PANELMAJ.
        set(LA HIGH_PERFORMANCE CACHE STRING "LA backend" FORCE)
        set(MF PANELMAJ CACHE STRING "Matrix format" FORCE)

        # STM32 stacks are small. Default K_MAX_STACK=300 makes
        # double pU_stack[4*300] ~9.6 KB on every BLAS GEMM/SYRK/TRMM call,
        # overflowing the FreeRTOS CLI task and corrupting results.
        set(K_MAX_STACK 16 CACHE STRING "Max k using stack memory in BLASFEO" FORCE)

        # Export Eigen-compatible F77 symbols (dgemm_, dpotrf_, ...) directly
        # from BLASFEO — avoids an extra shim call (blasfeo_blas_dgemm ->
        # dgemm_).
        set(FORTRAN_BLAS_API ON CACHE BOOL "Standard fortran BLAS API" FORCE)

        # Disable external dependencies (avoids POSIX headers like sys/stat.h).
        set(EXT_DEP OFF CACHE BOOL "Disable external dependencies" FORCE)
        set(EXT_DEP_MALLOC OFF CACHE BOOL "Disable malloc external dependencies" FORCE)

        # Cross-compile flags (do not affect TestSingleTarget but may affect
        # other checks).
        set(BLASFEO_CROSSCOMPILING ON CACHE BOOL "Cross-compiling" FORCE)

        # Disable examples and tests to reduce build footprint.
        set(BLASFEO_EXAMPLES OFF CACHE BOOL "Build examples" FORCE)
        set(BLASFEO_TESTS OFF CACHE BOOL "Build tests" FORCE)

        # Override TestSingleTarget so BLASFEO's CMake skips its x86 target
        # probes during cross-compilation (would otherwise fail to build).
        function(TestSingleTarget)
            message(STATUS "BLASFEO: Skipping TestSingleTarget (cross-compilation for GENERIC)")
        endfunction()

        add_subdirectory(blasfeo)

        message(STATUS "BLASFEO added with TARGET=GENERIC, LA=${LA}, MF=${MF}, FORTRAN_BLAS_API=ON")
    endif()
endif()

# ---------- Eigen ----------
if(ENABLE_EIGEN)
    if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/eigen/")
        target_include_directories(finemote_core PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/eigen")
    else()
        message(WARNING "Eigen not found in ThirdParty/eigen")
    endif()
endif()
