cmake_minimum_required(VERSION 3.11)

project(ofdm_txrx VERSION 0.0.1 LANGUAGES C)
message(STATUS "CMAKE_BUILD_TYPE: ${CMAKE_BUILD_TYPE}")

option(OFDM_BUILD_EXAMPLES "Build ofdm_txrx examples" ON)

add_library(${PROJECT_NAME} STATIC
    source/ofdm.c
    bitstream/src/bitstream.c
)

target_compile_options(${PROJECT_NAME} PUBLIC
    -Wall -Wextra
)

target_include_directories(${PROJECT_NAME} PUBLIC
    include
)
target_include_directories(${PROJECT_NAME} PRIVATE
    bitstream/include
)

target_compile_features(${PROJECT_NAME} PRIVATE c_std_11)

set(KISSFFT_PKGCONFIG OFF CACHE BOOL "KISSFFT_PKGCONFIG" FORCE)
set(KISSFFT_TEST OFF CACHE BOOL "KISSFFT_TEST" FORCE)
set(KISSFFT_TOOLS OFF CACHE BOOL "KISSFFT_TOOLS" FORCE)
set(KISSFFT_STATIC ON CACHE BOOL "KISSFFT_STATIC" FORCE)
add_subdirectory(kissfft)

add_subdirectory(crc32)

target_link_libraries(${PROJECT_NAME} PRIVATE
    kissfft
    crc32
)

if(OFDM_BUILD_EXAMPLES)
        add_executable(${PROJECT_NAME}_example_pc
            examples/main_pc.c
        )
        target_compile_features(${PROJECT_NAME}_example_pc PRIVATE c_std_11)
        target_include_directories(${PROJECT_NAME}_example_pc PRIVATE
            dr_libs
        )
        target_link_libraries(${PROJECT_NAME}_example_pc PRIVATE
            ${PROJECT_NAME}
        )
        target_compile_options(${PROJECT_NAME}_example_pc PUBLIC
            -Wall -Wextra
        )
endif()
