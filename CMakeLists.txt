cmake_minimum_required(VERSION 3.11)

project(ofdm_txrx VERSION 0.0.1 LANGUAGES C)

# Source code files
# find source -type f | grep "\.c" | clip
add_executable(${PROJECT_NAME}
    source/main.c
    source/ofdm.c
    bitstream/src/bitstream.c
)

target_include_directories(${PROJECT_NAME} PRIVATE
    include
    bitstream/include
    dr_libs
)

target_compile_features(${PROJECT_NAME} PRIVATE c_std_11)

# Options for code generation
# target_compile_options(${PROJECT_NAME} PRIVATE
#   -Wall -Wextra -Wno-psabi -Wno-unused-parameter
# )
# target_compile_definitions(${PROJECT_NAME} PRIVATE )

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
