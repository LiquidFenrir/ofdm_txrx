
add_library(crc32 STATIC
    src/crc32.c
    include/crc32.h
)

target_include_directories(crc32 INTERFACE
    include/
)
