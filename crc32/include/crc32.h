/*
 *    crc32.h - calculate crc32 checksums of data
 *
 *    Written in 2020 by Ayman El Didi
 *    Modified in 2024 to allow for block by block computation
 *
 *    To the extent possible under law, the author(s) have dedicated all
 *    domain worldwide. This software is distributed without any warranty.
 *
 *    You should have received a copy of the CC0 Public Domain Dedication along
 *    with this software. If not, see
 *    <http://creativecommons.org/publicdomain/zero/1.0/>.
 */

#ifdef __cplusplus
extern "C" {
#endif

#include <stddef.h>
#include <stdint.h>

/* Calculate crc32 checksum of size_t size bytes of const void *input and
 * return the result. Set initial_crc to 0, or update it with the return
 * value for every new block.
 *
 * No lookup tables are used, its simply a byte by byte crc32 algorithm.
 */
uint32_t
crc32(const void *input, size_t size, uint32_t initial_crc);

#ifdef __cplusplus
}
#endif
