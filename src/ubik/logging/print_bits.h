#pragma once

#include <cstddef>

static inline void print_bits(const void *memory,
        size_t start_bit_num, size_t n_bits, char *output, bool spaces=true)
{
#if __BYTE_ORDER__ == __ORDER_BIG_ENDIAN__
    int is_big_endian = 1;
#elif __BYTE_ORDER__ == __ORDER_LITTLE_ENDIAN__
    int is_big_endian = 0;
#endif
    int output_iter = 0;

    const unsigned char * const bytes = (const unsigned char *) memory;
    int byte_n = 0, curr_bit_n = start_bit_num;
    output[output_iter++] = '[';
    if (spaces)
        output[output_iter++] = ' ';
    for (size_t i = 0; i < n_bits; i++) {
        int bit_val;
        if (is_big_endian)
            bit_val = (bytes[byte_n] & (1 << curr_bit_n)) != 0;
        else
            bit_val = (bytes[byte_n] & (1 << (7-curr_bit_n))) != 0;
        output[output_iter++] = bit_val ? '1' : '0';
        curr_bit_n++;
        if (curr_bit_n >= 8) {
            curr_bit_n = 0;
            byte_n++;
            if (spaces)
                output[output_iter++] = ' ';
        }
    }
    output[output_iter++] = ']';
}
