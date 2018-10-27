#pragma once


#define ARRAY_LEN(arr)   (sizeof(arr) / sizeof(*(arr)))

static constexpr auto bit(int number) {
    return 1 << number;
}

