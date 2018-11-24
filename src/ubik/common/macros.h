#pragma once


// #define ARRAY_LEN(arr)   (sizeof(arr) / sizeof(*(arr)))
// better version (C++):
template <typename T, int N>
constexpr int n_elements(T(&)[N]) {
    return N;
}

static constexpr auto bit(int number) {
    return 1 << number;
}


