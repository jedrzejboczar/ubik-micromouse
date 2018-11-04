#include <cstddef>

#include "FreeRTOS.h"

void* operator new(size_t size) {
    void *memory = pvPortMalloc(size);
#ifdef	__EXCEPTIONS
    if (memory == 0) // did pvPortMalloc succeed?
        throw std::bad_alloc(); // ANSI/ISO compliant behavior
#endif
    return memory;
}

void operator delete(void *memory) noexcept {
    vPortFree(memory);
}

void operator delete(void *memory, size_t) { // ? required by C++14 ?
    vPortFree(memory);
}

