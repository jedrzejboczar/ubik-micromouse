#include "stdint.h"

/* The prototype shows it is a naked function - in effect this is just an
assembly function. */
void HardFault_Handler(void) __attribute__( ( naked ) );

/* The fault handler implementation calls a function called
prvGetRegistersFromStack(). */
void HardFault_Handler(void)
{
    __asm volatile
    (
        " tst lr, #4                                                \n"
        " ite eq                                                    \n"
        " mrseq r0, msp                                             \n"
        " mrsne r0, psp                                             \n"
        " ldr r1, [r0, #24]                                         \n"
        " ldr r2, handler2_address_const                            \n"
        " bx r2                                                     \n"
        " handler2_address_const: .word prvGetRegistersFromStack    \n"
    );
}


/* Suppress complier warnings about unsued variables (GCC), disable optimization */
#define DEBUG_VARIABLE __attribute__((unused)) volatile

void prvGetRegistersFromStack(uint32_t *pulFaultStackAddress)
{
    /* These are volatile to try and prevent the compiler/linker optimising them
       away as the variables never actually get used.  If the debugger won't show the
       values of the variables, make them global my moving their declaration outside
       of this function. */
    DEBUG_VARIABLE uint32_t r0;
    DEBUG_VARIABLE uint32_t r1;
    DEBUG_VARIABLE uint32_t r2;
    DEBUG_VARIABLE uint32_t r3;
    DEBUG_VARIABLE uint32_t r12;
    DEBUG_VARIABLE uint32_t lr; /* Link register. */
    DEBUG_VARIABLE uint32_t pc; /* Program counter. */
    DEBUG_VARIABLE uint32_t psr;/* Program status register. */

    r0 = pulFaultStackAddress[ 0 ];
    r1 = pulFaultStackAddress[ 1 ];
    r2 = pulFaultStackAddress[ 2 ];
    r3 = pulFaultStackAddress[ 3 ];

    r12 = pulFaultStackAddress[ 4 ];
    lr = pulFaultStackAddress[ 5 ];
    pc = pulFaultStackAddress[ 6 ];
    psr = pulFaultStackAddress[ 7 ];

    /* Suppress complier warnings about unsued variables (compiler-independant) */
    // (void) (r0 + r1 + r2 + r3 + r12 + lr + pc + psr);

    /* When the following line is hit, the variables contain the register values. */
    __asm("bkpt #0\n") ; // Break into the debugger
    for( ;; );
}
