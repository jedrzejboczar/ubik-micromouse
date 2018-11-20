/*
 * FreeRTOS Kernel V10.0.1
 * Copyright (C) 2017 Amazon.com, Inc. or its affiliates.  All Rights Reserved.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy of
 * this software and associated documentation files (the "Software"), to deal in
 * the Software without restriction, including without limitation the rights to
 * use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of
 * the Software, and to permit persons to whom the Software is furnished to do so,
 * subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS
 * FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
 * COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
 * IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
 * CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 *
 * http://www.FreeRTOS.org
 * http://aws.amazon.com/freertos
 *
 * 1 tab == 4 spaces!
 */

#ifndef FREERTOS_CONFIG_H
#define FREERTOS_CONFIG_H

/*-----------------------------------------------------------
 * Configuration, see: http://www.freertos.org/a00110.html
 *----------------------------------------------------------*/

/* Ensure stdint is only used by the compiler, and not the assembler. */
#if defined(__ICCARM__) || defined(__CC_ARM) || defined(__GNUC__)
    #include <stdint.h>
    #include "main.h"
    extern uint32_t SystemCoreClock;
#endif

/* Assertions function */
#if 0
// Default configASSERT version:
#define configASSERT(x)   if ((x) == 0) { taskDISABLE_INTERRUPTS(); for(;;); }
#else
// Use hook function for better debugging.
// Noticably increases binary size (because of filenames) !!!
// This function has to be implemented and should call taskDISABLE_INTERRUPTS().
extern void vApplicationConfigAssertFailedHook(const char *file, int line);
#define configASSERT(x)   if ((x) == 0) {                   \
    vApplicationConfigAssertFailedHook(__FILE__, __LINE__); \
    taskDISABLE_INTERRUPTS();                               \
    for(;;);                                                \
}
#endif

/* Tasks, scheduling and priorities */
#define configUSE_PREEMPTION                     1
#define configUSE_PORT_OPTIMISED_TASK_SELECTION  1
#define configMAX_PRIORITIES                     5
#define configMAX_TASK_NAME_LEN                  16

/* Clocking */
#define configUSE_16_BIT_TICKS                   0
#define configCPU_CLOCK_HZ                       ( SystemCoreClock )
#define configTICK_RATE_HZ                       ( (TickType_t) 1000 )

/* Memory and memory allocation */
#define configSUPPORT_STATIC_ALLOCATION          0
#define configSUPPORT_DYNAMIC_ALLOCATION         1
#define configMINIMAL_STACK_SIZE                 ( (uint16_t) 128 )
#define configTOTAL_HEAP_SIZE                    ( (size_t) (12 * 1024) )

/* Hook functions */
#define configUSE_IDLE_HOOK                      0
#define configUSE_TICK_HOOK                      0
#define configUSE_MALLOC_FAILED_HOOK             1
#define configUSE_DAEMON_TASK_STARTUP_HOOK       0

/* Stats gathering (good for development phase) */
#define configGENERATE_RUN_TIME_STATS            1
#define configUSE_TRACE_FACILITY                 1
#define configUSE_STATS_FORMATTING_FUNCTIONS     1
#define configCHECK_FOR_STACK_OVERFLOW           2  // requires hook function

/* Dummy implementation that should be adjusted when
 * we need vTaskGetRunTimeStats, also these should be 1:
 *    configGENERATE_RUN_TIME_STATS
 *    configUSE_TRACE_FACILITY
 *    configUSE_STATS_FORMATTING_FUNCTIONS
 * */
// #define portCONFIGURE_TIMER_FOR_RUN_TIME_STATS() { (void) 0; }
// #define portGET_RUN_TIME_COUNTER_VALUE() (1)
extern void configure_timer_for_runtime_stats(void);
extern uint32_t get_runtime_counter_value(void);
#define portCONFIGURE_TIMER_FOR_RUN_TIME_STATS() { configure_timer_for_runtime_stats(); }
#define portGET_RUN_TIME_COUNTER_VALUE() ( get_runtime_counter_value() )

/* Co-routine definitions. */
#define configUSE_CO_ROUTINES                    0
#define configMAX_CO_ROUTINE_PRIORITIES          2

/* FreeRTOS functionalities */
#define configUSE_MUTEXES				         1
#define configUSE_COUNTING_SEMAPHORES 	         1
#define configUSE_RECURSIVE_MUTEXES		         1
#define configQUEUE_REGISTRY_SIZE		         8

/* Set the following definitions to 1 to include the API function, or zero
to exclude the API function. */
#define INCLUDE_vTaskPrioritySet            1
#define INCLUDE_uxTaskPriorityGet           1
#define INCLUDE_vTaskDelete                 0
#define INCLUDE_vTaskCleanUpResources       0
#define INCLUDE_vTaskSuspend                1
#define INCLUDE_vTaskDelayUntil             1
#define INCLUDE_vTaskDelay                  1
#define INCLUDE_xTaskGetSchedulerState      1

/******************************************************************************
 *** Cortex-M specific definitions (as generated by STM32CubeMX)
 ******************************************************************************/

#ifdef __NVIC_PRIO_BITS
    /* __BVIC_PRIO_BITS will be specified when CMSIS is being used. */
    #define configPRIO_BITS         __NVIC_PRIO_BITS
#else
    #define configPRIO_BITS         4
#endif

/* The lowest interrupt priority that can be used in a call to a "set priority"
function. */
#define configLIBRARY_LOWEST_INTERRUPT_PRIORITY   15

/* The highest interrupt priority that can be used by any interrupt service
routine that makes calls to interrupt safe FreeRTOS API functions.  DO NOT CALL
INTERRUPT SAFE FREERTOS API FUNCTIONS FROM ANY INTERRUPT THAT HAS A HIGHER
PRIORITY THAN THIS! (higher priorities are lower numeric values. */
#define configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY 5

/* Interrupt priorities used by the kernel port layer itself.  These are generic
to all Cortex-M ports, and do not rely on any particular library functions. */
#define configKERNEL_INTERRUPT_PRIORITY 		( configLIBRARY_LOWEST_INTERRUPT_PRIORITY << (8 - configPRIO_BITS) )
/* !!!! configMAX_SYSCALL_INTERRUPT_PRIORITY must not be set to zero !!!!
See http://www.FreeRTOS.org/RTOS-Cortex-M3-M4.html. */
#define configMAX_SYSCALL_INTERRUPT_PRIORITY 	( configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY << (8 - configPRIO_BITS) )

/* Definitions that map the FreeRTOS port interrupt handlers to their CMSIS
standard names. */
#define vPortSVCHandler    SVC_Handler
#define xPortPendSVHandler PendSV_Handler

/* IMPORTANT: This define MUST be commented when used with STM32Cube firmware,
              to prevent overwriting SysTick_Handler defined within STM32Cube HAL */
#define xPortSysTickHandler SysTick_Handler

#endif /* FREERTOS_CONFIG_H */
