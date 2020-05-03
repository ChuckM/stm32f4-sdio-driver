
#ifndef FREERTOS_CONFIG_H
#define FREERTOS_CONFIG_H

/*
 * Source: http://www.freertos.org/a00110.html
 */

/* Here is a good place to include header files that are required across
 * your application. */
//#include "something.h"

/*-----------------------------------------------------------
 * Application specific definitions.
 *
 * These definitions should be adjusted for your particular hardware and
 * application requirements.
 *
 * THESE PARAMETERS ARE DESCRIBED WITHIN THE 'CONFIGURATION' SECTION OF THE
 * FreeRTOS API DOCUMENTATION AVAILABLE ON THE FreeRTOS.org WEB SITE. 
 *
 * See http://www.freertos.org/a00110.html.
 *----------------------------------------------------------*/

void vAssertCalled( const char * const pcFileName, unsigned long ulLine );

#define configUSE_PREEMPTION                    0
#define configUSE_IDLE_HOOK                     0
#define configUSE_TICK_HOOK                     1
#define configCPU_CLOCK_HZ                      ( ( unsigned long ) 168000000 )  
#define configSYSTICK_CLOCK_HZ                  ( configCPU_CLOCK_HZ / 8 )  /* fix for vTaskDelay() */
#define configTICK_RATE_HZ                      1000
#define configMAX_PRIORITIES                    5
#define configMINIMAL_STACK_SIZE                128
#define configTOTAL_HEAP_SIZE                   (15 * 1024)
#define configMAX_TASK_NAME_LEN                 16
#define configUSE_TRACE_FACILITY                0
#define configUSE_16_BIT_TICKS                  0
#define configIDLE_SHOULD_YIELD                 1
#define configUSE_MUTEXES                       1
#define configUSE_COUNTING_SEMAPHORES           1
#define configCHECK_FOR_STACK_OVERFLOW          2

/* Co-routine definitions. */
#define configUSE_CO_ROUTINES           0
#define configMAX_CO_ROUTINE_PRIORITIES 2

/* Interrupt nesting behaviour configuration. */
#define configKERNEL_INTERRUPT_PRIORITY         (0x0f << 4)
#define configMAX_SYSCALL_INTERRUPT_PRIORITY    (5 << 4)
#define configMAX_API_CALL_INTERRUPT_PRIORITY   (5 << 4)
/* This is the value being used as per the ST library which permits 16
priority values, 0 to 15.  This must correspond to the
configKERNEL_INTERRUPT_PRIORITY setting.  Here 15 corresponds to the lowest
NVIC value of 255. */
#define configLIBRARY_KERNEL_INTERRUPT_PRIORITY 15

/* Define to trap errors during development. */
#define configASSERT( x )     if( ( x ) == 0 ) vAssertCalled( __FILE__, __LINE__ )

/* Set the following definitions to 1 to include the API function, or zero
to exclude the API function. */

/* Optional functions - most linkers will remove unused functions anyway. */
#define INCLUDE_vTaskPrioritySet                1
#define INCLUDE_uxTaskPriorityGet               1
#define INCLUDE_vTaskDelete                     1
#define INCLUDE_vTaskCleanUpResources           1
#define INCLUDE_vTaskSuspend                    1
#define INCLUDE_vTaskDelayUntil                 1
#define INCLUDE_vTaskDelay                      1
#define INCLUDE_xTaskGetSchedulerState          1
#define INCLUDE_xTaskGetCurrentTaskHandle       1
#define INCLUDE_uxTaskGetStackHighWaterMark     0
#define INCLUDE_xTaskGetIdleTaskHandle          0
#define INCLUDE_xTimerGetTimerDaemonTaskHandle  0
#define INCLUDE_pcTaskGetTaskName               0
#define INCLUDE_eTaskGetState                   0
#define INCLUDE_xEventGroupSetBitFromISR        1
#define INCLUDE_xTimerPendFunctionCall          0

/* A header file that defines trace macro can be included here. */

/* Definitions that map FreeRTOS Port interrupts to libopencm3 vectors */
/* If you need to handle these yourself, comment these out and be sure to call them */
#define vPortSVCHandler sv_call_handler
#define xPortPendSVHandler pend_sv_handler
#define xPortSysTickHandler sys_tick_handler

#endif /* FREERTOS_CONFIG_H */
