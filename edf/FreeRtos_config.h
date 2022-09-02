
#ifndef FREERTOS_CONFIG_H
#define FREERTOS_CONFIG_H

#include <lpc21xx.h>
#include "GPIO.h"

/**/
extern int tsk1_IN, tsk1_OUT, tsk1_TOT;
extern int tsk2_IN, tsk2_OUT, tsk2_TOT;
extern int tsk3_IN, tsk3_OUT, tsk3_TOT;
extern int tsk4_IN , tsk4_OUT , tsk4_TOT;
extern int tsk5_IN , tsk5_OUT , tsk5_TOT;
extern int tsk6_IN , tsk6_OUT , tsk6_TOT;
extern int sys_time;
extern int cpu_load;

/*-----------------------------------------------------------
 * Application specific definitions.
 *
 * These definitions should be adjusted for your particular hardware and
 * application requirements.
 *
 * THESE PARAMETERS ARE DESCRIBED WITHIN THE 'CONFIGURATION' SECTION OF THE
 * FreeRTOS API DOCUMENTATION AVAILABLE ON THE FreeRTOS.org WEB SITE. 
 *
 * See http://www.freertos.org/a00110.html
 *----------------------------------------------------------*/

#define configUSE_PREEMPTION		1
#define configUSE_IDLE_HOOK			1
#define configUSE_TICK_HOOK			1
#define configCPU_CLOCK_HZ			( ( unsigned long ) 60000000 )	/* =12.0MHz xtal multiplied by 5 using the PLL. */
#define configTICK_RATE_HZ			( ( TickType_t ) 1000 )
#define configMAX_PRIORITIES		( 8 )
#define configMINIMAL_STACK_SIZE	( ( unsigned short ) 90 )
#define configTOTAL_HEAP_SIZE		( ( size_t ) 13 * 1024 )
#define configMAX_TASK_NAME_LEN		( 8 )
#define configUSE_TRACE_FACILITY	1
#define configUSE_16_BIT_TICKS		0
#define configIDLE_SHOULD_YIELD		1

#define configUSE_MUTEXES 				1
#define configUSE_APPLICATION_TASK_TAG 	1

#define configQUEUE_REGISTRY_SIZE 	0

/* Co-routine definitions. */
#define configUSE_CO_ROUTINES 		0
#define configMAX_CO_ROUTINE_PRIORITIES ( 2 )

/* Set the following definitions to 1 to include the API function, or zero
to exclude the API function. */

#define INCLUDE_vTaskPrioritySet		1
#define INCLUDE_uxTaskPriorityGet		1
#define INCLUDE_vTaskDelete				1
#define INCLUDE_vTaskCleanUpResources	0
#define INCLUDE_vTaskSuspend			1
#define INCLUDE_vTaskDelayUntil			1
#define INCLUDE_vTaskDelay				1


#define configUSE_EDF_SCHEDULER 		1

/*Trace Hooks*/

#define traceTASK_SWITCHED_OUT() do\
								{\
									if((int)pxCurrentTCB->pxTaskTag == 1)\
									{\
										GPIO_write(PORT_0,PIN1,PIN_IS_LOW) ;\
										tsk1_OUT = T1TC;\
										tsk1_TOT += (tsk1_OUT - tsk1_IN);\
									}\
									else if((int)pxCurrentTCB->pxTaskTag == 2)\
									{\
										GPIO_write(PORT_0,PIN2,PIN_IS_LOW) ;\
										tsk2_OUT = T1TC;\
										tsk2_TOT += (tsk2_OUT - tsk2_IN);\
									}\
									else if((int)pxCurrentTCB->pxTaskTag == 3)\
									{\
										GPIO_write(PORT_0,PIN3,PIN_IS_LOW) ;\
										tsk3_OUT = T1TC;\
										tsk3_TOT += (tsk3_OUT - tsk3_IN);\
									}\
										else if((int)pxCurrentTCB->pxTaskTag == 4)\
									{\
										GPIO_write(PORT_0,PIN4,PIN_IS_LOW) ;\
										tsk4_OUT = T1TC;\
										tsk4_TOT += (tsk4_OUT - tsk4_IN);\
									}\
									else if((int)pxCurrentTCB->pxTaskTag == 5)\
									{\
										GPIO_write(PORT_0,PIN5,PIN_IS_LOW) ;\
										tsk5_OUT = T1TC;\
										tsk5_TOT += (tsk5_OUT - tsk5_IN);\
									}\
										else if((int)pxCurrentTCB->pxTaskTag == 6)\
									{\
										GPIO_write(PORT_0,PIN6,PIN_IS_LOW) ;\
										tsk6_OUT = T1TC;\
										tsk6_TOT += (tsk6_OUT - tsk6_IN);\
									}\
									sys_time=T1TC;\
									cpu_load= ( (tsk1_TOT + tsk2_TOT + tsk3_TOT+tsk4_TOT+tsk5_TOT+tsk6_TOT) / (float) sys_time )*100;\
								}while(0)

#define traceTASK_SWITCHED_IN() do\
								{\
									if((int)pxCurrentTCB->pxTaskTag == 1)\
									{\
										GPIO_write(PORT_0,PIN1,PIN_IS_HIGH) ;\
										tsk1_IN = T1TC ;\
									}\
									else if((int)pxCurrentTCB->pxTaskTag == 2)\
									{\
										GPIO_write(PORT_0,PIN2,PIN_IS_HIGH) ;\
										tsk2_IN = T1TC ;\
									}\
									else if((int)pxCurrentTCB->pxTaskTag == 3)\
									{\
										GPIO_write(PORT_0,PIN3,PIN_IS_HIGH) ;\
										tsk3_IN = T1TC ;\
									}\
									else if((int)pxCurrentTCB->pxTaskTag == 4)\
									{\
										GPIO_write(PORT_0,PIN4,PIN_IS_HIGH) ;\
										tsk4_IN = T1TC ;\
									}\
									else if((int)pxCurrentTCB->pxTaskTag == 5)\
									{\
										GPIO_write(PORT_0,PIN5,PIN_IS_HIGH) ;\
										tsk5_IN = T1TC ;\
									}\
									else if((int)pxCurrentTCB->pxTaskTag == 6)\
									{\
										GPIO_write(PORT_0,PIN6,PIN_IS_HIGH) ;\
										tsk6_IN = T1TC ;\
									}\
								}while(0)

/*Configure Run-Time Stats*/
#define configUSE_STATS_FORMATTING_FUNCTIONS 1
#define configGENERATE_RUN_TIME_STATS 1
#define portCONFIGURE_TIMER_FOR_RUN_TIME_STATS()
#define portGET_RUN_TIME_COUNTER_VALUE() (T1TC)



#endif /* FREERTOS_CONFIG_H */