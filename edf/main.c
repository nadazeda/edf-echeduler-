/*
 * FreeRTOS Kernel V10.2.0
 * Copyright (C) 2019 Amazon.com, Inc. or its affiliates.  All Rights Reserved.
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

/* 
	NOTE : Tasks run in system mode and the scheduler runs in Supervisor mode.
	The processor MUST be in supervisor mode when vTaskStartScheduler is 
	called.  The demo applications included in the FreeRTOS.org download switch
	to supervisor mode prior to main being called.  If you are not using one of
	these demo application projects then ensure Supervisor mode is used.
*/


/*
 * Creates all the demo application tasks, then starts the scheduler.  The WEB
 * documentation provides more details of the demo application tasks.
 * 
 * Main.c also creates a task called "Check".  This only executes every three 
 * seconds but has the highest priority so is guaranteed to get processor time.  
 * Its main function is to check that all the other tasks are still operational.
 * Each task (other than the "flash" tasks) maintains a unique count that is 
 * incremented each time the task successfully completes its function.  Should 
 * any error occur within such a task the count is permanently halted.  The 
 * check task inspects the count of each task to ensure it has changed since
 * the last time the check task executed.  If all the count variables have 
 * changed all the tasks are still executing error free, and the check task
 * toggles the onboard LED.  Should any task contain an error at any time 
 * the LED toggle rate will change from 3 seconds to 500ms.
 *
 */

/* Standard includes. */
#include <stdlib.h>
#include <stdio.h>
#include <string.h>

/* Scheduler includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "lpc21xx.h"
#include "semphr.h"
#include "event_groups.h"
#include "queue.h"

/* Peripheral includes. */
#include "serial.h"
#include "GPIO.h"


/*-----------------------------------------------------------*/

/* Constants to setup I/O and processor. */
#define mainBUS_CLK_FULL	( ( unsigned char ) 0x01 )

/* Constants for the ComTest demo application tasks. */
#define mainCOM_TEST_BAUD_RATE	( ( unsigned long ) 115200 )
	
TaskHandle_t button1taskhandler = NULL;
TaskHandle_t button2taskhandler = NULL;
TaskHandle_t Periodic_Transmittertaskhandler =NULL;
TaskHandle_t Uart_Receivertaskhandler =NULL;
TaskHandle_t Load_1_Simulationtaskhandler =NULL;
TaskHandle_t Load_2_Simulationtaskhandler =NULL;


int tsk1_IN = 0, tsk1_OUT = 0, tsk1_TOT;
int tsk2_IN = 0, tsk2_OUT = 0, tsk2_TOT;
int tsk3_IN = 0, tsk3_OUT = 0, tsk3_TOT;
int tsk4_IN = 0, tsk4_OUT = 0, tsk4_TOT;
int tsk5_IN = 0, tsk5_OUT = 0, tsk5_TOT;
int tsk6_IN = 0, tsk6_OUT = 0, tsk6_TOT;
int sys_time = 0;
int cpu_load = 0;

 pinState_t button1_state;
 pinState_t button2_state;
 pinState_t button1_last_state=PIN_IS_LOW;
 pinState_t button2_last_state=PIN_IS_LOW;
 pinedge_t  button1=FALLING_EDGE;
 pinedge_t  button2=FALLING_EDGE;
 char transmit[10]="data_send";



char runTimeStatsBuf[200];

QueueHandle_t xQueue1,xQueue2,xQueue3;


/*
 * Configure the processor for use with the Keil demo board.  This is very
 * minimal as most of the setup is managed by the settings in the project
 * file.
 */
static void prvSetupHardware( void );
/*-----------------------------------------------------------*/



/* Task to be created. */
void Button_1_Monitor ( void * pvParameters )
{
	
	int i;
   unsigned int xLastWakeTime = xTaskGetTickCount();
    vTaskSetApplicationTaskTag(NULL,(void *) 1);
    for( ;; )
    { 
			for(i=0;i<10000;i++)
			{
				i=i;
			}
			button1_state = GPIO_read(PORT_0,PIN10);
			if(button1_state != button1_last_state)  // if the state has changed
			{
				if(button1_state==PIN_IS_HIGH)
				{
					/* rising edge*/
					button1=RISING_EDGE;
				}
				else 
				{
					/* falling edge*/
					button1=FALLING_EDGE;
				}
			}
			
				button1_last_state=button1_state;
			 xQueueSend( xQueue1,&button1,portMAX_DELAY );
        
	    vTaskDelayUntil(&xLastWakeTime, 50);
			/*IDLE TASK*/
			GPIO_write(PORT_0,PIN0,PIN_IS_LOW);
    }
}

void Button_2_Monitor ( void * pvParameters )
{
 //button2_last_state=0;  
int i;
   unsigned int xLastWakeTime = xTaskGetTickCount();
vTaskSetApplicationTaskTag(NULL,(void *) 2);
    for( ;; )
    { 
			for(i=0;i<20000;i++)
			{
				i=i;
			}
			button2_state = GPIO_read(PORT_0,PIN9);
			if(button2_state!=button2_last_state)  // if the state has changed
			{
				if(button2_state==PIN_IS_HIGH)
				{
					/* rising edge*/
					button2=RISING_EDGE;
				}
				else 
				{
					/* falling edge*/
					button2=FALLING_EDGE;
				}
			}
			
				button2_last_state=button2_state;  
			 xQueueSend( xQueue2,&button2,portMAX_DELAY );
	    vTaskDelayUntil(&xLastWakeTime, 50);
			/*IDLE TASK*/
			GPIO_write(PORT_0,PIN0,PIN_IS_LOW);
    }
}

void Periodic_Transmitter ( void * pvParameters )
{
	int i;
    unsigned int xLastWakeTime = xTaskGetTickCount();
vTaskSetApplicationTaskTag(NULL,(void *) 3);
    for( ;; )
    { 
			for(i=0;i<30000;i++)
			{
				i=i;
			}
   xQueueSend( xQueue3,&transmit,portMAX_DELAY );
	    vTaskDelayUntil(&xLastWakeTime, 100);
			/*IDLE TASK*/
			GPIO_write(PORT_0,PIN0,PIN_IS_LOW);
     }
}


void Uart_Receiver ( void * pvParameters )
{
   	int i;
   unsigned int xLastWakeTime = xTaskGetTickCount();
vTaskSetApplicationTaskTag(NULL,(void *) 4);
    for( ;; )
    { 
			for(i=0;i<10000;i++)
			{
				i=i;
			}
			
			xQueueReceive(  xQueue1,&button1,portMAX_DELAY );
			xQueueReceive(  xQueue2,&button2,portMAX_DELAY );
			xQueueReceive(  xQueue3,&transmit,portMAX_DELAY );
   
	    vTaskDelayUntil(&xLastWakeTime, 20);
			/*IDLE TASK*/
			GPIO_write(PORT_0,PIN0,PIN_IS_LOW);
		}
		
}

void Load_1_Simulation ( void * pvParameters )
{  int i;
	unsigned int xLastWakeTime = xTaskGetTickCount();
	vTaskSetApplicationTaskTag(NULL,(void *) 5);
    for( ;; )
    { 
			for(i=0;i<34000;i++)
			{
				i=i;
			}
    // GPIO_write(PORT_0,PIN2,PIN_IS_LOW);
	vTaskDelayUntil(&xLastWakeTime, 10);
			//GPIO_write(PORT_0,PIN2,PIN_IS_HIGH);
				/*IDLE TASK*/
			GPIO_write(PORT_0,PIN0,PIN_IS_LOW);
    }
}

void Load_2_Simulation ( void * pvParameters )
{
	 int i;
  unsigned int xLastWakeTime = xTaskGetTickCount();
vTaskSetApplicationTaskTag(NULL,(void *) 6);
    for( ;; )
    { 
			for(i=0;i<75990;i++)
			{
				i=i;
			}
   
	    vTaskDelayUntil(&xLastWakeTime, 100);
			/*IDLE TASK*/
			GPIO_write(PORT_0,PIN0,PIN_IS_LOW);
	   
    }
}



/* implement tick hook */
void vApplicationTickHook( void )
{  
	/* write code here */
	GPIO_write(PORT_0,PIN7,PIN_IS_HIGH);
	GPIO_write(PORT_0,PIN7,PIN_IS_LOW);

}

/* implement idel hook */
void vApplicationIdleHook( void )
{  
	/* write code here */
	GPIO_write(PORT_0,PIN0,PIN_IS_HIGH);
	
	
}

/*
 * Application entry point:
 * Starts all the other tasks, then starts the scheduler. 
 */
int main( void )
{
	/* Setup the hardware for use with the Keil demo board. */
	prvSetupHardware();
	xQueue1 = xQueueCreate( 10, sizeof( char ) );
	xQueue2 = xQueueCreate( 10, sizeof( char ) );
	xQueue3 = xQueueCreate( 10, sizeof( char ) );

		
    /* Create Tasks here */
	/* Create the task, storing the handle. */
xTaskPeriodicCreate(
                    Button_1_Monitor,       /* Function that implements the task. */
                    "Button_1_Monitor",          /* Text name for the task. */
                    100,      /* Stack size in words, not bytes. */
                    ( void * ) 0,    /* Parameter passed into the task. */
                    1,/* Priority at which the task is created. */
                   &button1taskhandler,
                    50										);      /* Used to pass out the created task's handle. */
										
										
xTaskPeriodicCreate(
                    Button_2_Monitor,       /* Function that implements the task. */
                    "Button_2_Monitor",          /* Text name for the task. */
                    100,      /* Stack size in words, not bytes. */
                    ( void * ) 0,    /* Parameter passed into the task. */
                    2,/* Priority at which the task is created. */
                   &button2taskhandler,
                   50										);      /* Used to pass out the created task's handle. */	
										
xTaskPeriodicCreate(
                    Periodic_Transmitter,       /* Function that implements the task. */
                    "Periodic_Transmitter",          /* Text name for the task. */
                    100,      /* Stack size in words, not bytes. */
                    ( void * ) 0,    /* Parameter passed into the task. */
                    3,/* Priority at which the task is created. */
                   &Periodic_Transmittertaskhandler,
                    100										);      /* Used to pass out the created task's handle. */	
										
xTaskPeriodicCreate(
                    Uart_Receiver,       /* Function that implements the task. */
                    "Uart_Receiver",          /* Text name for the task. */
                    100,      /* Stack size in words, not bytes. */
                    ( void * ) 0,    /* Parameter passed into the task. */
                    4,/* Priority at which the task is created. */
                   &Uart_Receivertaskhandler,
                    20										);      /* Used to pass out the created task's handle. */

	
xTaskPeriodicCreate(
                    Load_1_Simulation,       /* Function that implements the task. */
                    "Load_1_Simulation",          /* Text name for the task. */
                    100,      /* Stack size in words, not bytes. */
                    ( void * ) 0,    /* Parameter passed into the task. */
                    5,/* Priority at which the task is created. */
                   &Load_1_Simulationtaskhandler,
                   10	);      /* Used to pass out the created task's handle. */

										
										
										
xTaskPeriodicCreate(
                    Load_2_Simulation,       /* Function that implements the task. */
                    "Load_2_Simulation",          /* Text name for the task. */
                    100,      /* Stack size in words, not bytes. */
                    ( void * ) 0,    /* Parameter passed into the task. */
                    6,/* Priority at which the task is created. */
                   &Load_2_Simulationtaskhandler,
                    100										);      /* Used to pass out the created task's handle. */										

	/* Now all the tasks have been started - start the scheduler.

	NOTE : Tasks run in system mode and the scheduler runs in Supervisor mode.
	The processor MUST be in supervisor mode when vTaskStartScheduler is 
	called.  The demo applications included in the FreeRTOS.org download switch
	to supervisor mode prior to main being called.  If you are not using one of
	these demo application projects then ensure Supervisor mode is used here. */
	vTaskStartScheduler();

	/* Should never reach here!  If you do then there was not enough heap
	available for the idle task to be created. */
	for( ;; );
}
/*-----------------------------------------------------------*/
/*-----------------------------------------------------------*/

void timer1Reset(void)
{
	T1TCR |= 0x2;
	T1TCR &= ~0x2;
}
static void configTimer1(void)
{
	T1PR = 1000;
	T1TCR |= 0x1;
}

static void prvSetupHardware( void )
{
	/* Perform the hardware setup required.  This is minimal as most of the
	setup is managed by the settings in the project file. */

	/* Configure UART */
	xSerialPortInitMinimal(mainCOM_TEST_BAUD_RATE);

	/* Configure GPIO */
	GPIO_init();
	
	/* Configure Trace timer 1 and read T1TC to get current tick*/
	configTimer1();

	/* Setup the peripheral bus to be the same as the PLL output. */
	VPBDIV = mainBUS_CLK_FULL;
}
/*-----------------------------------------------------------*/


