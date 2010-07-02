/*
	FreeRTOS.org V5.4.2 - Copyright (C) 2003-2009 Richard Barry.

	This file is part of the FreeRTOS.org distribution.

	FreeRTOS.org is free software; you can redistribute it and/or modify it
	under the terms of the GNU General Public License (version 2) as published
	by the Free Software Foundation and modified by the FreeRTOS exception.
	**NOTE** The exception to the GPL is included to allow you to distribute a
	combined work that includes FreeRTOS.org without being obliged to provide
	the source code for any proprietary components.  Alternative commercial
	license and support terms are also available upon request.  See the
	licensing section of http://www.FreeRTOS.org for full details.

	FreeRTOS.org is distributed in the hope that it will be useful,	but WITHOUT
	ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
	FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
	more details.

	You should have received a copy of the GNU General Public License along
	with FreeRTOS.org; if not, write to the Free Software Foundation, Inc., 59
	Temple Place, Suite 330, Boston, MA  02111-1307  USA.


	***************************************************************************
	*                                                                         *
	* Get the FreeRTOS eBook!  See http://www.FreeRTOS.org/Documentation      *
	*                                                                         *
	* This is a concise, step by step, 'hands on' guide that describes both   *
	* general multitasking concepts and FreeRTOS specifics. It presents and   *
	* explains numerous examples that are written using the FreeRTOS API.     *
	* Full source code for all the examples is provided in an accompanying    *
	* .zip file.                                                              *
	*                                                                         *
	***************************************************************************

	1 tab == 4 spaces!

	Please ensure to read the configuration and relevant port sections of the
	online documentation.

	http://www.FreeRTOS.org - Documentation, latest information, license and
	contact details.

	http://www.SafeRTOS.com - A version that is certified for use in safety
	critical systems.

	http://www.OpenRTOS.com - Commercial support, development, porting,
	licensing and training services.
*/



/* Standard includes. */
#include <stdio.h>



/* Scheduler includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "lcdTask.h"
/* Library includes. */
#include "stm32f10x.h"
#include "stm32f10x_it.h"

#include "PushButton.h"

#include "lcdTask.h"
#include "STCode/STM3210c_eval_lcd.h"
#include "global_FreeRTOS.h"
#include "drvTask.h"
#include "EplApiFreeRTOS.h"

/*
 * Configure the hardware for the demo.
 */
#define LCD_QUEUE_SIZE	( 8 )

static void prvSetupHardware( void );

xQueueHandle g_lcdQueue;
xQueueHandle eventQueue;
xQueueHandle ipcCommandsToProcessQueue;
xQueueHandle ipcCommandsProcessedQueue;

void ledFunction();


/*
 * Configures the high frequency timers - those used to measure the timing
 * jitter while the real time kernel is executing.
 */
extern void vSetupHighFrequencyTimer( void );

/**
 * External dependence needed by printf implementation. Write a character to standard out.
 *
 * @param c Specifies the character to be written.
 * @return Returns the character written. No error conditions are managed.
 */
int putChar( int ch );
portBASE_TYPE xHighPrior=pdFALSE;
/*-----------------------------------------------------------*/
#include "stdio.h"
#include "EplTimer.h"



int main( void )
{

	GPIO_InitTypeDef GPIO_InitStructure;

	int x=65;
    char szBuffer[10];

    g_lcdQueue = xQueueCreate( LCD_QUEUE_SIZE, sizeof( LcdMsg ) );
    eventQueue = xQueueCreate( EVENT_QUEUE_SIZE, sizeof(eventQueueElement));

	prvSetupHardware();
	//initializing led array
	vLedHandlerInitialise();

	xTaskCreate(ledFunction, "ledtask", configMINIMAL_STACK_SIZE, NULL, configDRVTASK_PRIORITY+1 , NULL);

	drvStartTask(NULL);

    /* Start the scheduler. */
	vTaskStartScheduler();

    /* Will only get here if there was insufficient memory to create the idle
    task.  The idle task is created within vTaskStartScheduler(). */
	for(;;)
	{
	}
}
/*-----------------------------------------------------------*/

static void prvSetupHardware( void )
{
	SystemInit();

	/* Enable GPIOA, GPIOB, GPIOC, GPIOD, GPIOE and AFIO clocks */
	RCC_APB2PeriphClockCmd(	RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB |RCC_APB2Periph_GPIOC
							| RCC_APB2Periph_GPIOD | RCC_APB2Periph_GPIOE | RCC_APB2Periph_AFIO, ENABLE );

	/* Set the Vector Table base address at 0x08000000 */
	NVIC_SetVectorTable( NVIC_VectTab_FLASH, 0x0 );

	NVIC_PriorityGroupConfig( NVIC_PriorityGroup_4 );

	/* Configure HCLK clock as SysTick clock source. */
	SysTick_CLKSourceConfig( SysTick_CLKSource_HCLK );

	PushButtonInit(Button_TAMPER);
	PushButtonInit(Button_KEY);


}


void ledFunction() {
	for (;;) {
		vTaskDelay( 250 / portTICK_RATE_MS );
		ledToggle2();
	}

}



int putChar(int ch)
{
	return ch;
}

void vApplicationTickHook(void)
{

}

void vApplicationStackOverflowHook( xTaskHandle *pxTask, signed portCHAR *pcTaskName ){
	for(;;);
}


