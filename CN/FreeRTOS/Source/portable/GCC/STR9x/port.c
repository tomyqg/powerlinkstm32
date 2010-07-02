/*
	FreeRTOS.org V4.7.0 - Copyright (C) 2003-2007 Richard Barry.

	This file is part of the FreeRTOS.org distribution.

	FreeRTOS.org is free software; you can redistribute it and/or modify
	it under the terms of the GNU General Public License as published by
	the Free Software Foundation; either version 2 of the License, or
	(at your option) any later version.

	FreeRTOS.org is distributed in the hope that it will be useful,
	but WITHOUT ANY WARRANTY; without even the implied warranty of
	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
	GNU General Public License for more details.

	You should have received a copy of the GNU General Public License
	along with FreeRTOS.org; if not, write to the Free Software
	Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA

	A special exception to the GPL can be applied should you wish to distribute
	a combined work that includes FreeRTOS.org, without being obliged to provide
	the source code for any proprietary components.  See the licensing section
	of http://www.FreeRTOS.org for full details of how and when the exception
	can be applied.

	***************************************************************************
	See http://www.FreeRTOS.org for documentation, latest information, license
	and contact details.  Please ensure to read the configuration and relevant
	port sections of the online documentation.

	Also see http://www.SafeRTOS.com a version that has been certified for use
	in safety critical systems, plus commercial licensing, development and
	support options.
	***************************************************************************
*/

/*-----------------------------------------------------------
 * Implementation of functions defined in portable.h for the ST STR75x ARM7
 * port.
 *----------------------------------------------------------*/

/* Library includes. */
#include "91x_lib.h"

/* Scheduler includes. */
#include "FreeRTOS.h"
#include "task.h"

#ifndef configUSE_WATCHDOG_TICK
	#error configUSE_WATCHDOG_TICK must be set to either 1 or 0 in FreeRTOSConfig.h to use either the Watchdog or timer 2 to generate the tick interrupt respectively.
#endif

#if configUSE_WATCHDOG_TICK == 0
#define prvSetupTimerInterrupt prvSetupTim2AsTimerinterrupt
#elif configUSE_WATCHDOG_TICK == 1
#define prvSetupTimerInterrupt prvSetupWdgAsTimerinterrupt
#endif

/* Constants required to setup the initial stack. */
#define portINITIAL_SPSR				( ( portSTACK_TYPE ) 0x1f ) /* System mode, ARM mode, interrupts enabled. */
#define portTHUMB_MODE_BIT				( ( portSTACK_TYPE ) 0x20 )
#define portINSTRUCTION_SIZE			( ( portSTACK_TYPE ) 4 )

/* Constants required to handle critical sections. */
#define portNO_CRITICAL_NESTING 		( ( unsigned portLONG ) 0 )

#if configUSE_WATCHDOG_TICK == 0
	/* Used to update the OCR timer register */
	static u16 s_nPulseLength;
#endif

/* VIC interrupt default handler. */
static void prvDefaultHandler( void );

/*-----------------------------------------------------------*/

/* Setup the TB to generate the tick interrupts. */
static void prvSetupTimerInterrupt( void );

/*-----------------------------------------------------------*/

#ifdef THUMB_INTERWORK
/**
 * This function extends the standard FreeRTOS API xTaskCreate to allow the user to mix tasks
 * compiled in ARM mode and tasks compiled in THUMB mode. By default, if the THUMB_INTERWORK
 * macro is defined, the RTOS assume that the task entry function is compile in THUMB mode.
 * Use this function instead of  the standard xTaskCreate to overcome this limitation.
 *
 * For more information about this function look at the standard xTaskCreate documentation.
 *
 * @param pvTaskCode	Pointer to the task entry function. Tasks must be implemented to never return (i.e. continuous loop).
 * @param pcName		A descriptive name for the task. This is mainly used to facilitate debugging. Max length defined by configMAX_TASK_NAME_LEN.
 * @param usStackDepth	The size of the task stack specified as the number of variables the stack can hold - not the number of bytes. For example, if the stack is 16 bits wide and usStackDepth is defined as 100, 200 bytes will be allocated for stack storage. The stack depth multiplied by the stack width must not exceed the maximum value that can be contained in a variable of type size_t.
 * @param pvParameters	Pointer that will be used as the parameter for the task being created.
 * @param uxPriority	The priority at which the task should run.
 * @param uxMode		Provide to the scheduler additional information about the task entry function. Use ARM_MODE if the task entry function is compiled in ARM mode, THUM_MODE otherwise.
 * @param pxCreatedTask	Used to pass back a handle by which the created task can be referenced.
 * @return pdPASS if the task was successfully created and added to a ready list, otherwise an error code defined in the file projdefs.h
 */
portBASE_TYPE xTaskCreateEx( pdTASK_CODE pvTaskCode, const signed portCHAR * const pcName, unsigned portSHORT usStackDepth, void *pvParameters, unsigned portBASE_TYPE uxPriority, unsigned portBASE_TYPE uxMode, xTaskHandle *pxCreatedTask )
{
	xTaskHandle task, *pTask;
	signed portBASE_TYPE res;

	pTask = pxCreatedTask ? pxCreatedTask : &task;
	res = xTaskCreate(pvTaskCode, pcName, usStackDepth, pvParameters, uxPriority, pTask);
	if ((res == pdPASS) && (uxMode == ARM_MODE)) {
		portSTACK_TYPE	*pxTopOfStack = *pTask;
		*((portSTACK_TYPE*)((*pxTopOfStack) + portINSTRUCTION_SIZE)) = portINITIAL_SPSR;
	}

	return res;
}
#endif

/**
 * Initialize the stack of a task to look exactly as if a call to
 * portSAVE_CONTEXT had been called. The registers have to be placed on the stack in
 * the order that the port expects to find them.
 *
 * @param pxTopOfStack	Pointer to the task stack.
 * @param pxCode		Pointer to the task control function.
 * @param pvParameters	Pointer to the parameter to be passed to the task entry function.
 * @return 				The new top of the stack.
 */
portSTACK_TYPE *pxPortInitialiseStack( portSTACK_TYPE *pxTopOfStack, pdTASK_CODE pxCode, void *pvParameters )
{
portSTACK_TYPE *pxOriginalTOS;

	pxOriginalTOS = pxTopOfStack;

	/* Setup the initial stack of the task.  The stack is set exactly as
	expected by the portRESTORE_CONTEXT() macro. */

	/* First on the stack is the return address - which in this case is the
	start of the task.  The offset is added to make the return address appear
	as it would within an IRQ ISR. */
	*pxTopOfStack = ( portSTACK_TYPE ) pxCode + portINSTRUCTION_SIZE;
	pxTopOfStack--;

	*pxTopOfStack = ( portSTACK_TYPE ) 0xaaaaaaaa;	/* R14 */
	pxTopOfStack--;
	*pxTopOfStack = ( portSTACK_TYPE ) pxOriginalTOS; /* Stack used when task starts goes in R13. */
	pxTopOfStack--;
	*pxTopOfStack = ( portSTACK_TYPE ) 0x12121212;	/* R12 */
	pxTopOfStack--;
	*pxTopOfStack = ( portSTACK_TYPE ) 0x11111111;	/* R11 */
	pxTopOfStack--;
	*pxTopOfStack = ( portSTACK_TYPE ) 0x10101010;	/* R10 */
	pxTopOfStack--;
	*pxTopOfStack = ( portSTACK_TYPE ) 0x09090909;	/* R9 */
	pxTopOfStack--;
	*pxTopOfStack = ( portSTACK_TYPE ) 0x08080808;	/* R8 */
	pxTopOfStack--;
	*pxTopOfStack = ( portSTACK_TYPE ) 0x07070707;	/* R7 */
	pxTopOfStack--;
	*pxTopOfStack = ( portSTACK_TYPE ) 0x06060606;	/* R6 */
	pxTopOfStack--;
	*pxTopOfStack = ( portSTACK_TYPE ) 0x05050505;	/* R5 */
	pxTopOfStack--;
	*pxTopOfStack = ( portSTACK_TYPE ) 0x04040404;	/* R4 */
	pxTopOfStack--;
	*pxTopOfStack = ( portSTACK_TYPE ) 0x03030303;	/* R3 */
	pxTopOfStack--;
	*pxTopOfStack = ( portSTACK_TYPE ) 0x02020202;	/* R2 */
	pxTopOfStack--;
	*pxTopOfStack = ( portSTACK_TYPE ) 0x01010101;	/* R1 */
	pxTopOfStack--;

	/* When the task starts is will expect to find the function parameter in
	R0. */
	*pxTopOfStack = ( portSTACK_TYPE ) pvParameters; /* R0 */
	pxTopOfStack--;

	/* The status register is set for system mode, with interrupts enabled. */
	*pxTopOfStack = ( portSTACK_TYPE ) portINITIAL_SPSR;

	#ifdef THUMB_INTERWORK
	{
		/* We want the task to start in thumb mode. */
		*pxTopOfStack |= portTHUMB_MODE_BIT;
	}
	#endif

	pxTopOfStack--;

	/* Interrupt flags cannot always be stored on the stack and will
	instead be stored in a variable, which is then saved as part of the
	tasks context. */
	*pxTopOfStack = portNO_CRITICAL_NESTING;

	return pxTopOfStack;
}
/*-----------------------------------------------------------*/

portBASE_TYPE xPortStartScheduler( void )
{
extern void vPortISRStartFirstTask( void );

	/* Start the timer that generates the tick ISR.  Interrupts are disabled
	here already. */
	prvSetupTimerInterrupt();

	/* Start the first task. */
	vPortISRStartFirstTask();

	/* Should not get here! */
	return 0;
}
/*-----------------------------------------------------------*/

void vPortEndScheduler( void )
{
	/* It is unlikely that the ARM port will require this function as there
	is nothing to return to.  */
}
/*-----------------------------------------------------------*/

#if configUSE_WATCHDOG_TICK == 0

/**
 *
 * @param n
 * @param a
 * @param b
 */
static void prvFindFactors(u32 n, u8 *a, u16 *b)
{
	/* This function is copied from the ST STR7 library and is
	copyright STMicroelectronics.  Reproduced with permission. */

	u16 b0;
	u8 a0;
	long err, err_min=n;


	*a = a0 = ((n-1)/256) + 1;
	*b = b0 = n / *a;

	for (; *a <= 256; (*a)++)
	{
		*b = n / *a;
		err = (long)*a * (long)*b - (long)n;
		if (abs(err) > (*a / 2))
		{
			(*b)++;
			err = (long)*a * (long)*b - (long)n;
		}
		if (abs(err) < abs(err_min))
		{
			err_min = err;
			a0 = *a;
			b0 = *b;
			if (err == 0) break;
		}
	}

	*a = a0;
	*b = b0;
}

/**
 *
 * @param
 */
static void prvSetupTim2AsTimerinterrupt( void )
{
	unsigned portCHAR a;
	unsigned portSHORT b;
	unsigned portLONG n = configCPU_PERIPH_HZ / configTICK_RATE_HZ;

	TIM_InitTypeDef timer;

	SCU_APBPeriphClockConfig( __TIM23, ENABLE );
	TIM_DeInit(TIM2);
	TIM_StructInit(&timer);
	prvFindFactors( n, &a, &b );

	timer.TIM_Mode           = TIM_OCM_CHANNEL_1;
	timer.TIM_OC1_Modes      = TIM_TIMING;
	timer.TIM_Clock_Source   = TIM_CLK_APB;
	timer.TIM_Clock_Edge     = TIM_CLK_EDGE_RISING;
	timer.TIM_Prescaler      = a-1;
	timer.TIM_Pulse_Level_1  = TIM_HIGH;
	timer.TIM_Pulse_Length_1 = s_nPulseLength  = b-1;

	TIM_Init (TIM2, &timer);
	TIM_ITConfig(TIM2, TIM_IT_OC1, ENABLE);
	/* Configure the VIC for the WDG interrupt. */
	VIC_Config( TIM2_ITLine, VIC_IRQ, 10 );
	VIC_ITCmd( TIM2_ITLine, ENABLE );

	/* Install the default handlers for both VIC's. */
	VIC0->DVAR = ( unsigned portLONG ) prvDefaultHandler;
	VIC1->DVAR = ( unsigned portLONG ) prvDefaultHandler;

	TIM_CounterCmd(TIM2, TIM_CLEAR);
	TIM_CounterCmd(TIM2, TIM_START);
}

/**
 *
 * @param
 */
void TIM2_IRQHandler( void )
{
	/* Reset the timer counter to avioid overflow. */
	TIM2->OC1R += s_nPulseLength;

	/* Increment the tick counter. */
	vTaskIncrementTick();

	#if configUSE_PREEMPTION == 1
	{
		/* The new tick value might unblock a task.  Ensure the highest task that
		is ready to execute is the task that will execute when the tick ISR
		exits. */
		vTaskSwitchContext();
	}
	#endif

	/* Clear the interrupt in the watchdog. */
	TIM2->SR &= ~TIM_FLAG_OC1;
}

#elif configUSE_WATCHDOG_TICK == 1

/**
 *
 * @param
 */
static void prvFindFactors(u32 n, u16 *a, u32 *b)
{
	/* This function is copied from the ST STR7 library and is
	copyright STMicroelectronics.  Reproduced with permission. */

	u32 b0;
	u16 a0;
	long err, err_min=n;

	*a = a0 = ((n-1)/65536ul) + 1;
	*b = b0 = n / *a;

	for (; *a <= 256; (*a)++)
	{
		*b = n / *a;
		err = (long)*a * (long)*b - (long)n;
		if (abs(err) > (*a / 2))
		{
			(*b)++;
			err = (long)*a * (long)*b - (long)n;
		}
		if (abs(err) < abs(err_min))
		{
			err_min = err;
			a0 = *a;
			b0 = *b;
			if (err == 0) break;
		}
	}

	*a = a0;
	*b = b0;
}

/**
 *
 * @param
 */
static void prvSetupWdgAsTimerinterrupt( void )
{
WDG_InitTypeDef xWdg;
unsigned portSHORT a;
unsigned portLONG n = configCPU_PERIPH_HZ / configTICK_RATE_HZ, b;

	/* Configure the watchdog as a free running timer that generates a
	periodic interrupt. */

	SCU_APBPeriphClockConfig( __WDG, ENABLE );
	WDG_DeInit();
	WDG_StructInit(&xWdg);
	prvFindFactors( n, &a, &b );
	xWdg.WDG_Prescaler = a - 1;
	xWdg.WDG_Preload = b - 1;
	WDG_Init( &xWdg );
	WDG_ITConfig(ENABLE);

	/* Configure the VIC for the WDG interrupt. */
	VIC_Config( WDG_ITLine, VIC_IRQ, 10 );
	VIC_ITCmd( WDG_ITLine, ENABLE );

	/* Install the default handlers for both VIC's. */
	VIC0->DVAR = ( unsigned portLONG ) prvDefaultHandler;
	VIC1->DVAR = ( unsigned portLONG ) prvDefaultHandler;

	WDG_TimerModeCmd(ENABLE);
}

/**
 *
 * @param
 */
void WDG_IRQHandler( void )
{
	{
		/* Increment the tick counter. */
		vTaskIncrementTick();

		#if configUSE_PREEMPTION == 1
		{
			/* The new tick value might unblock a task.  Ensure the highest task that
			is ready to execute is the task that will execute when the tick ISR
			exits. */
			vTaskSwitchContext();
		}
		#endif /* configUSE_PREEMPTION. */

		/* Clear the interrupt in the watchdog. */
		WDG->SR &= ~0x0001;
	}
}

#endif

static void prvDefaultHandler( void )
{
}
